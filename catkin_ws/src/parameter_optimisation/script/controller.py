#!/usr/bin/env python

import gym
import rospy
import numpy as np
import tensorflow as tf
from tensorflow.keras.losses import BinaryCrossentropy
from tensorflow.keras.losses import CategoricalCrossentropy

class Controller:

    def __init__(self, search_space = None, resume = False, model_name = None, exploration = None):
        con_settings = rospy.get_param('nas_controller')
        pg_settings = rospy.get_param('policy_gradient')
        self.search_space = search_space
        self.reward = []
        self.state = []
        self.discounted_rewards = []
        self.total_rewards = []
        self.discount_factor = pg_settings['discount_factor']
        self.gamma = float(pg_settings['gamma'])
        self.alpha = float(pg_settings['alpha'])
        self.learning_rate = con_settings['learning_rate']
        self.batch_size = con_settings['batch_size']
        self.epochs = con_settings['epochs']

        # Exploration vs exploitation
        self.exploration = pg_settings['exploration']
        self.exploration_decay = pg_settings['exploration_decay']
        self.exploration_min = pg_settings['exploration_min']
        self.no_explored_step_batch = 0
        self.num_lstm = con_settings['num_lstm']

        # Timesteps determines the input for different architecture type such as one to many, many to one etc.
        self.timestep = con_settings['time_steps']

        # Features determines no.of features per time steps
        self.feature = con_settings['features']
        self.action_shape = self.search_space.value_size
        self.create_policy_gradient_model()
        self.update_theta = True
        self.pg_loss = 0
        self.total_pg_loss = 0
        self.resume = resume
        if self.resume:
            new_model = tf.keras.models.load_model(model_name)
            weights = new_model.get_weights()
            self.model.set_weights(weights)
            self.exploration = exploration
            print("Loaded exploration value: {}".format(self.exploration))
            print("MODELS ARE LOADED!!!")

    def create_policy_gradient_model(self):
        inputs = tf.keras.Input(shape=(self.timestep, self.search_space.size))
        x = tf.keras.layers.LSTM(32, activation='relu', input_shape=(self.timestep, self.search_space.size), return_sequences=True)(inputs)
        x = tf.keras.layers.LSTM(32, activation='relu',return_sequences=True)(x)
        action_list = []
        for index in range(self.search_space.size):
            name = "action{}".format(index)
            action_list.append(tf.keras.layers.Dense(self.search_space.get_parameter_size(index), activation='softmax', name=name)(x))
        
        self.model = tf.keras.Model(inputs=inputs, outputs=action_list)
        print(self.model.summary())
        self.optimizer = tf.keras.optimizers.Adam(lr = self.learning_rate)

        self.loss = []
        for index in range(self.search_space.size):
            name = "action{}".format(index)
            self.loss.append(CategoricalCrossentropy(reduction=tf.keras.losses.Reduction.SUM, name=name))

        self.gradBuffer = self.model.trainable_variables
        for ix,grad in enumerate(self.gradBuffer):
            self.gradBuffer[ix] = grad * 0

    def exploration_decay_progress(self):

        # Exploration/exploitation trade off rules as follows
        self.exploration *= self.exploration_decay
        self.exploration = max(self.exploration_min, self.exploration)
        print("Exploration value got updated")
        
        return self.exploration

    def get_action(self, state):
        
        action = None
        if self.exploration > 0 and np.random.random() < float(self.exploration):

            # Exploration steps
            print("Generating random action to explore")

            #action and hot encoded target
            action = self.search_space.get_random_parameter()
            action = np.array(tf.cast(action, tf.float32, name="explored_action"))
            self.no_explored_step_batch += 1
        else:
            
            #Exploitation steps      
            print("Model is predicting to exploit")
            action = None
            policy = self.model.predict(state)
            for index in range(len(policy)):
                sub_action = np.random.choice(policy[index].shape[2], 1, p=policy[index][0][0])[0]
                sub_action = self.search_space.get_parameter_value(index, sub_action)
                if action is None:
                    action = np.array(sub_action, dtype=np.float32)
                else:
                    action = np.column_stack((action, np.array(sub_action, dtype=np.float32)))

        return action
   

    def remember(self, state, reward):
        
        # Stores observations

        # Input to the controller
        self.state.append(state)

        # Reward of the action taken
        self.reward.append(reward)


    # calculate discounted rewards
    def get_discount_rewards(self, rewards):
        discounted_rewards = np.zeros_like(rewards)
        running_add = 0
        for t in reversed(range(0, len(rewards))):
            running_add = running_add * self.discount_factor + rewards[t]
            discounted_rewards[t] = running_add
        
        # Only one timestep per episode and it's difficult to have discounted reward
        #return discounted_rewards
        return rewards

    def update_policy(self):
        # IN OUR CASE, Where m is the number of different scenarios that the controller samples in one batch and T is the number of
        # hyper-parameters our controller has to predict to compose the scenarios.

        if self.update_theta:
            self.discount_rewards = self.get_discount_rewards(self.reward)
            for index in range(len(self.state)):
                with tf.GradientTape(persistent=True) as tape:
                    state = self.state[index]
                    state_array = tf.keras.backend.get_value(state)
                    state_array = np.array(tf.cast(state_array, tf.float32))
                    
                    # FInding the policy(probablity distribution for the given state using the currrent policy)
                    policy = self.model(state)
                    loss_list = []
                    if len(state_array.shape) == 3:
                        for sub_index in range(state_array.shape[2]):
                            # Need to round otherwise keyerror occur
                            s = round(state_array[0][0][sub_index].item(), 3)
                       
                            # Hot encoded target
                            target = self.search_space.embedding_encode(sub_index, s)

                            # FInding the categorical cross entropy loss
                            CCE = self.loss[sub_index] 
                            loss = CCE(policy[sub_index], target)
                            loss_list.append(loss)
                            print("Policy gradient loss: {}".format(float(loss)))
                            loss_per_episode = tf.keras.backend.get_value(loss)
                            loss_per_episode = loss_per_episode.item()
                            self.total_pg_loss += float(loss_per_episode)
                
                for each in range(len(loss_list)):
                    # Find the gradients
                    grads = tape.gradient(loss_list[each], self.model.trainable_variables)
                    
                    # Updating the gradient wrt to the reward: \delta j(\theta) = sum_batch_or_scenarios_or_trajectory(sum_no_parameters(\alpha * gradient loss * reward(trajectory)))
                    for ix, grad in enumerate(grads):
                        if grad != None:
                            self.gradBuffer[ix] += self.alpha * grad * self.discount_rewards[index]
            
            # Divide by m
            for ix, grad in enumerate(self.gradBuffer):
                self.gradBuffer[ix] += (grad/len(self.state))
        
            # After collecting gradient, we apply and change the weights of our network
            self.optimizer.apply_gradients(zip(self.gradBuffer, self.model.trainable_variables))

            print("Policy parameters \\theta get updated")

    def reset(self):

        # Resetting the gradient value in the buffer
        for ix,grad in enumerate(self.gradBuffer):
            self.gradBuffer[ix] = grad * 0

        # Resetting the batch data
        self.state = []
        self.action = []
        self.reward = []

        # Setting to zero for next batch
        self.no_explored_step_batch = 0

        # Setting to zero for next batch
        self.total_pg_loss = 0

        print("Variables are reset for next batch")

    def save_model(self, one_model = None, multiple_models = None):
        self.model.save(one_model)
        self.model.save(multiple_models)



if __name__ == "__main__":
    Controller()