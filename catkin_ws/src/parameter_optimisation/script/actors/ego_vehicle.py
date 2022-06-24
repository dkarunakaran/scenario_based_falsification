#!/usr/bin/env python

from abc import abstractmethod

import os
import random
import math
import json
import rospy
import carla
secure_random = random.SystemRandom()

class EgoVehicle:

    def __init__(self, client = None, world = None, spawn_points = None, long_pos = None):
        self.client = client
        self.world = world
        parameter_config = rospy.get_param('parameter_config')
        self.role_name = parameter_config['VUT']['name']
        self.actor_filter = parameter_config['VUT']['model']
        self.sensor_actors = []
        self.blueprint = secure_random.choice(self.world.get_blueprint_library().filter(self.actor_filter))
        self.blueprint.set_attribute('role_name', "{}".format(self.role_name))
        self.spawn_point = None
        self.sensor_definition_file = rospy.get_param('sensor_definition_file')
        self.initial_spawn_point = parameter_config['VUT']['initial_spawn_point']
        self.spawn_points = spawn_points
        self.long_pos = long_pos
        self.player = None
        self.collision_sensor = None
        

        # Set a spawn point
        self.set_spawn_point()

        # Create ego vehicle
        self.create()

    def set_spawn_point(self):
        self.spawn_point = self.spawn_points[self.initial_spawn_point]

        # At this carla pedestrian area, Y is the longitudinal position
        self.spawn_point.location.y += self.long_pos
    
    def get_longitudinal_location(self):
        loc = self.player.get_location()

        # Y is the longitudinal position
        if loc.y < 0:
            return loc.y * -1
        else:
            return loc.y

    def create(self):
        print("**********{} location: {}".format(self.role_name, self.spawn_point))
        self.player = self.world.try_spawn_actor(self.blueprint, self.spawn_point)

        # Read sensors from file
        if not os.path.exists(self.sensor_definition_file):
            raise RuntimeError(
                "Could not read sensor-definition from {}".format(self.sensor_definition_file))
        json_sensors = None
        with open(self.sensor_definition_file) as handle:
            json_sensors = json.loads(handle.read())

        # Set up the sensors
        self.sensor_actors = self.setup_sensors(json_sensors["sensors"])

        # Resetting the longitudinal and lateral postion
        self.spawn_point.location.y -= self.long_pos


    def setup_sensors(self, sensors):
        """
        Create the sensors defined by the user and attach them to the ego-vehicle
        :param sensors: list of sensors
        :return:
        """
        actors = []
        bp_library = self.world.get_blueprint_library()
        sensor_names = []
        for sensor_spec in sensors:
            try:
                collision_sensor_flag = False
                sensor_name = str(sensor_spec['type']) + "/" + str(sensor_spec['id'])
                if sensor_name in sensor_names:
                    rospy.logfatal(
                        "Sensor rolename '{}' is only allowed to be used once.".format(
                            sensor_spec['id']))
                    raise NameError(
                        "Sensor rolename '{}' is only allowed to be used once.".format(
                            sensor_spec['id']))
                sensor_names.append(sensor_name)
                bp = bp_library.find(str(sensor_spec['type']))
                bp.set_attribute('role_name', str(sensor_spec['id']))
                if sensor_spec['type'].startswith('sensor.camera'):
                    bp.set_attribute('image_size_x', str(sensor_spec['width']))
                    bp.set_attribute('image_size_y', str(sensor_spec['height']))
                    bp.set_attribute('fov', str(sensor_spec['fov']))
                    try:
                        bp.set_attribute('sensor_tick', str(sensor_spec['sensor_tick']))
                    except KeyError:
                        pass
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])
                    if sensor_spec['type'].startswith('sensor.camera.rgb'):
                        bp.set_attribute('gamma', str(sensor_spec['gamma']))
                        bp.set_attribute('shutter_speed', str(sensor_spec['shutter_speed']))
                        bp.set_attribute('iso', str(sensor_spec['iso']))
                        bp.set_attribute('fstop', str(sensor_spec['fstop']))
                        bp.set_attribute('min_fstop', str(sensor_spec['min_fstop']))
                        bp.set_attribute('blade_count', str(sensor_spec['blade_count']))
                        bp.set_attribute('exposure_mode', str(sensor_spec['exposure_mode']))
                        bp.set_attribute('exposure_compensation', str(
                            sensor_spec['exposure_compensation']))
                        bp.set_attribute('exposure_min_bright', str(
                            sensor_spec['exposure_min_bright']))
                        bp.set_attribute('exposure_max_bright', str(
                            sensor_spec['exposure_max_bright']))
                        bp.set_attribute('exposure_speed_up', str(sensor_spec['exposure_speed_up']))
                        bp.set_attribute('exposure_speed_down', str(
                            sensor_spec['exposure_speed_down']))
                        bp.set_attribute('calibration_constant', str(
                            sensor_spec['calibration_constant']))
                        bp.set_attribute('focal_distance', str(sensor_spec['focal_distance']))
                        bp.set_attribute('blur_amount', str(sensor_spec['blur_amount']))
                        bp.set_attribute('blur_radius', str(sensor_spec['blur_radius']))
                        bp.set_attribute('motion_blur_intensity', str(
                            sensor_spec['motion_blur_intensity']))
                        bp.set_attribute('motion_blur_max_distortion', str(
                            sensor_spec['motion_blur_max_distortion']))
                        bp.set_attribute('motion_blur_min_object_screen_size', str(
                            sensor_spec['motion_blur_min_object_screen_size']))
                        bp.set_attribute('slope', str(sensor_spec['slope']))
                        bp.set_attribute('toe', str(sensor_spec['toe']))
                        bp.set_attribute('shoulder', str(sensor_spec['shoulder']))
                        bp.set_attribute('black_clip', str(sensor_spec['black_clip']))
                        bp.set_attribute('white_clip', str(sensor_spec['white_clip']))
                        bp.set_attribute('temp', str(sensor_spec['temp']))
                        bp.set_attribute('tint', str(sensor_spec['tint']))
                        bp.set_attribute('chromatic_aberration_intensity', str(
                            sensor_spec['chromatic_aberration_intensity']))
                        bp.set_attribute('chromatic_aberration_offset', str(
                            sensor_spec['chromatic_aberration_offset']))
                        bp.set_attribute('enable_postprocess_effects', str(
                            sensor_spec['enable_postprocess_effects']))
                        bp.set_attribute('lens_circle_falloff', str(
                            sensor_spec['lens_circle_falloff']))
                        bp.set_attribute('lens_circle_multiplier', str(
                            sensor_spec['lens_circle_multiplier']))
                        bp.set_attribute('lens_k', str(sensor_spec['lens_k']))
                        bp.set_attribute('lens_kcube', str(sensor_spec['lens_kcube']))
                        bp.set_attribute('lens_x_size', str(sensor_spec['lens_x_size']))
                        bp.set_attribute('lens_y_size', str(sensor_spec['lens_y_size']))
                elif sensor_spec['type'].startswith('sensor.lidar'):
                    bp.set_attribute('range', str(sensor_spec['range']))
                    bp.set_attribute('rotation_frequency', str(sensor_spec['rotation_frequency']))
                    bp.set_attribute('channels', str(sensor_spec['channels']))
                    bp.set_attribute('upper_fov', str(sensor_spec['upper_fov']))
                    bp.set_attribute('lower_fov', str(sensor_spec['lower_fov']))
                    bp.set_attribute('points_per_second', str(sensor_spec['points_per_second']))
                    try:
                        bp.set_attribute('sensor_tick', str(sensor_spec['sensor_tick']))
                    except KeyError:
                        pass
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])
                elif sensor_spec['type'].startswith('sensor.other.gnss'):
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation()
                    bp.set_attribute('noise_alt_stddev', str(sensor_spec['noise_alt_stddev']))
                    bp.set_attribute('noise_lat_stddev', str(sensor_spec['noise_lat_stddev']))
                    bp.set_attribute('noise_lon_stddev', str(sensor_spec['noise_lon_stddev']))
                    bp.set_attribute('noise_alt_bias', str(sensor_spec['noise_alt_bias']))
                    bp.set_attribute('noise_lat_bias', str(sensor_spec['noise_lat_bias']))
                    bp.set_attribute('noise_lon_bias', str(sensor_spec['noise_lon_bias']))
                elif sensor_spec['type'].startswith('sensor.other.imu'):
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])
                    bp.set_attribute('noise_accel_stddev_x',
                                     str(sensor_spec['noise_accel_stddev_x']))
                    bp.set_attribute('noise_accel_stddev_y',
                                     str(sensor_spec['noise_accel_stddev_y']))
                    bp.set_attribute('noise_accel_stddev_z',
                                     str(sensor_spec['noise_accel_stddev_z']))

                    bp.set_attribute('noise_gyro_stddev_x', str(sensor_spec['noise_gyro_stddev_x']))
                    bp.set_attribute('noise_gyro_stddev_y', str(sensor_spec['noise_gyro_stddev_y']))
                    bp.set_attribute('noise_gyro_stddev_z', str(sensor_spec['noise_gyro_stddev_z']))
                    bp.set_attribute('noise_gyro_bias_x', str(sensor_spec['noise_gyro_bias_x']))
                    bp.set_attribute('noise_gyro_bias_y', str(sensor_spec['noise_gyro_bias_y']))
                    bp.set_attribute('noise_gyro_bias_z', str(sensor_spec['noise_gyro_bias_z']))
                elif sensor_spec['type'].startswith('sensor.other.radar'):
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation(pitch=sensor_spec['pitch'],
                                                     roll=sensor_spec['roll'],
                                                     yaw=sensor_spec['yaw'])

                    bp.set_attribute('horizontal_fov', str(sensor_spec['horizontal_fov']))
                    bp.set_attribute('vertical_fov', str(sensor_spec['vertical_fov']))
                    bp.set_attribute('points_per_second', str(sensor_spec['points_per_second']))
                    bp.set_attribute('range', str(sensor_spec['range']))
                elif sensor_spec['type'].startswith('sensor.other.collision'):
                    sensor_location = carla.Location(x=sensor_spec['x'], y=sensor_spec['y'],
                                                     z=sensor_spec['z'])
                    sensor_rotation = carla.Rotation()
                    collision_sensor_flag = True
            except KeyError as e:
                rospy.logfatal(
                    "Sensor will not be spawned, because sensor spec is invalid: '{}'".format(e))
                raise e

            # create sensor
            sensor_transform = carla.Transform(sensor_location, sensor_rotation)
            sensor = self.world.spawn_actor(bp, sensor_transform,
                                            attach_to=self.player)
            if collision_sensor_flag:
                self.collision_sensor = sensor
            
            actors.append(sensor)

        return actors

    def destroy(self):
        """
        destroy the current ego vehicle and its sensors
        """
        for i, _ in enumerate(self.sensor_actors):
            if self.sensor_actors[i] is not None:
                self.sensor_actors[i].destroy()
                self.sensor_actors[i] = None
        self.sensor_actors = []

        if self.player and self.player.is_alive:
            self.player.destroy()
        self.player = None
        self.collision_sensor = None
        print("EgoVehicle removed")

    



if __name__ == '__main__':
    EgoVehicle()
