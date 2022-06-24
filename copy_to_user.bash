#!/bin/bash

# Change the user according to your host system as users care copied from the ost system orginally 
user=dhanoop

mkdir -p /home/$user
cd /home/$user
cp /root/.bashrc .
cp -r /root/.ros .
chown -R "$user:$user" .bashrc
chown -R "$user:$user" .ros

cd /home
chown -R "$user:$user" .

