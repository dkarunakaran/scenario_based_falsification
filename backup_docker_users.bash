#!/bin/bash

# Creating the backup file for the user information
cp -r /etc/group /etc/group_bkup 
cp -r /etc/passwd /etc/passwd_bkup 
cp -r /etc/shadow /etc/shadow_bkup  



