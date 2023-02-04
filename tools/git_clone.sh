#!/bin/bash

package_list=$(cat package_list.txt)
echo $package_list
user_email="tylee@thirarobotics.com"
echo $user_email
current_path=$(pwd)
echo $current_path
# test_str="/home/parallels/Documents/tylee_ws/src/thira_mqtt_client/src/thira_mqtt_client/tools"
HOME=${current_path%%src*}"src/"
echo $HOME
#LIST=($package_list)

for package_name in $package_list
do
    cd $HOME
    echo current package_name :$package_name
    #git clone ssh://git@git.thirabot.com:30003/thirabot/$package_name.git ../$package_name
    git clone https://git.thirabot.com/$package_name.git $package_name    
    clone_path=$HOME$package_name
    echo $clone_path
    cd $clone_path
    git config user.email $user_email
    git config credential.helper store
    echo $package_name" done"
done

#git clone ssh://git@git.thirabot.com:30003/thirabot/amr_msgs.git ../amr_msgs

