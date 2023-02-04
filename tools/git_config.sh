#!/bin/bash
current_path=$(pwd)
HOME=${current_path%%src*}"src/rosbridge_tester"
echo "cd "$HOME
cd $HOME
user_email="tylee.yeonge@gmail.com"
user_name="tylee-yeonge"
echo "user.email: "$user_email
git config user.email $user_email
git config user.name $user_name
git config credential.helper store
echo "git config done"

