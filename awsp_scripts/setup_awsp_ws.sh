#!/usr/bin/env bash
cd
echo "Setting up the awsp_repo.."
sleep 1
mkdir awsp_ws
cd awsp_ws
mkdir src
echo "Now catkin_make to init the repo!"
sleep 2
catkin_make
echo "Cloning awsp_ros.git.."
sleep 2
git clone https://github.com/dmicha16/awsp_ros.git
rm -r src
mv awsp_ros/ src/
echo "catkin_make again!"
catkin_make || echo "Looks like catkin_make failed, but the repo is cloned anyways."
echo "Done! Now you can do whatever it is you do!"
