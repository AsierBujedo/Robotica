#!/bin/bash

echo "Updating apt repositories"
sudo apt update;
echo "Executing rosdep commands"
rosdep update;
rosdep install --from-paths src --ignore-src -r -y;
echo "Executing catkin build"
catkin build;
echo "Adding setup.bash source to bashrc"
echo "source /home/laboratorio/ros_workspace/devel/setup.bash;" >> ~/.bashrc
echo "Adding aliases to .bashrc"
echo "alias launch_sim='roslaunch launcher_robots_lab_robotica sim_203.launch';" >> ~/.bashrc;
echo "alias launch_robot='roslaunch launcher_robots_lab_robotica robot_205.launch';" >> ~/.bashrc;
echo "Applying changes"
source ~/.bashrc

