## Neato Drivers

This repository contains ROS drivers for Neato's robotic vacuums.

## Usage
You can check this out into your catkin workspace as follows:

    roscd
    cd src
    git clone https://github.com/jmtatsch/neato_robot.git
    cd ..
    rosdep update
    rosdep install neato_robot
    catkin_make

## Changes in this fork

 * The driver has been update to support a wider range of neato models and firmware versions including the Neato's Botvac line.
 * This ROS node works with Indigo. The required third parameter to rospy.Publisher() has been supplied.