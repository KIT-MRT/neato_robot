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

 * The driver has been update to support Neato's Botvac D5 Connected, most functionality should work with other Neato's too.
 * It publishes most sensor data including distance sensors, magnetic sensors, acceleration data, etc. 
 * and contains elementary safety features: stopping at walls, stairs and Neato's magnetic strips.
 * LDS, Sounds, and Leds can be set via service call
 * This ROS node works with Kinetic. 
 * Add config to control the Neato with a joystick.
