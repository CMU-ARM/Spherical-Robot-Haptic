# Spherical Robot Haptic

COPYRIGHT(C) 2018 - CMU Assitive Robots Lab - Code released under MIT.
Contact - Zhi - zhi.tan@ri.cmu.edu

This repository contains the code that ran the user studies reported in *Haptic Interaction for Human-Robot Communication Using a Spherical Robot* by Tan and Steinfeld (2018). Due to the experimental nature of the work, the code does not meet industry coding guidelines / best practices and would need modifications to work on different machines. We provide the basic system requirements and brief description for each file.

## System Requirements 
1. Ubuntu Linux system with
2. Python 3
    1. python package [sphero_sprk](https://github.com/CMU-ARM/sphero_sprk)
3. Sphero SPRK+

## Code files
#### SpheroLooper.py
This is python object that has the calibration, heading tracking, and generating different kinestatic haptic signals. The main ideas is a control loop that allow us to switch direction and heading of the internal unit.  
#### gui_control.py
This file contains the code that run the GUI used in the user study. The different signals can also be described as a list to be executed, these signals (in list form) are stored in `encoding.yaml`. The different signals reported in the paper are store there.
#### Tools.py
This file provides basic math function that are used in the system. 