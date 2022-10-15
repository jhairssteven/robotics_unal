# Phantom X forward kinematics using ROS

This repository contains python scripts to interact with the Phantom X robot in order to change its position. This is done by changing the configuration space of each one of its joints through the communication with different services and topics in ROS. MATLAB is used to model an plot the robot using  Denavit–Hartenberg parameters.

## Measuring the Phantom X

We measure the length of the links of then PhantomX using a caliber. The measurements taken are:

* Link 1: 14.5 cm
* Link 2: 10.63 cm
* Link 3: 10.65 cm
* Link 4: 8.97 cm

Then,we assign a frame to each joint following the Denavit - Hartemberg convention.

![diagram]("Images/Phantom_dimensions.png")


___

Developed by
[Juan David Díaz García](https://github.com/D4vidDG) and [Steven Gallego](https://github.com/jhairssteven).
