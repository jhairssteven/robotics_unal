# Industrial robotics - part 2 | Inputs and outputs

As the second part of [Lab 1](../lab1/README.md) we extend the robot behaviour by adding Input and Output capabilities, simulated in RobotStudio through the use of RAPPID's conditional statements and loops. The real-world I/O interface comes from a set of buttons and LEDs.

## Table of contents
___
- [Implementing I/O on RobotStudio](#implementing-io-on-robotstudio)
- [Getting started with loops and conditionals](#getting-started-with-loops-and-conditionals)
- [Using old trajectories and implementing new ones](#using-old-trajectories-and-implementing-new-ones)
- [Simulations and physical implementation](#simulations-and-physical-implementation)

## Implementing I/O on RobotStudio
___
<!-- Briefly describe how to create the `input` and `output` signals in RobotStudio. -->

## Getting started with loops and conditionals
___
After some research, [Robotics & Gaming](https://www.youtube.com/watch?v=kRuSMqpowLU) and [BME Teaching](https://www.youtube.com/watch?v=Z6foYAlgE8A) show how to use `while` loops and conditional statements in RobotStudio. The syntax is quite simple and pretty similar to other programming languages. Below you can find their implementation in the project.

```SQL
WHILE DI_1 = 1 OR DI_2 = 1 OR DI_3 = 1 DO
    IF DI_1 = 1 AND DI_2 = 0 THEN
        Home;
        Medium_point;
        SetDO DO_1, 1;
        Path_10;
        Path_20;
        Path_30;
        Medium_point;
        Home;
    ELSEIF DI_2 = 1 AND DI_1 = 0 THEN
        Soft_home;
        Mount_Tool;
        SetDO DO_1, 0;
    ELSEIF DI_3 = 1 THEN
        Home;
    ELSE
    ENDIF
ENDWHILE
```

## Using old trajectories and implementing new ones
___
We use the [trajectory paths](../lab1/RobotStudio%20modules/hor_plane/Module1.mod) developed on part one to write our name's initials on an horizontal plane and we also develop a new one which positions the robot at a convenient posture to easily install our [tool](../lab1/CADs/SATs/ToolM.sat). 
<!-- Below you can find a simplified flow diagram of the code arquitecture. -->

## Simulations and physical implementation
___

### Digital I/O - Simulation

https://user-images.githubusercontent.com/71862429/190883805-9fdef2ee-af37-474d-b7b7-35da6178ca70.mp4


___

Developed by
[Juan David Díaz García](https://github.com/D4vidDG) and [Steven Gallego](https://github.com/jhairssteven).
