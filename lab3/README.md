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
We create input and output signals following the steps shown in the image below.
![Creating - IOs](/lab3/images/Creating_IOs.png)

A window wizard opens to create a new signal, we create three inputs and one output.
![Created signals](/lab3/images/Created_signals.png)

Below is a detailed explanation of what each signal represents and how it relates to the electrical board.

- `DI_01`: Digital input 1 (button)
- `DI_02`: Digital input 2 (button)
- `DO_01`: Digital output 1 (LED)

> **Note:** _The final names must have the structure `DI_0m` or `DO_0m`, for inputs and outputs, respectively. This was made 'cause of the way the FlexPendant was configured._

Finally, we start the ***I/O Simulator*** to see and manage our signal's state. Note that we filter by device and select *\<none>* because we didn't assign a device when we created the signals. This way later, we can filter by device and see just our recently created signals.

![IO Simulator](/lab3/images/IO_simulator.png)

## Loops and conditionals
___
After some research, [Robotics & Gaming](https://www.youtube.com/watch?v=kRuSMqpowLU) and [BME Teaching](https://www.youtube.com/watch?v=Z6foYAlgE8A) show how to use `while` loops and conditional statements in RobotStudio. The syntax is quite simple and pretty similar to other programming languages. Below you can find their use in the project. Note the recently created signals (`DI_01` and `DI_02`) are evaluated as boolean variables.

```SQL
WHILE DI_01 = 1 OR DI_02 = 1 OR DI_03 = 1 DO
    IF DI_01 = 1 AND DI_02 = 0 THEN
        Home;
        Medium_point;
        SetDO DO_01, 1;
        Path_10;
        Path_20;
        Path_30;
        Medium_point;
        Home;
    ELSEIF DI_02 = 1 AND DI_01 = 0 THEN
        Soft_home;
        Mount_Tool;
        SetDO DO_01, 0;
    ELSEIF DI_03 = 1 THEN
        Home;
    ELSE
    ENDIF
ENDWHILE
```

Where `SetDO X, Y` sets the `X` digital output to the `Y` boolean state.
    
## Using old trajectories and implementing new ones
___
We use the [trajectory paths](../lab1/RobotStudio%20modules/hor_plane/Module1.mod) developed on part one to write our name's initials on an horizontal plane together with a new trajectory which positions the robot at a convenient posture to easily install our [tool](../lab1/CADs/SATs/ToolM.sat). Below you can find a simplified flow diagram of the code arquitecture.

![Code flow diagram](/lab3/images/code_flow_diagram.png)

## Simulations and physical implementation
___

### Digital I/O - Simulation

https://user-images.githubusercontent.com/71862429/190883805-9fdef2ee-af37-474d-b7b7-35da6178ca70.mp4

## Physical implementation
[Here](/lab3/RobotStudio%20modules/Digital_IO_hor/) you can find the module we used on the physical implementation.
___

Developed by
[Juan David Díaz García](https://github.com/D4vidDG) and [Steven Gallego](https://github.com/jhairssteven).
