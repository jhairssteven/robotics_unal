# Industrial robotics | Part 1

We are using the ABB IRB140 Robot with the RobotStudio software. We develop the necessary robot movements or trajectories to draw (with the aid of a whiteboard marker) our names' initials (i.e: **D** and **S**) on a planar surface and over a 30° inclined plane.

## Table of contents
___
- [Tool CAD design](#tool-cad-design)
- [Tool set up in RobotStudio](#tool-set-up-in-robotstudio)
- [Setting up the environment](#setting-up-the-environment)
- [Getting trajectories with the `auto-path` wizard](#getting-trajectories-with-the-auto-path-wizard)
- [Simulations and physical implementation](#simulations-and-physical-implementation)

## Tool CAD design
___
In the figure below you will find the final tool. It has 5 main components (apart from the whiteboard marker) and was designed such that the base (black-colored component) could be reused in other projects.

![Interior and exterior tool view](/lab1/images/in-out-tool-view.png)

### Material and Physical Properties

 - The tool was 3D printed and its physical properties were calculated according to the printing material and marker's physical characteristics.

 ![Tool physical characteristics](/lab1/images/tool-physical-characteristics.png)

### Considerations
 - The black-colored component (base) was designed taking into account the [Official Robot Tool Flange dimensions](https://library.e.abb.com/public/a7121292272d40a9992a50745fdaa3b2/3HAC041346%20PS%20IRB%20140-en.pdf).
 - The red-colored components were designed so that they prevent the marker's axial rotation and their base design was taken from [here](https://github.com/ariasAleia/RobotStudio_Robotics_Lab4#guide).
 - The whiteboard marker's CAD was taken from [GrabCad](https://grabcad.com/library/expo-marker-1).
- In order to use the CAD in RobotStudio you should export the model in .STL format.
## Tool set up in RobotStudio 
___
 - Import the tool model: `Home > Import geometry > Browse for geometry `
 - Match the tool's origin with RobotStudio's world origin.

    ![Match TCPs](/lab1/images/match_TCPs.png)

- Create a new frame (`Home > Frame > Create Frame`). Set it normal to and at the center of the marker's tip outermost face.
    
    ![New Frame on tip's face center](/lab1/images/new-frame-tip-center.png)
- Create a new Tool object in RobotStudio: `Modeling > Create tool`. Follow the instructions and put in the physical properties obtained before.
- Save the newly created tool as a library so that it can be easily imported on new projects.

    ![Save tool as library](/lab1/images/save-as-library.png)

## Setting up the environment
___
From `Virtual Controller > New Controller...` Add a new controller for the IRB140 robot and set the appropiate RobotWare version. Then on `Import Library > Browse for Library...` import the newly created library and finally attach it to the robot.

![Attach library tool to robot](/lab1/images/attach-library-tool-to-robot.png)


## Getting trajectories with the `auto-path` wizard
___

In order to get our trajectories automatically we first create a CAD model with the desired initials extruded just like the figure below.

![Initials CAD](/lab1/images/initials-cad.png)

After importing and relocating the CAD model to the desired position, we go to `Home > Path > AutoPath`. We create the targets selecting the CAD's edges accordingly, putting special care in the **Approach** and **Depart** options. In our example we set both to 50mm. We also check the **Circular** option to get a better path aproximation and to reduce the number of generated targets.


Then, set the tool orientation accordingly to prevent robot singularities.After setting one target's orientation you copy it, select all restant targets and apply this copied orientation.

![Tool-orientation-copy-and-replicate](/lab1/images/tool-position-copu.png)

### Creating a return-to-home routine

To safely return to a specified home position we create a JointTarget (`Home > Target > Create JointTarget`) and we set the `Robot axes` parameter all to $0°$. We then add it to a new path that we are gonna use for at least two times, at a first moment to relocate robot to desired home position for security purposes and as a last movement for safely switching between routines. We also create a `medium point path` to prevent RobotStudio stucking itself going from home to the first CAD's target. This target is created via the `Home > Target > Create Target` and we enter the desired position.

Last on this item, we set all move instructions' speed and error to `v500` and `zfine` except for the `home routine` which is set to v1000 and z50.


## Simulations and physical implementation
___

Developed by
[Juan David Díaz García](https://github.com/D4vidDG) and [Steven Gallego](https://github.com/jhairssteven).
