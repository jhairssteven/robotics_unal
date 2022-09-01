# Industrial robotics | Part 1

We are using the ABB IRB140 Robot with the RobotStudio software. We develop the necessary robot movements or trajectories to draw (with the aid of a whiteboard marker) our names' initials (i.e: **D** and **S**) on a planar surface and over a 30° inclined plane.

## Table of contents
___
- Tool CAD design
- Tool set up in RobotStudio
- Setting up the environment
- Getting trajectories with the `auto-path` wizard
- Setting up robot tool-orientation to prevent singularities
- Simulations and physical implementation

## Tool CAD design
___
In the figure below you will find the final tool. It has 5 main components (apart from the whiteboard marker) and was designed such that the base (black-colored component) could be reused in other projects.

![Interior and exterior tool view](/lab1/images/in-out-tool-view.png)

### Material and Physical Properties

 - The tool was 3D printed and its physical properties were calculated according to the printing material and marker's physical characteristics.

 ![Tool physical characteristics](/lab1/images/tool-physical-characteristics.png)

### Considerations
 - The black-colored component (base) was designed taking into account the [Official Robot Tool Flange dimensions](www.google.com).
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



___
Developed by
[Juan David Díaz García](https://github.com/D4vidDG) and [Steven Gallego](https://github.com/jhairssteven).
