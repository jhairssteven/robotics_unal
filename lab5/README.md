# Phantom X inverse kinematics using ROS

## Trajectory generation
___

Trajectories were generated using a base [CAD model](/lab5/CAD%20models/). Then, the autopath feature of RobotStudio was used in order to get the points' coordinates out from the generated targets in the RAPPID file.

![Point coordinates from targets](/lab5/Images/targets_RAPPID.png)

On that file, the first list of three numbers represent the point coordinates of each target. A text editor was used to extract the relevant information and the final format can be seen in the generated [trajectory text files](/lab5/trajectories/). MS Excel was used to check the results in a graphical way, an example is shown  for the [custom shape](/lab5/trajectories/custom_part.txt) trajectory.

![Plot for Custom part on Excel](/lab5/Images/custom_part_Excel_plot.png)

Below you can see the base CAD model with the generated targets in RobotStudio. The ring on which figures where build, represent the Phantom X workspace.

![CAD model with targets on it](/lab5/Images//targets_on_CAD_robotstudio.png)

### Reading trajectories with Python
___

A Python script was used to parse the trajectories' text files and import all targets into a Python list to be used by other modules.
Code snipped is shown below.

```python
# Text file having the format
# Target:= [x,y,z],
def getTrajectoryFromTextFile(filename):
    with open(filename, 'r') as fobj:
        trajectory = []
        for line in fobj:
            coords_str = line.split()[-1][1:-2].split(",")
            coords = [float(n) for n in coords_str]
            trajectory.append(coords)

    return trajectory
```

### Point Format

Each point is defined as a vector with _x_, _y_ and _z_ coordinates. However, our robot will be constantly changing between 2 constant heights: a height for drawing and a height for moving around without drawing. Therefore, we defined the _z_ coordinate as follows:

- _z=1_ means height for moving around. 
- _z=0_ means height for drawing.
- _z=-1_ means height for drawing. In this case, the robot transitions from moving to drawing. This is used to smooth the movement when the robot is descending.

## Inverse kinematics

We need to determine which joint values result in the position we need.

[Phantom_dimensions] (Images/Phantom_dimensions.png)

The first angle is simple to calculate.

[theta1_ikine] (Images/theta1_ikine.png)

### Kinematic Decoupling

We will use kinematic decoupling to calculate the position of the robot's wrist, which is located in the fourth joint. This can be expressed as:

```python
wristPos = desiredPos -  L4*approach
```
Here, `approach` refers to the _X_ direction of tool's frame.

### Inverse kinematics for double pendulum

Note that the links from the second joint to the fourth joint form a double pendulum, which inverse kinematics are well known.

[double_pendulum] (Images/double_pendulum.jpg)

However, we need to obtain the wrist position measured from the second joint frame. Then, we calculate the transformation matrix from the first frame to the second.

$$wrist^1 = H_{0}^1 wrist^0$$

### Fourth joint

Given a tool orientation $\phi$ in relation to second joint's frame, the fourth joint value can be found as:

$$\theta_4 = \phi - \theta_3 - \theta_2$$

$\phi$ can be obtained from the approach vector measured from the second joint frame.


$$
approach^1 = H_{0}^1 approach^0 
\phi = atan(\frac{y}{x})
$$
