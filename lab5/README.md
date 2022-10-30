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