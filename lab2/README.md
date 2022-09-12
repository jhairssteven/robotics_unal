# Basic ROS concepts
### **What's a node?**
A node is a representation of an entity or a "program" in ROS, which serves as a block in a bigger system for information processing or interchange.

### **What's a ROS package?**
It is simply a set of nodes.

### **What's a topic?**
A topic is a piece of information used and provided by nodes in other to make interaction possible.
### **What are messages, publisers and subscribers?**
These are terms that relate how information is produced and who produces it. For instance, to "publish" is the action a node takes to make available some information. On the other hadn, you can "subscribe" to information or nodes.
# System requirements
We are using an Ubuntu 20.04 installation, with ROS noetic version 1.15.14 and MATLAB R2022a.

## Required Matlab toolboxes
- Robotics ToolBox
- ROS ToolBox

We run the first script succesfully by following the instructions given on the laboratory [guide](https://drive.google.com/file/d/19UOE_eI-ob2ZymNHWFrYgrxLQfgOon43/view). Results are not shown because they are trivial. Although script's code is explained to clarify conceps.

```matlab
%%
rosinit; %Conexión con nodo maestro
%%
velPub = rospublisher('/turtle1/cmd_vel','geometry_msgs/Twist'); %Creación publicador
velMsg = rosmessage(velPub); %Creación de mensaje
%%
velMsg.Linear.X = 1; %Valor del mensaje
send(velPub,velMsg); %Envio
pause(1)  
```

## Code explanation
We first initialize the master node with the `rosinit` command. We create a ROS publisher which uses the `cmd_vel` topic and we use the Twist message *topic* which helps us control some geometric features like dots or vectors. 

We initialize a *message* instance with the already configured *publisher* instance and we set it with a linear movement in the X direction with a value of $1$.

Finally, the the `send` function sends these configurations to the ROS master node running on localhost which causes the turtle drawing to move as specified.


# Python implementation
## Required packages
- pynput
- numpy

We will build a custom ROS node to control the `turtlesim` node using our computer's keyboard. Mapping is as follows:
- `w`: Move forwards
- `s`: Move backwards
- `a`: Rotate left
- `d`: Rotate right
- `space`: Rotate turtle
- `r`: Reset position

> Note: No uppercase support yet.

## Getting started - Environment setup
If clonning this repo, begin by building the catking workspace:

```bash
    $ cd catkin_ws/
    $ catkin_make
```
If starting fresh, run:

```bash
    $ source /opt/ros/noetic/setup.bash #source env
    $ mkdir -p /catkin_ws/src
    $ cd /catkin_ws/
    $ catkin_make
    $ source devel/setup.bash # source workspace
```

To make sure your workspace is properly overlayed by the setup script, make sure ROS_PACKAGE_PATH environment variable includes the directory you're in.

```bash
    # output should look something like: 
    # /home/youruser/catkin_ws/src:/opt/ros/kinetic/share
    $ echo $ROS_PACKAGE_PATH
```

To test the results launch three terminals and run:
```bash
    # 1st terminal
    $ roscore # Enable ROS master node

    # 2nd terminal
    $ rosrun turtlesim turtlesim_node # Enable turtle node (GUI)

    # 3rd terminal
    $ source /dev/setup.bash # source catkin env
    $ rosrun hello_turtle myTeleopKey.py # Run keyboard-control implementation

```
## Code explanation 
___

Our whole program is implemented from a single function as follows:

```python 
    def key_turtle_control():
        kb = KeyListener() # Start listening for key presses
        turtleMov(5, 5, 0, 1) # Mov to screen's center
        velPub = VelPub()
        while not rospy.is_shutdown():
            if KeyCode.from_char('w') == kb.lastKey:
                velPub.pubVel(1, 0)
            if KeyCode.from_char('s') == kb.lastKey:
                velPub.pubVel(-1, 0)
            if KeyCode.from_char('a') == kb.lastKey:
                velPub.pubVel(0, pi/2)
            if KeyCode.from_char('d') == kb.lastKey:
                velPub.pubVel(0, -pi/2)
            if KeyCode.from_char('r') == kb.lastKey:
                turtleMov(5, 5, 0, 1)
                rospy.wait_for_service('/clear')  # Clear the trajectory
                clearTrajectory = rospy.ServiceProxy('/clear', Empty)
                clearTrajectory()
            if Key.space == kb.lastKey:
                turtleMov(0, 0, pi, 0)
            if Key.esc == kb.lastKey:
                break
```
In order to make it work we implemented three aditional custom functions and classes as follows:

- `KeyListener(x, y, a, m)`: It listens to typed keys
- `turtleMove()`: Moves the Turtle ("teleports") to the specified position and orientation, either by absolute or relative coordinates.
- `pubVe(vx, az)`: Moves the Turtle (no teleport) with an X directional velocity component and at a certain angle, in a continous way, leaving a trajectory.

> Note: All implemented functions were unit-tested using the following code as constructor. 
 
```python
    def parse_args():
        parser = argparse.ArgumentParser()
        parser.add_argument('-x', type = float, default = 5.0)
        parser.add_argument('-y', type = float, default = 5.0)
        parser.add_argument('-ang', type = float, default = 0)
        parser.add_argument('-m', type = float, default = 1)
        args = parser.parse_args()
        return args

    if __name__ == "__main__":
        args = parse_args()
        turtleMov(args.x, args.y, args.ang, args.m)
```

The `turtleMove()` function receives 4 requiered parameters which specify the final position and orientation (`x`, `y`, and `a`) and the coordinates mode (`m` $1$ for absolute and $0$ for relative movement). It calls two diferent ROS services one for each movement mode.


```python
    service = '/turtle1/teleport_' + ('absolute' if m else 'relative')
    mode = TeleportAbsolute if m else TeleportRelative
    rospy.wait_for_service(service)
    try:
        tp = rospy.ServiceProxy(service, mode)
    ...
```

The `VelPub` class implements an [`__init__`](/lab2/catkin_w/src/hello_turtle/scripts/turtleVel.py#L7-L11) method to initialize the ROS publisher `/turtle1/cmd_vel` only once, there we also define a `Twist` message type and a rate of execution of $0.1s$ given as a frecuency. We define a `pubVel()` function which actualize moves the Turtle. Here we set the linear X velocity and the rotation angle, publish the message and execute it by $0.1s$ as described in the configuration function before.

```python
    def pubVel(self, vx, az):
        '''Move in X direction at `vx` and rotate in Z `az` radians'''
        self.vel.linear.x = vx
        self.vel.angular.z = az
        rospy.loginfo(self.vel)
        self.pub.publish(self.vel)
        self.rate.sleep()
```


___
Developed by
[Juan David Díaz García](https://github.com/D4vidDG) and [Steven Gallego](https://github.com/jhairssteven).
