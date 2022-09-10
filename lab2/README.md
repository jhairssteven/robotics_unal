# Basic ROS concepts
### **What's a node?**
A node is a representation of an entity or like a "program" in ROS.
### **What's a topic?**
A topic is a piece of information which is used by topics to interact with other ones.
### **What are messages, publisers and subscribers?**
These are terms that relate how information is produced and who produces it. For instance, to "publish" is the action a node takes to make available some information. On the other hadn, you can "subscribe" to information or nodes.
# Methodology
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



___
Developed by
[Juan David Díaz García](https://github.com/D4vidDG) and [Steven Gallego](https://github.com/jhairssteven).