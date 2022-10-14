clc
L1 = 145;
L2 = 106.3;
L3 = 106.3;
L4 = 89.7;

a = [0;L2;L3;L4]/100;
alpha = [-90;0;0;0] * pi/180;
d = [L1;0;0;0]/100;
theta = [0;0;0;0];
sigma = zeros(4,1);
offset = [0;-90;90;0]* pi/180;

DH_params = [theta d a alpha sigma offset];
Phantom = SerialLink(DH_params)
Phantom.name = "PhantomX";

q1 = [0 0 0 0];
q2 = [-20 20 -20 20]*pi/180;
q3 = [30 -30 30 -30]*pi/180;
q4 = [-90 15 -55 17]*pi/180;
q5 = [-90 45 -55 45]*pi/180;



%% Conexión a nodo maestro
rosinit
%% Topic subscription
jointSub = rossubscriber('/dynamixel_workbench/joint_states','DataFormat','struct'); % We create the subscriptor
[msgSub1,status,statustext] = receive(jointSub,10); % We receive the message
disp("Angle in radians for each joint:")
disp(" ")

for i = 1:5
    disp("Joint" + i + ": " + msgSub1.Position(i))
end


%%
    [motorSvcClient, motorSvcRequest] = rossvcclient('/dynamixel_workbench/dynamixel_command'); %Creation of client for the service
    motorCommandMsg = rosmessage(motorSvcClient); %Creation of the service message
    motorCommandMsg.AddrName = "Goal_Position";    
        motorCommandMsg.Id = 1;
        disp(round(mapfun(rad2deg(q(i)),-150,150,0,1023)))
        motorCommandMsg.Value = round(mapfun(rad2deg(q(i)),-150,150,0,1023));
      call(motorSvcClient,motorCommandMsg); 


function PlotRobot(Robot,q)
figure()
Robot.plot(q,'nobase','notiles')
zlim([0 5])
end

%Function to move joints and gripper
function output = moveRobot(q)
    offsetID = 0;
    motorSvcClient = rossvcclient('/dynamixel_workbench/dynamixel_command'); %Creation of client for the service
    motorCommandMsg = rosmessage(motorSvcClient); %Creation of the service message
    

    %Torque adjustment
    motorCommandMsg.AddrName = "Torque_Limit";
    torque = [600, 400, 400, 400, 400];
    for i= 1: length(q) 
        motorCommandMsg.Id = i+offsetID;
        motorCommandMsg.Value = torque(i);
        call(motorSvcClient,motorCommandMsg);
    end

    %Movement of joints    
    motorCommandMsg.AddrName = "Goal_Position";
    for i= 1: length(q) %To move joints
        disp(i)
        motorCommandMsg.Id = i+offsetID;
        disp(round(mapfun(rad2deg(q(i)),-150,150,0,1023)))
        motorCommandMsg.Value = round(mapfun(rad2deg(q(i)),-150,150,0,1023));
        call(motorSvcClient,motorCommandMsg); 
        pause(1);
    end
    
end