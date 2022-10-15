clc
L1 = 145;
L2 = 106.3;
L3 = 106.3;
L4 = 89.7;

a = [0;L2;L3;L4]/100;
alpha = [90;0;0;0] * pi/180;
d = [L1;0;0;0]/100;
theta = [0;0;0;0];
sigma = zeros(4,1);
offset = [0;90;-90;0]* pi/180;

DH_params = [theta d a alpha sigma offset];
Phantom = SerialLink(DH_params)
Phantom.name = "PhantomX";

q1 = [0 0 0 0];
q2 = [-20 20 -20 20]*pi/180;
q3 = [30 -30 30 -30]*pi/180;
q4 = [-90 15 -55 17]*pi/180;
q5 = [-90 45 -55 45]*pi/180;

PlotRobot(Phantom,q5)

function PlotRobot(Robot,q)
figure()
Robot.plot(q,'nobase','notiles')
zlim([0 5])
end


