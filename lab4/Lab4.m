clc
%% DH convention
%Dimensions
L1 = 145;
L2 = 106.3;
L3 = 106.3;
L4 = 89.7;

%DH parameters
a = [0;L2;L3;L4]/100;
alpha = [90;0;0;0] * pi/180;
d = [L1;0;0;0]/100;
theta = [0;0;0;0];
sigma = zeros(4,1);
offset = [0;90;-90;0]* pi/180;

%Serial Link creation
DH_params = [theta d a alpha sigma offset];
Phantom = SerialLink(DH_params)
Phantom.name = "PhantomX";

%% Plotting

q1 = [0 0 0 0];
q2 = [-20 20 -20 20]*pi/180;
q3 = [30 -30 30 -30]*pi/180;
q4 = [-90 15 -55 17]*pi/180;
q5 = [-90 45 -55 45]*pi/180;

PlotRobot(Phantom,q5)

%% Forward Kinematics

syms Q1 Q2 Q3 Q4 Q5

% Calculate MTH
T_0T = Phantom.A([1 2 3 4],[Q1 Q2 Q3 Q4])*Phantom.tool;

% Simplify expression
T_0T = vpa(simplify(T_0T),10)

%% Verification

T_0T_q3_1 = fkine(Phantom,q3)
T_0T_q3_2 = eval(subs(T_0T,[Q1 Q2 Q3 Q4],q3))

function PlotRobot(Robot,q)
figure()
Robot.plot(q,'nobase','notiles')
zlim([0 5])
end


