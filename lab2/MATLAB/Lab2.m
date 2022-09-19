rosinit; %Conexión con nodo maestro
%%
velPub = rospublisher("/turtle1/cmd_vel","geometry_msgs/Twist"); %Creación publicador
velMsg = rosmessage(velPub); %Creación de mensaje
%%
velMsg.Linear.X = 1; %Valor del mensaje
send(velPub,velMsg); %Envio
pause(1)
%% Suscripción al topic pose y obtención de posición actual
SUB = rossubscriber("/turtle1/pose");
message = SUB.LatestMessage
%% Uso del servicio de teleport_absolute
SVCINF = rosservice("info","/turtle1/teleport_absolute")
[CLIENT,REQUEST] = rossvcclient("/turtle1/teleport_absolute","DataFormat","struct")
REQUEST.X = single(2);
REQUEST.Y = single(3);
theta = deg2rad(45);
REQUEST.Theta = single(theta);
waitForServer(CLIENT,"Timeout",3)
response = call(CLIENT,REQUEST)
%% Desconexión del nodo maestro
rosshutdown
