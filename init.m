%Initiating constants for RCAM simulation
clear
clc
close all
    
%% Define constants
x0 = [85;   %approx 165 knots
    0;
    0;
    0;
    0;
    0;
    0;
    0;    %approx 5.73 deg
    0;
    0;%coordinates
    0;
    0];

u = [0;     % aileron
    0;      % elevator % %approx -5.73 deg
    0;      % rudder
    0.08;   % throttle 1%recall minimum for throttles are 0.5*pi/180 = 0.0087
    0.08];  % throttle 2
   
   TF = 60; %simugpslation time
   
   %% Trimming
   temp = load('trim_values_straight_level');
XStar = temp.XStar;
UStar = temp.UStar;
      %% Saturation
u1min = -25*pi/180; %Ailerons limits
u1max = 25*pi/180;

u2min = -25*pi/180; %Elevator limits
u2max = 10*pi/180;

u3min = -30*pi/180; % Rudder limits
u3max = 30*pi/180;

u4min = 0.5*pi/180; % Throttle 1 limits
u4max = 10*pi/180;

u5min = 0.5*pi/180; % Throttle 2 limits
u5max = 10*pi/180;

   %% Initiate Mavlink
dialect = mavlinkdialect("common.xml");
uavNode = mavlinkio(dialect,'SystemID',1,'ComponentID',1, ...
    'AutopilotType',"MAV_AUTOPILOT_PX4",'ComponentType',"MAV_TYPE_FIXED_WING");
uavPort = 14750;
connect(uavNode,"UDP",'LocalPort',uavPort);

qgcPort = 14550;
heartbeat = createmsg(dialect,"HEARTBEAT");
heartbeat.Payload.type(:) = enum2num(dialect,'MAV_TYPE',uavNode.LocalClient.ComponentType);
heartbeat.Payload.autopilot(:) = enum2num(dialect,'MAV_AUTOPILOT',uavNode.LocalClient.AutopilotType);
heartbeat.Payload.system_status(:) = enum2num(dialect,'MAV_STATE',"MAV_STATE_STANDBY");

heartbeatTimer = timer;
heartbeatTimer.ExecutionMode = 'fixedRate';
heartbeatTimer.TimerFcn = @(~,~)sendudpmsg(uavNode,heartbeat,'127.0.0.1',qgcPort);
start(heartbeatTimer);
   %% Run the model
   sim('UAVSimulation.slx')

   %% Terminate Mavlink
stop(heartbeatTimer);
delete(heartbeatTimer);
disconnect(uavNode);

   
   %% Plot the results
   t = ans.simX.Time;
   
   u1 = ans.simU.data(:,1);
   u2 = ans.simU.data(:,2);
   u3 = ans.simU.data(:,3);
   u4 = ans.simU.data(:,4);
   u5 = ans.simU.data(:,5);
   
   x1 = ans.simX.data(:,1);
   x2 = ans.simX.data(:,2);
   x3 = ans.simX.data(:,3);
   x4 = ans.simX.data(:,4);
   x5 = ans.simX.data(:,5);
   x6 = ans.simX.data(:,6);
   x7 = ans.simX.data(:,7);
   x8 = ans.simX.data(:,8);
   x9 = ans.simX.data(:,9);
%% plots
   figure
   
   subplot(5,1,1)
   plot(t,u1)
   legend('u_1')
   grid on
   title('Aileron deflection')
   
   subplot(5,1,2)
   plot(t,u2)
   legend('u_2')
   grid on
   title('Elevator deflection')
   
   subplot(5,1,3)
   plot(t,u3)
   legend('u_3')
   grid on
   title('Rudder deflection')
   
   subplot(5,1,4)
   plot(t,u4)
   legend('u_4')
   grid on
   title('Engine 1 throttle')
   
   subplot(5,1,5)
   plot(t,u5)
   legend('u_5')
   grid on
   title('Engine 2 throttle')
   
%Plot the states
figure

%u,v,w
subplot(3,3,1)
plot(t,x1)
legend('x_1')
grid on
title('Axial velocity (u)')

subplot(3,3,4)
plot(t,x2)
legend('x_2')
grid on
title('Lateral velocity (v)')

subplot(3,3,7)
plot(t,x3)
legend('x_3')
grid on
title('Normal velocity (w)')

%p,q,r
subplot(3,3,2)
plot(t,x4)
legend('x_4')
grid on
title('Roll angular velocity (p)')

subplot(3,3,5)
plot(t,x5)
legend('x_5')
grid on
title('Pitch angular velocity (q)')

subplot(3,3,8)
plot(t,x6)
legend('x_6')
grid on
title('Yaw angular velocity (r)')

%phi,theta,psi
subplot(3,3,3)
plot(t,x7)
legend('x_7')
grid on
title('Roll angle (phi)')

subplot(3,3,6)
plot(t,x8)
legend('x_8')
grid on
title('Pitch angle (theta)')

subplot(3,3,9)
plot(t,x9)
legend('x_9')
grid on
title('Yaw angle (psi)')