
%clear all; close all; clc
addpath('functions','SWparts','SWparts/wheelchair')
load('Kinematics.mat')


%% Inputs to Calculate XYZ coordines of SP Leg Member Ends

%top ellipse (upper abdomen)
a2 = 15/1.1;        %cm
b2 = 11.28/1.1;     %cm
z2 = 20;            %cm

%bottom ellipse (trunk bottom / waist)
a1 = 28/2/1.1;      %cm
b1 = 21/2/1.1;       %cm
z1 = 0;             %cm

%Misc
jntPitch = 4;       %cm
    


varAngle = 120;    %deg

            
%% Calculate Initial Coordinate Data
[TopLocalCoordinates, BotLocalCoordinates, TopGlobalCoordinates, L_initial] ...
    = SPCoordinateCalc( a1,b1,z1,a2,b2,z2,jntPitch,varAngle);
% TopLocalCoordinates = initial top [4x7] row=[x;y;z;1] z=0 for local
%   the 1st column is the local origin
% BotLocalCoordinates = initial top [3x7] row=[x;y;z] z=0, no Transform so
%   no 4th row. first column is reference origin
% TopGlobalCoordinates = [3x7] row=[x;y;z] initial top in global coord.
% L_initial = [1x7] initial leg length at starting position

%% Input Parameters

time = 10;
time_steps = 1000;

Prm.m1 = 0.5;
Prm.m2 = 4;
Prm.m3 = 2;
Prm.l1 = 2;
Prm.l2 = 4;
Prm.l3 = 5;
Prm.I1 = 1;
Prm.I2 = 1;
Prm.I3 = 1;
Prm.k = [5000 5000 5000];         %springK
Prm.B = 90000;          %damping Ns/m      
Prm.Spinek = 1;         %springK
Prm.SpineB = 0;          %damping Ns/m  
Prm.SpineOffset = 7;
Prm.g = 9.81;          %m/s^2
Prm.BC = [BotLocalCoordinates]; clear BotLocalCoordinates
Prm.TC = [TopLocalCoordinates]; clear TopLocalCoordinates
Prm.SpringLim = 0.5/100;
Prm.SpringStopForce = 300;%130;
Prm.ActuatorStopForce = 300;
Prm.ActuatorBotThreshold = -Prm.l2 /100;
Prm.ActuatorTopThreshold = Prm.l2/100 ;
Prm.ActuatorB = 10;

%%%%%%%% SimMechanics Specific Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% these parameters arise due to the way simmechanics prescribe frames. They
% are goverend by dimensions of the platfor components.



SmmPrm.BSurfaceOffset = 1;   %vertical offset from the base Ref. frame to
% the surface of the base in Smm
SmmPrm.BJointOffset = 0.2;   %vertical offset from the base surface in Smm to 
% the equivlanet Z=0 reference frame used in node generation. I.E, where the
% joint centers of the bottom ball joints have z = 0
SmmPrm.SocketOffset = 0.2;   %vertical offset from base surface to socket
% Ref. frame in Smm
SmmPrm.SocketToJointC = 0;

SmmPrm.TSurfaceOffset = 2.5;   %vertical offset from the top Ref. frame to
% the bottom surface of the top in Smm

L_InitAct = L_initial(2:7)-1.7*2-Prm.l1*2-Prm.l2-Prm.l3*2;

%% Inverse Kinematics

q_target = [0 0 20 0 0 0];  %desired [x y z alpha beta gamma] of end ef




TopTransform = [roty(q_target(5))*rotx(q_target(4))*rotz(q_target(6)), ...
    [q_target(1);q_target(2);q_target(3)]];
TrTopCoordinates = TopTransform * Prm.TC; %top coordinates at q_target
L_Tr = zeros(1,7);
for i = 1:7
L_Tr(i) = norm(TrTopCoordinates(:,i)-Prm.BC(:,i));
end

settle_time = 1;

q_input = [linspace(0,time,time_steps)', zeros(time_steps,6)];
temp = round((settle_time/time)*length(q_input));
q_input(1:temp,2:end) = repmat(L_initial(2:7),[temp 1]);
q_input(temp+1:end,2:end) = repmat(L_Tr(2:7),[time_steps-temp 1]);
clear temp;

%% Inverse Kinematics Figure

% figure;
% 
% subplot(2,2,1)
%     grid on; hold on
%     xlabel('x'),ylabel('y'),zlabel('z');
%     scatter3(Prm.BC(1,:),Prm.BC(2,:),Prm.BC(3,:),800,'.')
%     scatter3(TopGlobalCoordinates(1,:),TopGlobalCoordinates(2,:),TopGlobalCoordinates(3,:),800,'.')
%     
%     for i = 1+1:6+1
%     plot3([Prm.BC(1,i) TopGlobalCoordinates(1,i)],[Prm.BC(2,i) TopGlobalCoordinates(2,i)], [Prm.BC(3,i) TopGlobalCoordinates(3,i)],...
%     'LineWidth',2,'Color','m')
%     end
%     
%     
%     plot3(TopGlobalCoordinates(1,2:end), TopGlobalCoordinates(2,2:end), TopGlobalCoordinates(3,2:end),...
%     'LineWidth',2,'Color','k')
%     plot3([TopGlobalCoordinates(1,end),TopGlobalCoordinates(1,2)], ...
%         [TopGlobalCoordinates(2,end),TopGlobalCoordinates(2,2)], ...
%         [TopGlobalCoordinates(3,end),TopGlobalCoordinates(3,2)],...
%     'LineWidth',2,'Color','k')
% axis equal
% view(3)
% 
% subplot(2,2,2)
%     grid on; hold on
%     xlabel('x'),ylabel('y'),zlabel('z');
%     scatter3(Prm.BC(1,:),Prm.BC(2,:),Prm.BC(3,:),800,'.')
%     scatter3(TrTopCoordinates(1,:),TrTopCoordinates(2,:),TrTopCoordinates(3,:),800,'.')
%     
%     for i = 1+1:6+1
%     plot3([Prm.BC(1,i) TrTopCoordinates(1,i)],[Prm.BC(2,i) TrTopCoordinates(2,i)], [Prm.BC(3,i) TrTopCoordinates(3,i)],...
%     'LineWidth',2,'Color','m')
%     end
%     
%     
%     plot3(TrTopCoordinates(1,2:end), TrTopCoordinates(2,2:end), TrTopCoordinates(3,2:end),...
%     'LineWidth',2,'Color','k')
%     plot3([TrTopCoordinates(1,end),TrTopCoordinates(1,2)], ...
%         [TrTopCoordinates(2,end),TrTopCoordinates(2,2)], ...
%         [TrTopCoordinates(3,end),TrTopCoordinates(3,2)],...
%     'LineWidth',2,'Color','k')
% axis equal
% view(3)
% 
% 
% subplot(2,2,3)
% bar(1:6,L_initial(2:7))
% 
% subplot(2,2,4)
% bar(1:6,L_Tr(2:7))

%% Simmechanics


%%%%% Forward Dynamics Loading %%%%%%%%%%%%%%%

force_increments = 100;


%
if mod(time_steps,force_increments) ~= 0
    while mod(time_steps,force_increments) ~= 0
        force_increments = force_increments+1;
    end
disp(['force increment was non integer; adjusted to force_increments =',int2str(force_increments)])
end
%

Finc_ts_span = time_steps/force_increments;
time_perFincrement = time/force_increments;
forces = zeros(time_steps,1);
forcesteps = linspace(0,660,force_increments);
for i = 1:force_increments
forces(i*Finc_ts_span-(Finc_ts_span-1):i*Finc_ts_span) = forcesteps(i);
end
%AppliedForce = [linspace(0,time,time_steps)' forces];
AppliedForce = [linspace(0,time,time_steps)' zeros(time_steps,1)];
AppliedForce(1:length(AppliedForce)/2,2) =  forces(1:length(AppliedForce)/2);



%%%%%%|*|*|*|*|*|*|*|*|GO|*|*|*|*|*|*|*|*|%%%%%%
tic
simulinkout = sim('HumanPlatform_WithControllerB','SimMechanicsOpenEditorOnUpdate','on','MaxConsecutiveZCsMsg','none');
toc

%timeout=600;
% set_param('HumanPlatform_WithArmsController','SimulationCommand','start')
% tic;
% while(true)
%     if not(strcmpi(get_param('HumanPlatform_WithArmsController','SimulationStatus'),'running'))
%         disp('simulation exited')
%         break;
%     end
%     if toc>=timeout
%         disp('timout reached')
%         set_param('HumanPlatform_WithArmsController','SimulationCommand','stop')
%         break;
%     end
%     pause(1);
% end

%% Post

%BendEval
%TrajectoryError
