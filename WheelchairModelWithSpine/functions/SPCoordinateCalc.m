function [ TopLocalCoordinates, BotLocalCoordinates,TopGlobalCoordinates, L_initial ] = SPCoordinateCalc( a1,b1,z1,a2,b2,z2,jntPitch,varAngle )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here


% Actuator End Points

jntBotPoints = zeros(2,3,3); %declare variable storing jnt Points in XYZ,
%[x1, y1, z1; x2...] where 1 and 2 are clockwise joints at each joint area
jntTopPoints = zeros(2,3,3);

% jntBotPoints(:,:,1) 
x = [-jntPitch/2,jntPitch/2];
y = sqrt(b1^2*(1- x.^2/a1^2));
jntBotPoints(:,:,1) = [x(1) y(1) 0; x(2) y(2) 0];

% jntTopPoints(:,:,1) 
x = [-jntPitch/2,jntPitch/2];
y = sqrt(b2^2*(1- x.^2/a2^2));
jntTopPoints(:,:,1) = [x(1) -y(1) z2; x(2) -y(2) z2];


jntBotPoints(:,:,2) = jntCalc (90-varAngle,a1,b1,jntPitch,z1,varAngle);
jntBotPoints(:,:,3) = jntBotPoints(:,:,2) .* [-1 1 1; -1 1 1];

jntTopPoints(:,:,2) = jntCalc (-90+varAngle,a2,b2,jntPitch,z2,varAngle);
jntTopPoints(:,:,3) = jntTopPoints(:,:,2) .* [-1 1 1; -1 1 1];


clear x y



%% Actuator Comp

ActuatorVect = zeros(2,3,6);

ActuatorVect(:,:,1) = ...
 [jntBotPoints(1,1,3) jntBotPoints(1,2,3) jntBotPoints(1,3,3)
 jntTopPoints(1,1,1) jntTopPoints(1,2,1) jntTopPoints(1,3,1)];

ActuatorVect(:,:,2) = ...
 [jntBotPoints(2,1,3) jntBotPoints(2,2,3) jntBotPoints(2,3,3)
 jntTopPoints(1,1,3) jntTopPoints(1,2,3) jntTopPoints(1,3,3)];

ActuatorVect(:,:,3) = ...
 [jntBotPoints(1,1,1) jntBotPoints(1,2,1) jntBotPoints(1,3,1)
 jntTopPoints(2,1,3) jntTopPoints(2,2,3) jntTopPoints(2,3,3)];

ActuatorVect(:,:,4) = ...
 [jntBotPoints(2,1,1) jntBotPoints(2,2,1) jntBotPoints(2,3,1)
 jntTopPoints(2,1,2) jntTopPoints(2,2,2) jntTopPoints(2,3,2)];

ActuatorVect(:,:,5) = ...
 [jntBotPoints(2,1,2) jntBotPoints(2,2,2) jntBotPoints(2,3,2)
 jntTopPoints(1,1,2) jntTopPoints(1,2,2) jntTopPoints(1,3,2)];

ActuatorVect(:,:,6) = ...
 [jntBotPoints(1,1,2) jntBotPoints(1,2,2) jntBotPoints(1,3,2)
 jntTopPoints(2,1,1) jntTopPoints(2,2,1) jntTopPoints(2,3,1)];


TopLocalCoordinates = zeros(4,7);
BotLocalCoordinates = zeros(3,7);
TopLocalCoordinates(:,1) = [0 0 0 1]';
BotLocalCoordinates(:,1) = [0 0 0]';
L_initial = zeros(1,7);
L_initial(1) = z2-z1;
for i = 2:7
TopLocalCoordinates(:,i) = [ActuatorVect(2,1,i-1) ActuatorVect(2,2,i-1) 0 1]';
BotLocalCoordinates(:,i) = [ActuatorVect(1,:,i-1)]';
L_initial(i) = norm([ActuatorVect(2,:,i-1)]-[ActuatorVect(1,:,i-1)]);
end
%L_initial(2:7) = circshift(L_initial(2:7),-3,2);
TopGlobalCoordinates = TopLocalCoordinates(1:3,:);
TopGlobalCoordinates(3,:) = z2;
