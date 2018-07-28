syms alpha beta gamma

% Rot_x = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
% Rot_y = [cos(beta) 0 -sin(beta); 0 1 0; sin(beta) 0 cos(beta)];
% Rot_z = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];

Rot_x = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
Rot_y = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
Rot_z = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];

R = Rot_y*Rot_x*Rot_z;