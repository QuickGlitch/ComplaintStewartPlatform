function TP_output = TP_tempGen(r,p,ang1,ang2,ang3)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

syms ang real
temp = [cos(ang) sin(ang) 0 r*sin(ang); -sin(ang) cos(ang) 0 r*cos(ang); 0 0 1 0]*[p(1) p(2); 0 0; 0 0; 1 1];

TP_output = [eval(subs(temp,ang,deg2rad(ang1))), eval(subs(temp,ang,deg2rad(ang2))), eval(subs(temp,ang,deg2rad(ang3)))];


end

