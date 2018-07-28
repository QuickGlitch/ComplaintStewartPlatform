function [temp5] = jntCalc( ang,a,b,jntPitch,z,RootAngle)
%Calculate jntPositions given the area (center between them) coordinate

r = a*b / sqrt((b*cosd(ang))^2+(a*sind(ang))^2);
angpoint = [r * cosd(ang); r * sind(ang)];
temp = angpoint(1);

if angpoint(2)<0
    slope = (b^2*temp)./(a^2*sqrt(b^2*(1-temp.^2/a^2)));
else
    slope = -(b^2*temp)./(a^2*sqrt(b^2*(1-temp.^2/a^2)));
end


const = angpoint(2)- slope*angpoint(1);

% x = linspace(angpoint(1)-1,angpoint(1)+1,10);
% y = slope*x+const;
% plot(x,y,'b');

temp = sqrt(1^2 + slope^2);
temp2 = 1/temp;
temp3 = [angpoint(1)+temp2*jntPitch/2 angpoint(1)-temp2*jntPitch/2];
temp4 = slope*temp3+const;
clear temp

temp = [temp3(1);temp4(1)];

syms delta;
eqn = temp(2)-sind(ang)*delta == sqrt(b^2*(1-(temp(1)-cosd(ang)*delta).^2/a^2));
sol = solve(eqn,delta);
temp6 = double(sol);

temp5 = [temp(1)-cosd(ang)*temp6 temp(2)-sind(ang)*temp6;
         temp3(2)-cosd(ang)*temp6 temp4(2)-sind(ang)*temp6];
     
   if  length(temp5) > 2
     temp5 = [[temp5(3,:);temp5(1,:)], [z;z]];
     
   elseif isempty(temp5)
     
    ang = 90-RootAngle;
     
         %%
    r = a*b / sqrt((b*cosd(ang))^2+(a*sind(ang))^2);
    angpoint = [r * cosd(ang); r * sind(ang)];
    temp = angpoint(1);

    if angpoint(2)<0
        slope = (b^2*temp)./(a^2*sqrt(b^2*(1-temp.^2/a^2)));
    else
        slope = -(b^2*temp)./(a^2*sqrt(b^2*(1-temp.^2/a^2)));
    end


    const = angpoint(2)- slope*angpoint(1);

    % x = linspace(angpoint(1)-1,angpoint(1)+1,10);
    % y = slope*x+const;
    % plot(x,y,'b');

    temp = sqrt(1^2 + slope^2);
    temp2 = 1/temp;
    temp3 = [angpoint(1)+temp2*jntPitch/2 angpoint(1)-temp2*jntPitch/2];
    temp4 = slope*temp3+const;
    clear temp

    temp = [temp3(1);temp4(1)];

    syms delta;
    eqn = temp(2)-sind(ang)*delta == sqrt(b^2*(1-(temp(1)-cosd(ang)*delta).^2/a^2));
    sol = solve(eqn,delta);
    temp6 = double(sol);

    temp5 = [temp(1)-cosd(ang)*temp6 temp(2)-sind(ang)*temp6;
             temp3(2)-cosd(ang)*temp6 temp4(2)-sind(ang)*temp6];
    temp5 = [[temp5(3,:);temp5(1,:)], [z;z]];
          
   elseif angpoint(2)<0
     temp5 = temp5.*-1;
     temp5 = [temp5, [z;z]];
   
   else
     temp5 = [temp5, [z;z]];  
   end

end

