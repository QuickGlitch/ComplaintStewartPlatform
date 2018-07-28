

figure('Position',[scrsz(1)/1 scrsz(4)/2-540 scrsz(4)/2 scrsz(4)/2.5],'Name','Animation'); hold on; grid on;
axis([-10 10 -10 10 0 20])
%view(3)
view(90,0) %XZ
% 
% grid on
xlabel('x'),ylabel('y'),zlabel('z');
% 
% An_i = 1

AnLeg1 = animatedline();%[Prm.BP(1,2);TopPointsInTime(1,2,1)],[Prm.BP(2,2);TopPointsInTime(2,2,1)],[Prm.BP(3,2);TopPointsInTime(3,2,1)]);
AnLeg2 = animatedline();
AnLeg3 = animatedline();
AnLeg4 = animatedline();
AnLeg5 = animatedline();
AnLeg6 = animatedline();
AnTop  = animatedline();
redfill = fill3(0,0,0,'r');

% writerObj = VideoWriter('./samples/out.avi'); % Name it.
% writerObj.FrameRate = 60; % How many frames per second.
% open(writerObj); 

for reps = 1:3;
    for An_i = 1:length(TopPointsInTime)

       AnLeg1New = [[Prm.BP(1,1);TopPointsInTime(1,1,An_i)],[Prm.BP(2,1);TopPointsInTime(2,1,An_i)],[Prm.BP(3,1);TopPointsInTime(3,1,An_i)]];
       AnLeg2New = [[Prm.BP(1,2);TopPointsInTime(1,2,An_i)],[Prm.BP(2,2);TopPointsInTime(2,2,An_i)],[Prm.BP(3,2);TopPointsInTime(3,2,An_i)]];
       AnLeg3New = [[Prm.BP(1,3);TopPointsInTime(1,3,An_i)],[Prm.BP(2,3);TopPointsInTime(2,3,An_i)],[Prm.BP(3,3);TopPointsInTime(3,3,An_i)]];
       AnLeg4New = [[Prm.BP(1,4);TopPointsInTime(1,4,An_i)],[Prm.BP(2,4);TopPointsInTime(2,4,An_i)],[Prm.BP(3,4);TopPointsInTime(3,4,An_i)]];
       AnLeg5New = [[Prm.BP(1,5);TopPointsInTime(1,5,An_i)],[Prm.BP(2,5);TopPointsInTime(2,5,An_i)],[Prm.BP(3,5);TopPointsInTime(3,5,An_i)]];
       AnLeg6New = [[Prm.BP(1,6);TopPointsInTime(1,6,An_i)],[Prm.BP(2,6);TopPointsInTime(2,6,An_i)],[Prm.BP(3,6);TopPointsInTime(3,6,An_i)]];

       AnTopNew  = [TopPointsInTime(1,[1:6,1],An_i)', TopPointsInTime(2,[1:6,1],An_i)', TopPointsInTime(3,[1:6,1],An_i)'];

       clearpoints(AnTop);
       clearpoints(AnLeg1);
       clearpoints(AnLeg2);
       clearpoints(AnLeg3);
       clearpoints(AnLeg4);
       clearpoints(AnLeg5);
       clearpoints(AnLeg6);

       addpoints(AnTop,AnTopNew(:,1),AnTopNew(:,2),AnTopNew(:,3));
       addpoints(AnLeg1,AnLeg1New(:,1),AnLeg1New(:,2),AnLeg1New(:,3));
       addpoints(AnLeg2,AnLeg2New(:,1),AnLeg2New(:,2),AnLeg2New(:,3));
       addpoints(AnLeg3,AnLeg3New(:,1),AnLeg3New(:,2),AnLeg3New(:,3));
       addpoints(AnLeg4,AnLeg4New(:,1),AnLeg4New(:,2),AnLeg4New(:,3));
       addpoints(AnLeg5,AnLeg5New(:,1),AnLeg5New(:,2),AnLeg5New(:,3));
       addpoints(AnLeg6,AnLeg6New(:,1),AnLeg6New(:,2),AnLeg6New(:,3));

       delete(redfill)
       redfill = fill3(AnTopNew(:,1),AnTopNew(:,2),AnTopNew(:,3),'r');

    %view(0,0) 
    %view(3)
    drawnow limitrate

    % frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
    %         writeVideo(writerObj, frame);

    %pause(0.002)
    end
end

hold off
%close(writerObj);
