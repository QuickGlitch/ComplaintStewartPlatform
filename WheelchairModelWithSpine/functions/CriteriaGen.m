%% Criteria Gen 

SimOut_Moment = [0 SimOut_TopAngle.Data(2:end,5)'.*SimOut_TopAngle.Data(2:end,6)']; 
    %[SimOut_TopAngle.Data(2:(end+1)/2,2)'.*SimOut_TopAngle.Data(2:(end+1)/2,5)'.*SimOut_TopAngle.Data(2:(end+1)/2,6)' ...
    %0 SimOut_TopAngle.Data((end+1)/2+1:end,2)'.*SimOut_TopAngle.Data((end+1)/2+1:end,5)'.*SimOut_TopAngle.Data((end+1)/2+1:end,6)'];

flexROM = 40;
%extROM = -25;
 
TestROM = [0 SimOut_TopAngle.Data(2:end,1)'];
    %[SimOut_TopAngle.Data(2:(end+1)/2,1)'.*SimOut_TopAngle.Data(2:(end+1)/2,2)' ...
    %0 SimOut_TopAngle.Data((end+1)/2+1:end,1)'.*SimOut_TopAngle.Data((end+1)/2+1:end,2)'];
bestScore = 1000000000;


rom1 = TestROM./flexROM *100;
%rom2 = TestROM(1:end/2)/extROM *100;


flexionBraceLvl = [2.565 .0252; 2.831 .0268; 5.531 .0190];
%extensionBraceLvl = [12.030 .00232; 12.520 .00235; 15.350 .00206];
%lateralBraceLvl = [4.474 .0161; 7.353 .0138; 6.913 .0148];

    
    %for brace = [1:3]
    brace = 1;
    
        Mflex = BrownF(flexionBraceLvl(brace,1),flexionBraceLvl(brace,2),rom1);
        %Mext = BrownF(extensionBraceLvl(brace,1),extensionBraceLvl(brace,2),rom2);

FigMF = figure();
                 hold on; grid on
                 ylabel('Bending Moment [Nm]'); xlabel('Platform tilt [deg]')        
                 %plot([-rom2 rom1],[-SimOut_Moment(1:end/2) SimOut_Moment(end/2:end)],'b','LineWidth',2)
                 plot(rom1,Mflex,'m','LineWidth',2)
                 plot(rom1,SimOut_Moment,'b','LineWidth',2)
                 scatter(rom1,SimOut_Moment,'b','LineWidth',2)
                 %plot(-rom2,-Mext,'m','LineWidth',2)      

    
