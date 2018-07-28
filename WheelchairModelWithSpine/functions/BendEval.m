%% Criteria Gen 

%SimOut_TopAngle= simOut.get('SimOut_TopAngle');

SimOut_Moment = [0 SimOut_TopAngle.Data(2:end,5)'.*SimOut_TopAngle.Data(2:end,6)']; 
 
flexROM = 40;
TestROM = [0 SimOut_TopAngle.Data(2:end,1)'];
   
rom1 = TestROM./flexROM *100;

flexionBraceLvl = [2.565 .0252; 2.831 .0268; 5.531 .0190];

brace = 1;
Mflex = BrownF(flexionBraceLvl(brace,1),flexionBraceLvl(brace,2),rom1);
  
Score = sum( (SimOut_Moment-Mflex).^2 )
    
%FigMF = figure();
                 %hold on; grid on
                 %ylabel('Bending Moment [Nm]'); xlabel('Platform tilt [deg]')
                 cla
                 plot(rom1,Mflex,'-mo','LineWidth',2, 'MarkerFaceColor','m')
                 plot(rom1,SimOut_Moment,'-bo','LineWidth',2,'MarkerFaceColor','b')
                 %scatter(rom1,SimOut_Moment,'b','LineWidth',2)
                 legend('Brown Function','SimOut')
