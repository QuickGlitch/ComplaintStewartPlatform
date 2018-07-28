
scrsz = get(groot,'ScreenSize');

figure('Position',[scrsz(1)/1+10 scrsz(4)/2 scrsz(4)/1.2 scrsz(4)/2.5],'name','Error');
temp = round((settle_time/time)*length(q_input));
targetx = [zeros(1,temp) q_target(1)*ones(1,time_steps-temp)];
targety = [zeros(1,temp) q_target(2)*ones(1,time_steps-temp)];
targetz = [(z2-z1)*ones(1,temp) q_target(3)*ones(1,time_steps-temp)];

subplot(1,3,1); hold on; grid on
plot(linspace(0,time,time_steps),targetx);
plot(simout_TopTranslation.Time,-simout_TopTranslation.Data(:,2))
xlabel('time [s]'); ylabel('x coordinate [cm]')
legend('desired','achieved')
xlim([0 time])

subplot(1,3,2); hold on; grid on
plot(linspace(0,time,time_steps),targety);
plot(simout_TopTranslation.Time,-simout_TopTranslation.Data(:,1))
xlabel('time [s]'); ylabel('y coordinate [cm]')
%legend('desired','achieved')
xlim([0 time])

subplot(1,3,3); hold on; grid on
plot(linspace(0,time,time_steps),targetz);
plot(simout_TopTranslation.Time,simout_TopTranslation.Data(:,3))
xlabel('time [s]'); ylabel('z coordinate [cm]')
%legend('desired','achieved')
xlim([0 time])