clear all
close all

addpath './functions'

define_Rotations
des_case = '';
define_simmechanics  = 1;            % <------ If == 0 simechanics ewquivalent is not run, results (and run times) not compared


%% For Barys :)
define_actuator_lock = 1;            % <------ If == 0 actuator forces (constant here) are applied, else actuator is locked with a calculated force per step

%% Initiate defines
switch define_actuator_lock
    case 1
        disp('simulating with actuators locked')
        if define_simmechanics == 1
                disp('SimMechanics comparison will be simulated')
        end
            
    case 0
        disp('simulating with actuation in legs')
        if define_simmechanics == 1
               disp('SimMechanics comparison will be simulated')
        end
end

%% start

start_t   = 0;
switch define_actuator_lock
    case 0
        sim_time = 2.7;
    case 1
        sim_time = 60;
end
timesteps = 700;

q0 = [0 0 16 0 0 0 ...          % x y z alpha beta gamma
      0 0 0 0 0 0  ...          % d1_1 d1_2 d1_3 d1_4 d1_5 d1_6
      0 0 0 0 0 0  ...          % x_dot y_dot z_dot ... gamma_dot
      0 0 0 0 0 0  ...          % d1_dot_1 d2_dot_2 ... d2_dot_6
      0 0 0 0 0 0];             % F1 F2 ... F6  

TP = [TP_tempGen(2,[-1 1],0,120,-120); ones(1,6)];
BP =  circshift(TP_tempGen(7,[-1 1],60,180,-60),[0,1]);

%% Parameters
Prm.TP = TP;
Prm.BP = BP;
Prm.l1 = 5;
Prm.l2 = 2;
Prm.l3 = 3;
Prm.l4 = Prm.l3;
Prm.lc = Prm.l1*2 + Prm.l2 + Prm.l3 + Prm.l4;

Prm.mp  = 27;
Prm.I_x = 10;
Prm.I_y = 10;
Prm.I_z = 20;
Prm.m3  = 5;
Prm.m2  = 8;
Prm.m1  = 0;

Prm.k   = [800 800 800 800 800 800];
Prm.B   = 50*ones(1,6);

Prm.g   = 9.81;
Prm.Fratio = 0;

%% Initial Kinematics
TP_Current = [ Generate_R(q0(4),q0(5),q0(6)), [q0(1);q0(2);q0(3)] ] * TP;
L_vector = TP_Current - BP;
[L_theta, L_phi, L_length] = cart2sph(L_vector(1,:),L_vector(2,:),L_vector(3,:));
q0(7:12) = L_length - Prm.lc;

TP_dot = Generate_TP_dot(TP(1,1),TP(1,2),TP(1,3),TP(1,4),TP(1,5),TP(1,6),TP(2,1),TP(2,2),TP(2,3),TP(2,4),TP(2,5),TP(2,6),TP(3,1),TP(3,2),TP(3,3),TP(3,4),TP(3,5),TP(3,6),q0(4),q0(16),q0(5),q0(17),q0(6),q0(18),q0(13),q0(14),q0(15));
L_dot  = dot(TP_dot,L_vector./repmat(sqrt(sum(L_vector.*L_vector)),[3 1]));

d2_0 = (L_length'-Prm.lc-q0(7:12)');
d2dot_0 = L_dot'-q0(19:24)';


%% Controls and Loads

F_load = [0 22 8];

switch define_actuator_lock
    case 0
        F = 100*ones(1,6); %[1 0 0 0 0 0]; %   
       
end

%% Simulation

% ODE45
options = odeset('RelTol',1.0e-8,'AbsTol',1.0e-8,'MaxStep',.2);
tic

switch define_actuator_lock
    case 0
    [t,y]= ode45(@(t,y)platform3DFunction(t,y,Prm,F,F_load,des_case), linspace(start_t,sim_time,timesteps), q0, options);

    case 1
    [t,y]= ode45(@(t,y)platform3DFunction_AL(t,y,Prm,F_load,des_case), linspace(start_t,sim_time,timesteps), q0, options);    
end

manual_time = toc;
disp(['manual simulation time: ',num2str(manual_time)])


%%% Post
AccelerationSolutions = zeros(timesteps,length(q0));
TopPointsInTime       = zeros(3,6,timesteps);
F_joint_InT           = zeros(3,6,timesteps);
L_vector_InT          = zeros(3,6,timesteps);

switch define_actuator_lock
    case 0
        for i = 1:length(TopPointsInTime);
        R_top_base = [ Generate_R(y(i,4),y(i,5),y(i,6)), [y(i,1);y(i,2);y(i,3)] ];
        TopPointsInTime(:,:,i) = R_top_base * TP;
        [AccelerationSolutions(i,:), F_joint_InT(:,:,i), L_vector_InT(:,:,i)] = ...
            platform3DFunction(t(i),y(i,:)',Prm,F,F_load,des_case);
        end
        
    case 1
        for i = 1:length(TopPointsInTime);
        R_top_base = [ Generate_R(y(i,4),y(i,5),y(i,6)), [y(i,1);y(i,2);y(i,3)] ];
        TopPointsInTime(:,:,i) = R_top_base * TP;
        [AccelerationSolutions(i,:), F_joint_InT(:,:,i), L_vector_InT(:,:,i)] = ...
            platform3DFunction_AL(t(i),y(i,:)',Prm,F_load,des_case);
        end
end



switch define_simmechanics
    case 1;
        
        switch define_actuator_lock
            case 0
            %set_param('simmechtest3D','SimMechanicsOpenEditorOnUpdate','off');
            tic;
            %simulinkout = sim('simmechtest3D','SimMechanicsOpenEditorOnUpdate','on');
            simulinkout = sim('simmechtest3D','SimMechanicsOpenEditorOnUpdate','on');
            SM_time = toc;
            disp(['Simmechanics simulation time: ',num2str(SM_time)])
            %set_param('simmechtest3D','SimMechanicsOpenEditorOnUpdate','on');
            
            case 1
            %set_param('simmechtest3D_AL','SimMechanicsOpenEditorOnUpdate','off');
            tic;
            %simulinkout = sim('simmechtest3D_AL','SimMechanicsOpenEditorOnUpdate','on');
            simulinkout = sim('simmechtest3D_AL','SimMechanicsOpenEditorOnUpdate','on');
            SM_time = toc;
            disp(['Simmechanics simulation time: ',num2str(SM_time)])
            %set_param('simmechtest3D_AL','SimMechanicsOpenEditorOnUpdate','on');     
        end 
        logsout = simulinkout.get('logsout');
        
        SM_x_ddt = get(logsout,'SM_x_ddt');
        SM_y_ddt = get(logsout,'SM_y_ddt');
        SM_z_ddt = get(logsout,'SM_z_ddt');
        SM_F_joint = get(logsout,'SM_F_joint');
        SM_L1_x_pos = get(logsout,'SM_L1_x_pos');
        SM_L1_y_pos = get(logsout,'SM_L1_y_pos');
        SM_L1_z_pos = get(logsout,'SM_L1_z_pos');
        SM_base_quaternian = get(logsout,'SM_base_quaternian');
        SM_Phi_ddt_2 = get(logsout,'SM_Phi_ddt_2');
        SM_Phi_dot_1 = get(logsout,'SM_Phi_dot_1');
        SM_Theta_dot_2 = get(logsout,'SM_Theta_dot_2');
        SM_Theta_ddt_2 = get(logsout,'SM_Theta_ddt_2');
        SM_alpha_ddt = get(logsout,'SM_alpha_ddt');
        SM_beta_ddt = get(logsout,'SM_beta_ddt');
        SM_gamma_ddt = get(logsout,'SM_gamma_ddt');
        SM_x = get(logsout,'SM_x');
        SM_y = get(logsout,'SM_y');
        SM_z = get(logsout,'SM_z');
end

%% Figures

scrsz = get(groot,'ScreenSize');

figure('Position',[scrsz(1)-900 scrsz(4)-1420 scrsz(3)/3 scrsz(4)/2],'Name','Platform'); 
subplot(1,3,1); hold on; grid on;
plot(t,AccelerationSolutions(:,13),'linewidth',2);
plot(t,AccelerationSolutions(:,14),'linewidth',2);
plot(t,AccelerationSolutions(:,15),'linewidth',2);
try
    plot(SM_x_ddt.Values.Time, SM_x_ddt.Values.Data,'linewidth',2);
    plot(SM_y_ddt.Values.Time, SM_y_ddt.Values.Data,'linewidth',2);
    plot(SM_z_ddt.Values.Time, SM_z_ddt.Values.Data,'linewidth',2);
    legend('x ddt','y ddt','z ddt','SM x ddt','SM y ddt','SM z ddt')
catch
    legend('x ddt','y ddt','z ddt');
end

subplot(1,3,2); hold on; grid on;
plot(t,AccelerationSolutions(:,16),'linewidth',2);
plot(t,AccelerationSolutions(:,17),'linewidth',2);
plot(t,AccelerationSolutions(:,18),'linewidth',2);
try
    plot(SM_alpha_ddt.Values.Time, SM_alpha_ddt.Values.Data,'linewidth',2);
    plot(SM_beta_ddt.Values.Time, SM_beta_ddt.Values.Data,'linewidth',2);
    plot(SM_gamma_ddt.Values.Time, SM_gamma_ddt.Values.Data,'linewidth',2);
    legend('\alpha ddt','\beta ddt','\gamma ddt', 'SM \alpha ddt', 'SM \beta ddt', 'SM \gamma ddt')
catch
    legend('\alpha ddt','\beta ddt','\gamma ddt')
end

subplot(1,3,3); hold on; grid on;
plot(t,y(:,1),'linewidth',2);
plot(t,y(:,2),'linewidth',2);
plot(t,y(:,3),'linewidth',2);
try
    plot(SM_x.Values.Time, SM_x.Values.Data,'linewidth',2);
    plot(SM_y.Values.Time, SM_y.Values.Data,'linewidth',2);
    plot(SM_z.Values.Time, SM_z.Values.Data,'linewidth',2);
    legend('x','y','z', 'SM x', 'SM y', 'SM z')
catch
    legend('x','y','z')
end

%savefig('./results/Results_Platform.fig')



%% Wrap Up

AnimatePlatform  %loops 3 times

%fileName = './results/ODE_Results.mat';
%save(fileName,'t','y')

rmpath('./functions');
%rmpath('./results');
