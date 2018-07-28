%%%%%%%%%%%%%%%%%%%%%
%%% EquationGen3D %%%
%%%%%%%%%%%%%%%%%%%%%

%clear all
tic

%% Declarations

N_legs   = 6;
TP       = sym('TP',      [4 N_legs],'real');
TP(4,:)  = ones(1,N_legs);
BP       = sym('BP',      [3 N_legs],'real');
F        = sym('F',       [1,N_legs],'real');
F_lock   = sym('F_lock',  [1,N_legs],'real');
Fr       = sym('Fr',      [3,N_legs],'real');
L_vector = sym('L_vector',[3,N_legs],'real');

zb = sym('zb',[1 N_legs],'real');
xb = sym('xb',[1 N_legs],'real');
yb = sym('yb',[1 N_legs],'real');
xt = sym('xt',[1 N_legs],'real');
yt = sym('yt',[1 N_legs],'real');
zt = sym('zt',[1 N_legs],'real');
xt_dot = sym('xt_dot',[1 N_legs],'real');
yt_dot = sym('yt_dot',[1 N_legs],'real');
zt_dot = sym('zt_dot',[1 N_legs],'real');

d1ddt     = sym('d1ddt',     [1 N_legs],'real');
d1ddt_AL  = sym('d1ddt_AL',     [1 N_legs],'real');
d2ddt     = sym('d2ddt',     [1 N_legs],'real');
d2ddt_AL  = sym('d2ddt_AL',     [1 N_legs],'real');
theta     = sym('theta',     [1 N_legs],'real');
theta_dot = sym('theta_dot', [1 N_legs],'real');
thetaddt  = sym('thetaddt',  [1 N_legs],'real');
phi       = sym('phi',     [1 N_legs],'real');
phi_dot   = sym('phi_dot', [1 N_legs],'real');
phiddt    = sym('phiddt',  [1 N_legs],'real');

l4ddt    = sym('l4ddt',   [3 N_legs],'real');
d1_      = sym('d1_',     [1,N_legs],'real');
d1dot_   = sym('d1dot_',   [1,N_legs],'real'); 
d2_      = sym('d2_',     [1,N_legs],'real');
d2dot_   = sym('d2dot_',  [1,N_legs],'real');
r2_      = sym('r2_',     [1,N_legs],'real');
r3_      = sym('r3_',     [1,N_legs],'real');
Prmk_    = sym('Prmk_',   [1,N_legs],'real');
PrmB_    = sym('PrmB_',   [1,N_legs],'real');

syms Prml1 Prml2 Prml3 Prmlc Prmg   real
syms PrmIx PrmIy PrmIz              real
syms Prmm1 Prmm2 Prmm3 Prmmp        real

syms x y z alpha beta gamma       real
syms x_dot y_dot z_dot            real
syms alpha_dot beta_dot gamma_dot real
syms x_ddt y_ddt z_ddt            real
syms alpha_ddt beta_ddt gamma_ddt real
syms F_load_x F_load_y F_load_z   real
syms M_load_x M_load_y            real

Rot_x = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
Rot_y = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
Rot_z = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];

R_inv = Rot_y *Rot_x *Rot_z;                                % top to global
R_fwd = transpose(Rot_z)*transpose(Rot_x)*transpose(Rot_y); % global to top


%% Platform


%%%% Platform DDT equations
temp1 = PrmIx*cos(gamma)^2 + PrmIy*sin(gamma)^2;
temp2 = (PrmIx - PrmIy)*cos(alpha)*cos(gamma)*sin(gamma);
temp4 = -PrmIz*sin(alpha)^2;
temp3 = cos(alpha)^2*(PrmIx*sin(gamma)^2 + PrmIy*cos(gamma)^2)-temp4;

K_platform = 0.5*[x_dot y_dot z_dot alpha_dot beta_dot gamma_dot] * ...
    [Prmmp 0 0 0 0 0
     0 Prmmp 0 0 0 0
     0 0 Prmmp 0 0 0
     0 0 0 temp1 temp2 0
     0 0 0 temp2 temp3 temp4
     0 0 0 0 temp4 PrmIz] *  ...
     [x_dot; y_dot; z_dot; alpha_dot; beta_dot; gamma_dot];

     clear temp1 temp2 temp3 temp4
 
platform_dxdot = diff(K_platform,x_dot);
platform_xddt = diff(platform_dxdot,x_dot)*x_ddt - F_load_x;
 
platform_dydot = diff(K_platform,y_dot);
platform_yddt = diff(platform_dydot,y_dot)*y_ddt - F_load_y;

platform_dzdot = diff(K_platform,z_dot);
platform_zddt = diff(platform_dzdot,z_dot)*z_ddt - F_load_z...
    + Prmmp*Prmg;

platform_dalphadot = diff(K_platform,alpha_dot);
platform_alphaddt  = diff(platform_dalphadot,alpha)*alpha_dot ...
    + diff(platform_dalphadot,beta)*beta_dot ...
    + diff(platform_dalphadot,gamma)*gamma_dot...
    + diff(platform_dalphadot,alpha_dot)*alpha_ddt ...
    + diff(platform_dalphadot,beta_dot)*beta_ddt ...
    + diff(platform_dalphadot,gamma_dot)*gamma_ddt ...
    - diff(K_platform,alpha)- M_load_x;

platform_dbetadot = diff(K_platform,beta_dot);
platform_betaddt = diff(platform_dbetadot,alpha)*alpha_dot ...
    + diff(platform_dbetadot,beta)*beta_dot ...
    + diff(platform_dbetadot,gamma)*gamma_dot...
    + diff(platform_dbetadot,alpha_dot)*alpha_ddt ...
    + diff(platform_dbetadot,beta_dot)*beta_ddt ...
    + diff(platform_dbetadot,gamma_dot)*gamma_ddt ...
    - diff(K_platform,beta)- M_load_y;

platform_dgammadot = diff(K_platform,gamma_dot);
platform_gammaddt = diff(platform_dgammadot,alpha)*alpha_dot ...
    + diff(platform_dgammadot,beta)*beta_dot ...
    + diff(platform_dgammadot,gamma)*gamma_dot...
    + diff(platform_dgammadot,alpha_dot)*alpha_ddt ...
    + diff(platform_dgammadot,beta_dot)*beta_ddt ...
    + diff(platform_dgammadot,gamma_dot)*gamma_ddt ...
    - diff(K_platform,gamma);


                      %%% J1 and J2 for forces

% J1
J1 = sym(zeros(6,18));
for i = 1:N_legs
J1(i,3*i-2:3*i) = -Fr(:,i)./sqrt(sum(Fr(:,i).*Fr(:,i)));
end

% J2

TP_transformation =   [ R_inv, [x;y;z] ]; %R (top to base)
                    
   


% TP Current
TP_current = TP_transformation * TP;

J2 = jacobian(reshape(TP_current,18,1),[x y z alpha beta gamma]);
% J = J1*J2;
% J_T = transpose(J);

Platform_rhs = transpose(J2) * reshape(Fr,[18 1]);

% TP_zbase = TP(1:3,:); F_zbase = R_fwd*-Fr;
% platform_moment_zbase = sum(  transpose(cross(TP_zbase, F_zbase))  );
% 
% Platform_rhs = [Platform_rhs(1:3); transpose(platform_moment_zbase)];

Platform_eq = [platform_xddt; platform_yddt; platform_zddt; ...
    platform_alphaddt; platform_betaddt; platform_gammaddt] == Platform_rhs;

matlabFunction(Platform_eq,'File','Generate_PlatformEquations')

%% TP dot and  TP ddt

TP_dot     = simplify( diff(TP_current,alpha)*alpha_dot ...
    + diff(TP_current,beta)*beta_dot ...
    + diff(TP_current,gamma)*gamma_dot ...
    + diff(TP_current,x)*x_dot ... 
    + diff(TP_current,y)*y_dot ...
    + diff(TP_current,z)*z_dot, 'Steps',30);

matlabFunction(TP_dot,'File','Generate_TP_dot')

TP_ddt     = diff(TP_dot,alpha)*alpha_dot ...
    + diff(TP_dot,beta)*beta_dot ...
    + diff(TP_dot,gamma)*gamma_dot ...
    + diff(TP_dot,x)*x_dot ...
    + diff(TP_dot,y)*y_dot ...
    + diff(TP_dot,z)*z_dot ...
    + diff(TP_dot,alpha_dot)*alpha_ddt  ... 
    + diff(TP_dot,beta_dot)*beta_ddt  ... 
    + diff(TP_dot,gamma_dot)*gamma_ddt  ... 
    + diff(TP_dot,x_dot)*x_ddt ...
    + diff(TP_dot,y_dot)*y_ddt ...
    + diff(TP_dot,z_dot)*z_ddt;
        
            %TP_ddt_initial = subs(TP_ddt,[alpha beta gamma alpha_dot beta_dot gamma_dot x_dot y_dot z_dot],[0 0 0 0 0 0 0 0 0]);
            
matlabFunction(TP_ddt,'File','Generate_TP_ddt')
            
            
%% Theta & Theta_dot

theta = atan2((yt-yb),(xt-xb));
theta_dot = transpose(jacobian(theta,[xt yt])*[transpose(xt_dot); transpose(yt_dot)]);

matlabFunction(theta,'File','Generate_Theta')


theta_glob = subs(theta,[xt, yt],[TP_current(1,:) TP_current(2,:)]);
theta_dot_glob = subs(theta_dot,[xt, yt, xt_dot, yt_dot], ...
    [TP_current(1,:) TP_current(2,:) TP_dot(1,:) TP_dot(2,:)]);

matlabFunction(theta_glob,'File','Generate_Theta_Glob')
matlabFunction(theta_dot_glob,'File','Generate_Theta_Dot_Glob')

%% Phi & Phi_dot

phi = atan2( zt-zb,sqrt( (xt-xb).^2 + (yt-yb).^2) );
phi_dot = transpose(jacobian(phi,[xt yt zt])*[transpose(xt_dot); transpose(yt_dot); transpose(zt_dot)]);

phi_glob = subs(phi,[xt, yt zt],[TP_current(1,:) TP_current(2,:) TP_current(3,:)]);
phi_dot_glob = simplify(subs(phi_dot,[xt, yt, zt, xt_dot, yt_dot, zt_dot], ...
    [TP_current(1,:) TP_current(2,:) TP_current(3,:) TP_dot(1,:) TP_dot(2,:) TP_dot(3,:)]),'Steps',10);

matlabFunction(phi_glob,'File','Generate_Phi_Glob')
matlabFunction(phi_dot_glob,'File','Generate_Phi_Dot_Glob')

%% d1 & d2

%%ddt 1
t_phi       = sym('t_phi',[1 N_legs],'real');
t_phi_dot   = sym('t_phi_dot',[1 N_legs],'real');
t_theta     = sym('t_theta',[1 N_legs],'real');
t_theta_dot = sym('t_theta_dot',[1 N_legs],'real');

d1ddt = ( (Prmm2+Prmm1) .* ( r2_.*t_phi_dot.^2   +  r2_.*cos(t_phi).^2 .* t_theta_dot.^2  ) ...
    - F - Prmk_.*d1_ - Prmg.*(Prmm2+Prmm1/2).*sin(t_phi) - PrmB_.*d1dot_ ) / (Prmm2+Prmm1);   %%%%%%%% check the /2 on the gravity???
d1_ddt_glob = subs(d1ddt,[t_theta t_theta_dot t_phi t_phi_dot],[theta_glob theta_dot_glob phi_glob phi_dot_glob]);

matlabFunction(d1_ddt_glob,'File','Generate_D1_Ddt_Glob')

%%ddt 2
d2ddt =   (-Prmm3*d1ddt ...
    + Prmm3*r3_.*(t_phi_dot.^2 + t_theta_dot.^2.*cos(t_phi).^2) ...
    + F ...
    + dot( Fr, L_vector./  repmat(sqrt(sum(L_vector.*L_vector)),[3,1]) ) ...
    - Prmg*Prmm3*sin(t_phi)  )/Prmm3;

d2_ddt_glob = subs(d2ddt,[t_theta t_theta_dot t_phi t_phi_dot],[theta_glob theta_dot_glob phi_glob phi_dot_glob]);

matlabFunction(d2_ddt_glob,'File','Generate_D2_Ddt_Glob')

%%F_lock

d1ddt_AL = ((Prmm2+Prmm1) .* ( r2_.*t_phi_dot.^2   +  r2_.*cos(t_phi).^2 .* t_theta_dot.^2  ) ...
     - F_lock - Prmk_.*d1_ - Prmg.*(Prmm2+Prmm1/2).*sin(t_phi) - PrmB_.*d1dot_ ) / (Prmm2+Prmm1);
d1_ddt_AL_glob = subs(d1ddt_AL,[t_theta t_theta_dot t_phi t_phi_dot],[theta_glob theta_dot_glob phi_glob phi_dot_glob]);

matlabFunction(d1_ddt_AL_glob,'File','Generate_D1_Ddt_AL_Glob')

d2ddt_AL = (-Prmm3*d1_ddt_AL_glob ...
    + Prmm3*r3_.*(t_phi_dot.^2 + t_theta_dot.^2.*cos(t_phi).^2) ...
    + F_lock ...
    + dot( Fr, L_vector./  repmat(sqrt(sum(L_vector.*L_vector)),[3,1]) ) ...
    - Prmg*Prmm3*sin(t_phi)  )/Prmm3;
d2_ddt_AL_glob = subs(d2ddt_AL,[t_theta t_theta_dot t_phi t_phi_dot],[theta_glob theta_dot_glob phi_glob phi_dot_glob]);


% temp0 = (-Prmm3*d1ddt ...
%     + Prmm3*r3_.*(t_phi_dot.^2 + t_theta_dot.^2.*cos(t_phi).^2) ...
%     + F_lock ...
%     + dot( Fr, L_vector./  repmat(sqrt(sum(L_vector.*L_vector)),[3,1]) ) ...
%     - Prmg*Prmm3*sin(t_phi)  )/Prmm3 ...
%     == ...
%     ((Prmm2+Prmm1) .* ( r2_.*t_phi_dot.^2   +  r2_.*cos(t_phi).^2 .* t_theta_dot.^2  ) ...
%     - F_lock - Prmk_.*d1_ - Prmg.*(Prmm2+Prmm1/2).*sin(t_phi) - PrmB_.*d1dot_ ) / (Prmm2+Prmm1);
%     
% [A, b] = equationsToMatrix(temp0,F_lock(1:end));
% temp1 = linsolve(A,b);
% F_lock_eq_glob = subs(temp1,[t_theta t_theta_dot t_phi t_phi_dot],[theta_glob theta_dot_glob phi_glob phi_dot_glob]);
% matlabFunction(F_lock_eq_glob,'File','Calculate_F_lock')
% clear A b temp0 temp1 temp2

%% Phi ddt

Rotated_Fr = sym('Rot_Fr',[3 6], 'real');
Rotated_L_vector = sym('Rotated_L_vector',[3 6],'real');

for i = 1:N_legs
Rotated_L_vector(:,i) = transpose([cos(t_theta(i)) -sin(t_theta(i)) 0; sin(t_theta(i)) cos(t_theta(i)) 0; 0 0 1])*L_vector(:,i);
Rotated_Fr(:,i)       = transpose([cos(t_theta(i)) -sin(t_theta(i)) 0; sin(t_theta(i)) cos(t_theta(i)) 0; 0 0 1])*Fr(:,i);
end


moment = cross(Rotated_Fr,Rotated_L_vector);

func = phiddt.*(  Prmm1*((r2_-Prml2)/2).^2 ...
    + Prmm2*r2_.^2 ...
    + Prmm3*r3_.^2  ) ...
    + 2*t_phi_dot.*d1dot_.*(Prmm1/2*(r2_-Prml2)/2 + Prmm2*r2_ + Prmm3*r3_) ...
    + 2*t_phi_dot.*d2dot_.*(Prmm3*r3_) == moment(2,:) ...
    - Prmg*cos(t_phi).*( Prmm1*(r2_-Prml2)/2 + Prmm2*r2_ + Prmm3*r3_ ) ...
    - t_theta_dot.^2.*sin(t_phi).*cos(t_phi).*(Prmm1*((r2_-Prml2)/2).^2 + Prmm2*r2_.^2 + Prmm3*r3_.^2) ;

[A,b] = equationsToMatrix(func,phiddt);
temp1 = rref([A,b]);
temp2 = subs(temp1,[t_phi t_phi_dot t_theta t_theta_dot],[phi phi_dot theta theta_dot_glob]);
phi_ddt_glob = transpose(temp2(:,end));
clear temp1 temp2

matlabFunction(phi_ddt_glob,'File','Generate_Phi_Ddt_Glob')

%% Theta ddt

moment = cross(L_vector,Fr);

func = thetaddt.*cos(t_phi).^2.*(  Prmm1*((r2_-Prml2)/2).^2 ...
    + Prmm2*r2_.^2 ...
    + Prmm3*r3_.^2  ) ...
    - 2*t_theta_dot.*t_phi_dot.*cos(t_phi).*sin(t_phi).*(Prmm1*((r2_-Prml2)/2).^2 + Prmm2*r2_.^2 + Prmm3*r3_.^2) ...
    + 2*t_theta_dot.*d1dot_.* cos(t_phi).^2 .* (Prmm1*((r2_-Prml2)/2) + Prmm2*r2_ + Prmm3*r3_) ...
    + 2*t_theta_dot.*d2dot_.* cos(t_phi).^2 .* (Prmm3*r3_) ...
      == moment(3,:);
  
[A,b] = equationsToMatrix(func,thetaddt);
temp1 = rref([A,b]);
matlabFunction(temp1(:,end),'File','Generate_Theta_Ddt')
%temp2 = subs(temp1,[t_phi t_phi_dot t_theta t_theta_dot],[phi phi_dot theta theta_dot_glob]);
temp2 = subs(temp1,[t_phi_dot t_theta t_theta_dot],[phi_dot theta theta_dot_glob]);
theta_ddt_glob = transpose(temp2(:,end));
clear temp1 temp2

matlabFunction(theta_ddt_glob,'File','Generate_Theta_Ddt_Glob')


%% L4ddt

l4ddt       = sym('l4ddt',[3 N_legs],'real');
t_theta_ddt = sym('t_theta_ddt',[1 N_legs],'real');
t_phi_ddt   = sym('t_phi_ddt',  [1 N_legs],'real');
t_d2_ddt    = sym('t_d2_ddt',[1 N_legs],'real');
t_d1_ddt    = sym('t_d1_ddt',[1 N_legs],'real');

for i = 1:N_legs
Ry = [cos(t_phi(i)) 0 -sin(t_phi(i)); 0 1 0; sin(t_phi(i)) 0 cos(t_phi(i))];
Rz = [cos(t_theta(i)) -sin(t_theta(i)) 0; sin(t_theta(i)) cos(t_theta(i)) 0; 0 0 1];
l4 = [Rz*Ry,[xb(i); yb(i); zb(i)]] * [Prmlc+d1_(i)+d2_(i); 0 ; 0; 1];

l4_dot = diff(l4,t_theta(i))*t_theta_dot(i) ...
       + diff(l4,t_phi(i))*t_phi_dot(i) ...
       + diff(l4,d2_(i))*d2dot_(i) ...
       + diff(l4,d1_(i))*d1dot_(i);

l4ddt(:,i) = diff(l4_dot,t_theta(i)) *t_theta_dot(i) ...
      + diff(l4_dot,t_phi(i)) *t_phi_dot(i) ...
      + diff(l4_dot,d2_(i))*d2dot_(i) ...
      + diff(l4_dot,d1_(i))*d1dot_(i) ...
      + diff(l4_dot,t_theta_dot(i))*t_theta_ddt(i) ...
      + diff(l4_dot,t_phi_dot(i))*t_phi_ddt(i) ...
      + diff(l4_dot,d2dot_(i))*t_d2_ddt(i) ...
      + diff(l4_dot,d1dot_(i))*t_d1_ddt(i);

end

disp('generating l4ddt_glob')

l4_ddt_general = subs(l4ddt,[t_theta t_theta_dot t_theta_ddt t_phi t_phi_dot t_phi_ddt],...
           [theta theta_dot_glob theta_ddt_glob phi phi_dot_glob phi_ddt_glob]);
       
l4_ddt_glob    = subs(l4_ddt_general,[t_d1_ddt t_d2_ddt],[d1_ddt_glob d2_ddt_glob]);  
l4_ddt_AL_glob = subs(l4_ddt_general,[t_d1_ddt t_d2_ddt],[d1_ddt_AL_glob d2_ddt_AL_glob]);  

% temp1 = subs(l4ddt,[t_d1_ddt t_d2_ddt t_theta_ddt t_phi_ddt],[d1_ddt_glob d2_ddt_glob theta_ddt_glob phi_ddt_glob]);
% matlabFunction(temp1,'File','Generate_l4_Ddt')
matlabFunction(l4_ddt_glob,'File','Generate_l4_Ddt_Glob')
    
testy_glob    = TP_ddt == l4_ddt_glob;
testy_AL_glob = TP_ddt == l4_ddt_AL_glob;

[A, b] = equationsToMatrix([Platform_eq; transpose(testy_glob(1:end))],[x_ddt y_ddt z_ddt alpha_ddt beta_ddt gamma_ddt Fr(1:end)]);
matlabFunction(A,'File','Generate_combinedEqA_3d_glob')
matlabFunction(b,'File','Generate_combinedEqb_3d_glob')

d2_condition = d2_ddt_AL_glob == 0;
[A1, b1] = equationsToMatrix([Platform_eq; transpose(testy_AL_glob(1:end));transpose(d2_condition)],[x_ddt y_ddt z_ddt alpha_ddt beta_ddt gamma_ddt F_lock Fr(1:end)]);
matlabFunction(A1,'File','Generate_combinedEqA_3d_AL_glob')
matlabFunction(b1,'File','Generate_combinedEqb_3d_AL_glob')

toc

load handel
sound(y,Fs)