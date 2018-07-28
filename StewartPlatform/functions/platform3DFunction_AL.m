function [ q_prime, Fr, L_vector ] = platform3DFunction_AL(t,q,Prm,F_load,des_case)


TP = Prm.TP;
Generated_R = Generate_R(q(4),q(5),q(6));
TP_Current = [ Generated_R, [q(1);q(2);q(3)] ] * TP;
L_vector = TP_Current - Prm.BP;
[~,~, L_length] = cart2sph(L_vector(1,:),L_vector(2,:),L_vector(3,:));

TP_dot = Generate_TP_dot(TP(1,1),TP(1,2),TP(1,3),TP(1,4),TP(1,5),TP(1,6),TP(2,1),TP(2,2),TP(2,3),TP(2,4),TP(2,5),TP(2,6),TP(3,1),TP(3,2),TP(3,3),TP(3,4),TP(3,5),TP(3,6),q(4),q(16),q(5),q(17),q(6),q(18),q(13),q(14),q(15));
L_dot  = dot(TP_dot,L_vector./repmat(sqrt(sum(L_vector.*L_vector)),[3 1]));

r2 = 2*Prm.l1+q(7:12)+Prm.l2;
r3 = 2*Prm.l1+q(7:12)+Prm.l2+(L_length'-Prm.lc-q(7:12))+Prm.l3;

d1 = q(7:12);
d1dot = q(19:24);
d2 = (L_length'-Prm.lc-q(7:12));
d2dot = L_dot'-q(19:24);


%% debug zone

if strcmp(des_case,'lean')
    F_lean = Generated_R * F_load';
    [F_load_x, F_load_y, F_load_z] = deal(F_lean(1),F_lean(2),F_lean(3));
else
    [F_load_x, F_load_y, F_load_z] = deal(F_load(1),F_load(2),F_load(3));
end

[x_dot, y_dot, z_dot] = deal(q(13),q(14),q(15));
[Prmlc,Prmmp,alpha,alpha_dot,beta,beta_dot,gamma,gamma_dot] = deal(Prm.lc,Prm.mp,q(4),q(16),q(5),q(17),q(6),q(18));
[Prmg,Prml2,Prmm1,Prmm2,Prmm3,PrmIx,PrmIy,PrmIz] = deal(Prm.g,Prm.l2,Prm.m1,Prm.m2,Prm.m3,Prm.I_x,Prm.I_y,Prm.I_z);
temp0=num2cell(Prm.k);  [Prmk_1,Prmk_2,Prmk_3,Prmk_4,Prmk_5,Prmk_6]=deal(temp0{:});
temp1=num2cell(Prm.B);  [PrmB_1,PrmB_2,PrmB_3,PrmB_4,PrmB_5,PrmB_6]=deal(temp1{:});
clear temp0 temp1

[TP1_1,TP1_2,TP1_3,TP1_4,TP1_5,TP1_6,TP2_1,TP2_2,TP2_3,TP2_4,TP2_5,TP2_6,TP3_1,TP3_2,TP3_3,TP3_4,TP3_5,TP3_6] = ...
    deal(TP(1,1),TP(1,2),TP(1,3),TP(1,4),TP(1,5),TP(1,6),TP(2,1),TP(2,2),TP(2,3),TP(2,4),TP(2,5),TP(2,6),TP(3,1),TP(3,2),TP(3,3),TP(3,4),TP(3,5),TP(3,6));

temp1 = num2cell(d1); temp2 = num2cell(d2); temp3 = num2cell(d1dot); temp4 = num2cell(d2dot);
[d1_1,d1_2,d1_3,d1_4,d1_5,d1_6,d2_1,d2_2,d2_3,d2_4,d2_5,d2_6,...
    d1dot_1,d1dot_2,d1dot_3,d1dot_4,d1dot_5,d1dot_6,d2dot_1,d2dot_2,d2dot_3,d2dot_4,d2dot_5,d2dot_6] = ...
    deal(temp1{:},temp2{:},temp3{:},temp4{:});

temp5 = num2cell(r2); temp6 = num2cell(r3);
temp7 = num2cell([Prm.BP(1,:) TP_Current(1,:)]);
temp8 = num2cell([Prm.BP(2,:) TP_Current(2,:)]);
temp9 = num2cell([Prm.BP(3,:) TP_Current(3,:)]);
[r2_1,r2_2,r2_3,r2_4,r2_5,r2_6,r3_1,r3_2,r3_3,r3_4,r3_5,r3_6,...
    x,xb1,xb2,xb3,xb4,xb5,xb6,xt1,xt2,xt3,xt4,xt5,xt6, ...
    y,yb1,yb2,yb3,yb4,yb5,yb6,yt1,yt2,yt3,yt4,yt5,yt6, ...
    z,zb1,zb2,zb3,zb4,zb5,zb6,zt1,zt2,zt3,zt4,zt5,zt6] = ...
    deal(temp5{:},temp6{:},...
    q(1),temp7{:},...
    q(2),temp8{:},...
    q(3),temp9{:});

temp10 = transpose(L_vector); temp11 = num2cell(temp10(1:end));
[L_vector1_1,L_vector1_2,L_vector1_3,L_vector1_4,L_vector1_5,L_vector1_6,L_vector2_1,L_vector2_2,L_vector2_3,L_vector2_4,...
    L_vector2_5,L_vector2_6,L_vector3_1,L_vector3_2,L_vector3_3,L_vector3_4,L_vector3_5,L_vector3_6] = deal(temp11{:});
clear temp0 temp1 temp2 temp3 temp4 temp5 temp6 temp7 temp8 temp9 temp10 temp11

temp12 = num2cell(TP_dot(1,:)); temp13 = num2cell(TP_dot(2,:)); temp14 = num2cell(TP_dot(3,:));
[xt_dot1,xt_dot2,xt_dot3,xt_dot4,xt_dot5,xt_dot6] = deal(temp12{:});
[yt_dot1,yt_dot2,yt_dot3,yt_dot4,yt_dot5,yt_dot6] = deal(temp13{:});
[zt_dot1,zt_dot2,zt_dot3,zt_dot4,zt_dot5,zt_dot6] = deal(temp14{:});
clear temp12 temp13 temp14

debug_phi = Generate_Phi_Glob(TP1_1,TP1_2,TP1_3,TP1_4,TP1_5,TP1_6,TP2_1,TP2_2,TP2_3,TP2_4,TP2_5,TP2_6,TP3_1,TP3_2,TP3_3,TP3_4,TP3_5,TP3_6,alpha,beta,gamma,x,xb1,xb2,xb3,xb4,xb5,xb6,y,yb1,yb2,yb3,yb4,yb5,yb6,z,zb1,zb2,zb3,zb4,zb5,zb6);
 temp15 = num2cell(debug_phi); 
 [t_phi1,t_phi2,t_phi3,t_phi4,t_phi5,t_phi6] = deal(temp15{:});


%% solution

A1 = Generate_combinedEqA_3d_AL_glob(L_vector1_1,L_vector1_2,L_vector1_3,L_vector1_4,L_vector1_5,L_vector1_6,L_vector2_1,L_vector2_2,L_vector2_3,L_vector2_4,L_vector2_5,L_vector2_6,L_vector3_1,L_vector3_2,L_vector3_3,L_vector3_4,L_vector3_5,L_vector3_6,Prml2,Prmm1,Prmm2,Prmm3,PrmIx,PrmIy,PrmIz,Prmlc,Prmmp,TP1_1,TP1_2,TP1_3,TP1_4,TP1_5,TP1_6,TP2_1,TP2_2,TP2_3,TP2_4,TP2_5,TP2_6,TP3_1,TP3_2,TP3_3,TP3_4,TP3_5,TP3_6,alpha,beta,d1_1,d1_2,d1_3,d1_4,d1_5,d1_6,d2_1,d2_2,d2_3,d2_4,d2_5,d2_6,gamma,r2_1,r2_2,r2_3,r2_4,r2_5,r2_6,r3_1,r3_2,r3_3,r3_4,r3_5,r3_6,t_phi1,t_phi2,t_phi3,t_phi4,t_phi5,t_phi6,xb1,xb2,xb3,xb4,xb5,xb6,xt1,xt2,xt3,xt4,xt5,xt6,yb1,yb2,yb3,yb4,yb5,yb6,yt1,yt2,yt3,yt4,yt5,yt6,zb1,zb2,zb3,zb4,zb5,zb6,zt1,zt2,zt3,zt4,zt5,zt6);
b1 = Generate_combinedEqb_3d_AL_glob(F_load_x,F_load_y,F_load_z,Prmg,Prml2,Prmm1,Prmm2,Prmm3,PrmB_1,PrmB_2,PrmB_3,PrmB_4,PrmB_5,PrmB_6,PrmIx,PrmIy,PrmIz,Prmk_1,Prmk_2,Prmk_3,Prmk_4,Prmk_5,Prmlc,Prmk_6,Prmmp,TP1_1,TP1_2,TP1_3,TP1_4,TP1_5,TP1_6,TP2_1,TP2_2,TP2_3,TP2_4,TP2_5,TP2_6,TP3_1,TP3_2,TP3_3,TP3_4,TP3_5,TP3_6,alpha,alpha_dot,beta,beta_dot,d1_1,d1_2,d1_3,d1_4,d1_5,d1_6,d2_1,d2_2,d2_3,d2_4,d2_5,d2_6,d1dot_1,d1dot_2,d1dot_3,d1dot_4,d1dot_5,d1dot_6,d2dot_1,d2dot_2,d2dot_3,d2dot_4,d2dot_5,d2dot_6,gamma,gamma_dot,r2_1,r2_2,r2_3,r2_4,r2_5,r2_6,r3_1,r3_2,r3_3,r3_4,r3_5,r3_6,t_phi1,t_phi2,t_phi3,t_phi4,t_phi5,t_phi6,x,x_dot,xb1,xb2,xb3,xb4,xb5,xb6,xt1,xt2,xt3,xt4,xt5,xt6,xt_dot1,xt_dot2,xt_dot3,xt_dot4,xt_dot5,xt_dot6,y,y_dot,yb1,yb2,yb3,yb4,yb5,yb6,yt1,yt2,yt3,yt4,yt5,yt6,yt_dot1,yt_dot2,yt_dot3,yt_dot4,yt_dot5,yt_dot6,z,z_dot,zb1,zb2,zb3,zb4,zb5,zb6,zt1,zt2,zt3,zt4,zt5,zt6,zt_dot1,zt_dot2,zt_dot3,zt_dot4,zt_dot5,zt_dot6);

%b1(4) = 6;
%disp(['b1(4) arb set to 6',num2str(b1(4))])

solu = linsolve(A1,b1);

[F_lock1,F_lock2,F_lock3,F_lock4,F_lock5,F_lock6] = deal(solu(7),solu(8),solu(9),solu(10),solu(11),solu(12));
res_d1_ddt = Generate_D1_Ddt_AL_Glob(F_lock1,F_lock2,F_lock3,F_lock4,F_lock5,F_lock6,Prmg,Prmm1,Prmm2,PrmB_1,PrmB_2,PrmB_3,PrmB_4,PrmB_5,PrmB_6,Prmk_1,Prmk_2,Prmk_3,Prmk_4,Prmk_5,Prmk_6,TP1_1,TP1_2,TP1_3,TP1_4,TP1_5,TP1_6,TP2_1,TP2_2,TP2_3,TP2_4,TP2_5,TP2_6,TP3_1,TP3_2,TP3_3,TP3_4,TP3_5,TP3_6,alpha,alpha_dot,beta,beta_dot,d1_1,d1_2,d1_3,d1_4,d1_5,d1_6,d1dot_1,d1dot_2,d1dot_3,d1dot_4,d1dot_5,d1dot_6,gamma,gamma_dot,r2_1,r2_2,r2_3,r2_4,r2_5,r2_6,x,x_dot,xb1,xb2,xb3,xb4,xb5,xb6,y,y_dot,yb1,yb2,yb3,yb4,yb5,yb6,z,z_dot,zb1,zb2,zb3,zb4,zb5,zb6);

% temp0 = num2cell(solu(7:24));
% [Fr1_1, Fr2_1, Fr3_1, Fr1_2, Fr2_2, Fr3_2, Fr1_3, Fr2_3, Fr3_3, ...
%  Fr1_4, Fr2_4, Fr3_4, Fr1_5, Fr2_5, Fr3_5, Fr1_6, Fr2_6, Fr3_6] = deal(temp0{:}); clear temp0;

q_prime = [q(13:24); solu(1:6); res_d1_ddt';solu(25:30)];
Fr = reshape(solu(7:24),[3,6]);
end