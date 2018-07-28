function temp1 = Generate_l4_Ddt(F1,F2,F3,F4,F5,F6,Fr1_1,Fr1_2,Fr1_3,Fr1_4,Fr1_5,Fr1_6,Fr2_1,Fr2_2,Fr2_3,Fr2_4,Fr2_5,Fr2_6,Fr3_1,Fr3_2,Fr3_3,Fr3_4,Fr3_5,Fr3_6,L_vector1_1,L_vector1_2,L_vector1_3,L_vector1_4,L_vector1_5,L_vector1_6,L_vector2_1,L_vector2_2,L_vector2_3,L_vector2_4,L_vector2_5,L_vector2_6,L_vector3_1,L_vector3_2,L_vector3_3,L_vector3_4,L_vector3_5,L_vector3_6,Prmg,Prml2,Prmm1,Prmm2,Prmm3,PrmB_1,PrmB_2,PrmB_3,PrmB_4,PrmB_5,PrmB_6,Prmk_1,Prmk_2,Prmk_3,Prmk_4,Prmk_5,Prmlc,Prmk_6,TP1_1,TP1_2,TP1_3,TP1_4,TP1_5,TP1_6,TP2_1,TP2_2,TP2_3,TP2_4,TP2_5,TP2_6,TP3_1,TP3_2,TP3_3,TP3_4,TP3_5,TP3_6,alpha,alpha_dot,beta,beta_dot,d1_1,d1_2,d1_3,d1_4,d1_5,d1_6,d2_1,d2_2,d2_3,d2_4,d2_5,d2_6,d1dot_1,d1dot_2,d1dot_3,d1dot_4,d1dot_5,d1dot_6,d2dot_1,d2dot_2,d2dot_3,d2dot_4,d2dot_5,d2dot_6,gamma,gamma_dot,r2_1,r2_2,r2_3,r2_4,r2_5,r2_6,r3_1,r3_2,r3_3,r3_4,r3_5,r3_6,t_phi1,t_phi2,t_phi3,t_phi4,t_phi5,t_phi6,t_phi_dot1,t_phi_dot2,t_phi_dot3,t_phi_dot4,t_phi_dot5,t_phi_dot6,t_theta1,t_theta2,t_theta3,t_theta4,t_theta5,t_theta6,t_theta_dot1,t_theta_dot2,t_theta_dot3,t_theta_dot4,t_theta_dot5,t_theta_dot6,x,x_dot,xb1,xb2,xb3,xb4,xb5,xb6,xt1,xt2,xt3,xt4,xt5,xt6,xt_dot1,xt_dot2,xt_dot3,xt_dot4,xt_dot5,xt_dot6,y,y_dot,yb1,yb2,yb3,yb4,yb5,yb6,yt1,yt2,yt3,yt4,yt5,yt6,yt_dot1,yt_dot2,yt_dot3,yt_dot4,yt_dot5,yt_dot6,z,z_dot,zb1,zb2,zb3,zb4,zb5,zb6,zt1,zt2,zt3,zt4,zt5,zt6,zt_dot1,zt_dot2,zt_dot3,zt_dot4,zt_dot5,zt_dot6)
%GENERATE_L4_DDT
%    TEMP1 = GENERATE_L4_DDT(F1,F2,F3,F4,F5,F6,FR1_1,FR1_2,FR1_3,FR1_4,FR1_5,FR1_6,FR2_1,FR2_2,FR2_3,FR2_4,FR2_5,FR2_6,FR3_1,FR3_2,FR3_3,FR3_4,FR3_5,FR3_6,L_VECTOR1_1,L_VECTOR1_2,L_VECTOR1_3,L_VECTOR1_4,L_VECTOR1_5,L_VECTOR1_6,L_VECTOR2_1,L_VECTOR2_2,L_VECTOR2_3,L_VECTOR2_4,L_VECTOR2_5,L_VECTOR2_6,L_VECTOR3_1,L_VECTOR3_2,L_VECTOR3_3,L_VECTOR3_4,L_VECTOR3_5,L_VECTOR3_6,PRMG,PRML2,PRMM1,PRMM2,PRMM3,PRMB_1,PRMB_2,PRMB_3,PRMB_4,PRMB_5,PRMB_6,PRMK_1,PRMK_2,PRMK_3,PRMK_4,PRMK_5,PRMLC,PRMK_6,TP1_1,TP1_2,TP1_3,TP1_4,TP1_5,TP1_6,TP2_1,TP2_2,TP2_3,TP2_4,TP2_5,TP2_6,TP3_1,TP3_2,TP3_3,TP3_4,TP3_5,TP3_6,ALPHA,ALPHA_DOT,BETA,BETA_DOT,D1_1,D1_2,D1_3,D1_4,D1_5,D1_6,D2_1,D2_2,D2_3,D2_4,D2_5,D2_6,D1DOT_1,D1DOT_2,D1DOT_3,D1DOT_4,D1DOT_5,D1DOT_6,D2DOT_1,D2DOT_2,D2DOT_3,D2DOT_4,D2DOT_5,D2DOT_6,GAMMA,GAMMA_DOT,R2_1,R2_2,R2_3,R2_4,R2_5,R2_6,R3_1,R3_2,R3_3,R3_4,R3_5,R3_6,T_PHI1,T_PHI2,T_PHI3,T_PHI4,T_PHI5,T_PHI6,T_PHI_DOT1,T_PHI_DOT2,T_PHI_DOT3,T_PHI_DOT4,T_PHI_DOT5,T_PHI_DOT6,T_THETA1,T_THETA2,T_THETA3,T_THETA4,T_THETA5,T_THETA6,T_THETA_DOT1,T_THETA_DOT2,T_THETA_DOT3,T_THETA_DOT4,T_THETA_DOT5,T_THETA_DOT6,X,X_DOT,XB1,XB2,XB3,XB4,XB5,XB6,XT1,XT2,XT3,XT4,XT5,XT6,XT_DOT1,XT_DOT2,XT_DOT3,XT_DOT4,XT_DOT5,XT_DOT6,Y,Y_DOT,YB1,YB2,YB3,YB4,YB5,YB6,YT1,YT2,YT3,YT4,YT5,YT6,YT_DOT1,YT_DOT2,YT_DOT3,YT_DOT4,YT_DOT5,YT_DOT6,Z,Z_DOT,ZB1,ZB2,ZB3,ZB4,ZB5,ZB6,ZT1,ZT2,ZT3,ZT4,ZT5,ZT6,ZT_DOT1,ZT_DOT2,ZT_DOT3,ZT_DOT4,ZT_DOT5,ZT_DOT6)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    19-Sep-2016 23:36:37

t2 = cos(t_theta1);
t3 = sin(t_phi1);
t4 = t2.*t3.*t_phi_dot1;
t5 = cos(t_phi1);
t6 = sin(t_theta1);
t7 = t5.*t6.*t_theta_dot1;
t8 = t4+t7;
t9 = Prmlc+d1_1+d2_1;
t10 = Prmm1+Prmm2;
t11 = cos(beta);
t12 = sin(gamma);
t13 = cos(gamma);
t14 = sin(alpha);
t15 = sin(beta);
t17 = cos(alpha);
t20 = t11.*t13;
t21 = t12.*t14.*t15;
t22 = t20+t21;
t23 = TP1_1.*t22;
t24 = t11.*t12;
t25 = t13.*t14.*t15;
t26 = t24-t25;
t27 = TP2_1.*t26;
t28 = TP3_1.*t15.*t17;
t16 = t23-t27+t28+x-xb1;
t30 = TP3_1.*t14;
t31 = TP2_1.*t13.*t17;
t32 = TP1_1.*t12.*t17;
t18 = -t30+t31+t32+y-yb1;
t34 = t12.*t15;
t35 = t11.*t13.*t14;
t36 = t34+t35;
t37 = t13.*t15;
t38 = t11.*t12.*t14;
t39 = t37-t38;
t41 = TP1_1.*t39;
t42 = TP2_1.*t36;
t43 = TP3_1.*t11.*t17;
t19 = -t41+t42+t43+z-zb1;
t29 = t16.^2;
t33 = t18.^2;
t40 = t29+t33;
t44 = t19.^2;
t45 = t29+t33+t44;
t46 = 1.0./t45;
t47 = 1.0./sqrt(t40);
t49 = sqrt(t40);
t50 = -t41+t42+t43;
t51 = beta_dot.*t50;
t52 = TP3_1.*t14.*t15;
t53 = TP2_1.*t13.*t15.*t17;
t54 = TP1_1.*t12.*t15.*t17;
t55 = -t52+t53+t54;
t56 = alpha_dot.*t55;
t57 = TP1_1.*t26;
t58 = TP2_1.*t22;
t59 = t57+t58;
t60 = gamma_dot.*t59;
t61 = t51+t56-t60+x_dot;
t63 = TP3_1.*t17;
t64 = TP2_1.*t13.*t14;
t65 = TP1_1.*t12.*t14;
t66 = t63+t64+t65;
t67 = alpha_dot.*t66;
t68 = TP1_1.*t13.*t17;
t69 = TP2_1.*t12.*t17;
t70 = t68-t69;
t71 = gamma_dot.*t70;
t72 = -t67+t71+y_dot;
t81 = t23-t27+t28;
t82 = beta_dot.*t81;
t83 = TP3_1.*t11.*t14;
t84 = TP2_1.*t11.*t13.*t17;
t85 = TP1_1.*t11.*t12.*t17;
t86 = -t83+t84+t85;
t87 = alpha_dot.*t86;
t88 = TP1_1.*t36;
t89 = TP2_1.*t39;
t90 = t88+t89;
t91 = gamma_dot.*t90;
t92 = -t82+t87+t91+z_dot;
t93 = t46.*t49.*t92;
t94 = x.*2.0;
t95 = xb1.*2.0;
t96 = TP1_1.*t22.*2.0;
t97 = TP2_1.*t26.*2.0;
t98 = TP3_1.*t15.*t17.*2.0;
t99 = t94-t95+t96-t97+t98;
t100 = t19.*t46.*t47.*t61.*t99.*(1.0./2.0);
t101 = y.*2.0;
t102 = yb1.*2.0;
t103 = TP3_1.*t14.*2.0;
t104 = TP2_1.*t13.*t17.*2.0;
t105 = TP1_1.*t12.*t17.*2.0;
t106 = t101-t102-t103+t104+t105;
t107 = t19.*t46.*t47.*t72.*t106.*(1.0./2.0);
t48 = -t93+t100+t107;
t62 = 1.0./t40;
t110 = t18.*t61.*t62;
t111 = t16.*t62.*t72;
t73 = t110-t111;
t74 = z.*1i;
t75 = zb1.*-1i;
t76 = TP1_1.*t39.*-1i;
t77 = TP2_1.*t36.*1i;
t78 = TP3_1.*t11.*t17.*1i;
t79 = t49+t74+t75+t76+t77+t78;
t80 = abs(t79);
t108 = t48.^2;
t109 = 1.0./t80.^2;
t112 = t73.^2;
t113 = L_vector1_1.^2;
t114 = L_vector2_1.^2;
t115 = L_vector3_1.^2;
t116 = t113+t114+t115;
t117 = 1.0./sqrt(t116);
t118 = 1.0./t10;
t119 = Prmk_1.*d1_1;
t120 = PrmB_1.*d1dot_1;
t121 = r2_1.*t108;
t122 = r2_1.*t40.*t109.*t112;
t123 = t121+t122;
t124 = 1.0./t80;
t125 = Prmm1.*(1.0./2.0);
t126 = Prmm2+t125;
t127 = Prmg.*t19.*t124.*t126;
t866 = t10.*t123;
t128 = F1+t119+t120+t127-t866;
t129 = r2_1.^2;
t130 = t5.^2;
t131 = Prml2.^2;
t132 = xb1-xt1;
t133 = yb1-yt1;
t134 = t132.^2;
t135 = t133.^2;
t136 = zb1-zt1;
t137 = t134+t135;
t138 = t136.^2;
t139 = t134+t135+t138;
t140 = 1.0./t139;
t141 = 1.0./sqrt(t137);
t142 = sqrt(t137);
t149 = xt1.*2.0;
t143 = t95-t149;
t144 = t136.*t140.*t141.*t143.*xt_dot1.*(1.0./2.0);
t150 = yt1.*2.0;
t145 = t102-t150;
t146 = t136.*t140.*t141.*t145.*yt_dot1.*(1.0./2.0);
t148 = t140.*t142.*zt_dot1;
t147 = t144+t146-t148;
t151 = r3_1.^2;
t152 = Prmm1.*t131;
t153 = Prmm1.*t129;
t154 = Prmm2.*t129.*4.0;
t155 = Prmm3.*t151.*4.0;
t876 = Prml2.*Prmm1.*r2_1.*2.0;
t156 = t152+t153+t154+t155-t876;
t157 = 1.0./t156;
t158 = yb1.*1i;
t159 = yt1.*-1i;
t160 = t158+t159+xb1-xt1;
t161 = abs(t160);
t162 = 1.0./t161;
t163 = zt1.*1i;
t164 = t75+t142+t163;
t165 = abs(t164);
t166 = 1.0./t165;
t167 = 1.0./t165.^2;
t168 = cos(t_theta2);
t169 = sin(t_phi2);
t170 = t168.*t169.*t_phi_dot2;
t171 = cos(t_phi2);
t172 = sin(t_theta2);
t173 = t171.*t172.*t_theta_dot2;
t174 = t170+t173;
t175 = Prmlc+d1_2+d2_2;
t179 = TP1_2.*t22;
t180 = TP2_2.*t26;
t181 = TP3_2.*t15.*t17;
t176 = t179-t180+t181+x-xb2;
t183 = TP3_2.*t14;
t184 = TP2_2.*t13.*t17;
t185 = TP1_2.*t12.*t17;
t177 = -t183+t184+t185+y-yb2;
t188 = TP1_2.*t39;
t189 = TP2_2.*t36;
t190 = TP3_2.*t11.*t17;
t178 = -t188+t189+t190+z-zb2;
t182 = t176.^2;
t186 = t177.^2;
t187 = t182+t186;
t191 = t178.^2;
t192 = t182+t186+t191;
t193 = 1.0./t192;
t194 = 1.0./sqrt(t187);
t196 = sqrt(t187);
t197 = -t188+t189+t190;
t198 = beta_dot.*t197;
t199 = TP3_2.*t14.*t15;
t200 = TP2_2.*t13.*t15.*t17;
t201 = TP1_2.*t12.*t15.*t17;
t202 = -t199+t200+t201;
t203 = alpha_dot.*t202;
t204 = TP1_2.*t26;
t205 = TP2_2.*t22;
t206 = t204+t205;
t207 = gamma_dot.*t206;
t208 = t198+t203-t207+x_dot;
t210 = TP3_2.*t17;
t211 = TP2_2.*t13.*t14;
t212 = TP1_2.*t12.*t14;
t213 = t210+t211+t212;
t214 = alpha_dot.*t213;
t215 = TP1_2.*t13.*t17;
t216 = TP2_2.*t12.*t17;
t217 = t215-t216;
t218 = gamma_dot.*t217;
t219 = -t214+t218+y_dot;
t228 = t179-t180+t181;
t229 = beta_dot.*t228;
t230 = TP3_2.*t11.*t14;
t231 = TP2_2.*t11.*t13.*t17;
t232 = TP1_2.*t11.*t12.*t17;
t233 = -t230+t231+t232;
t234 = alpha_dot.*t233;
t235 = TP1_2.*t36;
t236 = TP2_2.*t39;
t237 = t235+t236;
t238 = gamma_dot.*t237;
t239 = -t229+t234+t238+z_dot;
t240 = t193.*t196.*t239;
t241 = xb2.*2.0;
t242 = TP1_2.*t22.*2.0;
t243 = TP2_2.*t26.*2.0;
t244 = TP3_2.*t15.*t17.*2.0;
t245 = t94-t241+t242-t243+t244;
t246 = t178.*t193.*t194.*t208.*t245.*(1.0./2.0);
t247 = yb2.*2.0;
t248 = TP3_2.*t14.*2.0;
t249 = TP2_2.*t13.*t17.*2.0;
t250 = TP1_2.*t12.*t17.*2.0;
t251 = t101-t247-t248+t249+t250;
t252 = t178.*t193.*t194.*t219.*t251.*(1.0./2.0);
t195 = -t240+t246+t252;
t209 = 1.0./t187;
t255 = t177.*t208.*t209;
t256 = t176.*t209.*t219;
t220 = t255-t256;
t221 = zb2.*-1i;
t222 = TP1_2.*t39.*-1i;
t223 = TP2_2.*t36.*1i;
t224 = TP3_2.*t11.*t17.*1i;
t225 = t74+t196+t221+t222+t223+t224;
t226 = abs(t225);
t227 = 1.0./Prmm3;
t253 = t195.^2;
t254 = 1.0./t226.^2;
t257 = t220.^2;
t258 = L_vector1_2.^2;
t259 = L_vector2_2.^2;
t260 = L_vector3_2.^2;
t261 = t258+t259+t260;
t262 = 1.0./sqrt(t261);
t263 = Prmk_2.*d1_2;
t264 = PrmB_2.*d1dot_2;
t265 = r2_2.*t253;
t266 = r2_2.*t187.*t254.*t257;
t267 = t265+t266;
t268 = 1.0./t226;
t269 = Prmg.*t126.*t178.*t268;
t901 = t10.*t267;
t270 = F2+t263+t264+t269-t901;
t271 = r2_2.^2;
t272 = t171.^2;
t273 = xb2-xt2;
t274 = yb2-yt2;
t275 = t273.^2;
t276 = t274.^2;
t277 = zb2-zt2;
t278 = t275+t276;
t279 = t277.^2;
t280 = t275+t276+t279;
t281 = 1.0./t280;
t282 = 1.0./sqrt(t278);
t283 = sqrt(t278);
t290 = xt2.*2.0;
t284 = t241-t290;
t285 = t277.*t281.*t282.*t284.*xt_dot2.*(1.0./2.0);
t291 = yt2.*2.0;
t286 = t247-t291;
t287 = t277.*t281.*t282.*t286.*yt_dot2.*(1.0./2.0);
t289 = t281.*t283.*zt_dot2;
t288 = t285+t287-t289;
t292 = r3_2.^2;
t293 = Prmm1.*t271;
t294 = Prmm2.*t271.*4.0;
t295 = Prmm3.*t292.*4.0;
t911 = Prml2.*Prmm1.*r2_2.*2.0;
t296 = t152+t293+t294+t295-t911;
t297 = 1.0./t296;
t298 = yb2.*1i;
t299 = yt2.*-1i;
t300 = t298+t299+xb2-xt2;
t301 = abs(t300);
t302 = 1.0./t301;
t303 = zt2.*1i;
t304 = t221+t283+t303;
t305 = abs(t304);
t306 = 1.0./t305;
t307 = 1.0./t305.^2;
t308 = cos(t_theta3);
t309 = sin(t_phi3);
t310 = t308.*t309.*t_phi_dot3;
t311 = cos(t_phi3);
t312 = sin(t_theta3);
t313 = t311.*t312.*t_theta_dot3;
t314 = t310+t313;
t315 = Prmlc+d1_3+d2_3;
t319 = TP1_3.*t22;
t320 = TP2_3.*t26;
t321 = TP3_3.*t15.*t17;
t316 = t319-t320+t321+x-xb3;
t323 = TP3_3.*t14;
t324 = TP2_3.*t13.*t17;
t325 = TP1_3.*t12.*t17;
t317 = -t323+t324+t325+y-yb3;
t328 = TP1_3.*t39;
t329 = TP2_3.*t36;
t330 = TP3_3.*t11.*t17;
t318 = -t328+t329+t330+z-zb3;
t322 = t316.^2;
t326 = t317.^2;
t327 = t322+t326;
t331 = t318.^2;
t332 = t322+t326+t331;
t333 = 1.0./t332;
t334 = 1.0./sqrt(t327);
t336 = sqrt(t327);
t337 = -t328+t329+t330;
t338 = beta_dot.*t337;
t339 = TP3_3.*t14.*t15;
t340 = TP2_3.*t13.*t15.*t17;
t341 = TP1_3.*t12.*t15.*t17;
t342 = -t339+t340+t341;
t343 = alpha_dot.*t342;
t344 = TP1_3.*t26;
t345 = TP2_3.*t22;
t346 = t344+t345;
t347 = gamma_dot.*t346;
t348 = t338+t343-t347+x_dot;
t350 = TP3_3.*t17;
t351 = TP2_3.*t13.*t14;
t352 = TP1_3.*t12.*t14;
t353 = t350+t351+t352;
t354 = alpha_dot.*t353;
t355 = TP1_3.*t13.*t17;
t356 = TP2_3.*t12.*t17;
t357 = t355-t356;
t358 = gamma_dot.*t357;
t359 = -t354+t358+y_dot;
t367 = t319-t320+t321;
t368 = beta_dot.*t367;
t369 = TP3_3.*t11.*t14;
t370 = TP2_3.*t11.*t13.*t17;
t371 = TP1_3.*t11.*t12.*t17;
t372 = -t369+t370+t371;
t373 = alpha_dot.*t372;
t374 = TP1_3.*t36;
t375 = TP2_3.*t39;
t376 = t374+t375;
t377 = gamma_dot.*t376;
t378 = -t368+t373+t377+z_dot;
t379 = t333.*t336.*t378;
t380 = xb3.*2.0;
t381 = TP1_3.*t22.*2.0;
t382 = TP2_3.*t26.*2.0;
t383 = TP3_3.*t15.*t17.*2.0;
t384 = t94-t380+t381-t382+t383;
t385 = t318.*t333.*t334.*t348.*t384.*(1.0./2.0);
t386 = yb3.*2.0;
t387 = TP3_3.*t14.*2.0;
t388 = TP2_3.*t13.*t17.*2.0;
t389 = TP1_3.*t12.*t17.*2.0;
t390 = t101-t386-t387+t388+t389;
t391 = t318.*t333.*t334.*t359.*t390.*(1.0./2.0);
t335 = -t379+t385+t391;
t349 = 1.0./t327;
t394 = t317.*t348.*t349;
t395 = t316.*t349.*t359;
t360 = t394-t395;
t361 = zb3.*-1i;
t362 = TP1_3.*t39.*-1i;
t363 = TP2_3.*t36.*1i;
t364 = TP3_3.*t11.*t17.*1i;
t365 = t74+t336+t361+t362+t363+t364;
t366 = abs(t365);
t392 = t335.^2;
t393 = 1.0./t366.^2;
t396 = t360.^2;
t397 = L_vector1_3.^2;
t398 = L_vector2_3.^2;
t399 = L_vector3_3.^2;
t400 = t397+t398+t399;
t401 = 1.0./sqrt(t400);
t402 = Prmk_3.*d1_3;
t403 = PrmB_3.*d1dot_3;
t404 = r2_3.*t392;
t405 = r2_3.*t327.*t393.*t396;
t406 = t404+t405;
t407 = 1.0./t366;
t408 = Prmg.*t126.*t318.*t407;
t936 = t10.*t406;
t409 = F3+t402+t403+t408-t936;
t410 = r2_3.^2;
t411 = t311.^2;
t412 = xb3-xt3;
t413 = yb3-yt3;
t414 = t412.^2;
t415 = t413.^2;
t416 = zb3-zt3;
t417 = t414+t415;
t418 = t416.^2;
t419 = t414+t415+t418;
t420 = 1.0./t419;
t421 = 1.0./sqrt(t417);
t422 = sqrt(t417);
t429 = xt3.*2.0;
t423 = t380-t429;
t424 = t416.*t420.*t421.*t423.*xt_dot3.*(1.0./2.0);
t430 = yt3.*2.0;
t425 = t386-t430;
t426 = t416.*t420.*t421.*t425.*yt_dot3.*(1.0./2.0);
t428 = t420.*t422.*zt_dot3;
t427 = t424+t426-t428;
t431 = r3_3.^2;
t432 = Prmm1.*t410;
t433 = Prmm2.*t410.*4.0;
t434 = Prmm3.*t431.*4.0;
t946 = Prml2.*Prmm1.*r2_3.*2.0;
t435 = t152+t432+t433+t434-t946;
t436 = 1.0./t435;
t437 = yb3.*1i;
t438 = yt3.*-1i;
t439 = t437+t438+xb3-xt3;
t440 = abs(t439);
t441 = 1.0./t440;
t442 = zt3.*1i;
t443 = t361+t422+t442;
t444 = abs(t443);
t445 = 1.0./t444;
t446 = 1.0./t444.^2;
t447 = cos(t_theta4);
t448 = sin(t_phi4);
t449 = t447.*t448.*t_phi_dot4;
t450 = cos(t_phi4);
t451 = sin(t_theta4);
t452 = t450.*t451.*t_theta_dot4;
t453 = t449+t452;
t454 = Prmlc+d1_4+d2_4;
t458 = TP1_4.*t22;
t459 = TP2_4.*t26;
t460 = TP3_4.*t15.*t17;
t455 = t458-t459+t460+x-xb4;
t462 = TP3_4.*t14;
t463 = TP2_4.*t13.*t17;
t464 = TP1_4.*t12.*t17;
t456 = -t462+t463+t464+y-yb4;
t467 = TP1_4.*t39;
t468 = TP2_4.*t36;
t469 = TP3_4.*t11.*t17;
t457 = -t467+t468+t469+z-zb4;
t461 = t455.^2;
t465 = t456.^2;
t466 = t461+t465;
t470 = t457.^2;
t471 = t461+t465+t470;
t472 = 1.0./t471;
t473 = 1.0./sqrt(t466);
t475 = sqrt(t466);
t476 = -t467+t468+t469;
t477 = beta_dot.*t476;
t478 = TP3_4.*t14.*t15;
t479 = TP2_4.*t13.*t15.*t17;
t480 = TP1_4.*t12.*t15.*t17;
t481 = -t478+t479+t480;
t482 = alpha_dot.*t481;
t483 = TP1_4.*t26;
t484 = TP2_4.*t22;
t485 = t483+t484;
t486 = gamma_dot.*t485;
t487 = t477+t482-t486+x_dot;
t489 = TP3_4.*t17;
t490 = TP2_4.*t13.*t14;
t491 = TP1_4.*t12.*t14;
t492 = t489+t490+t491;
t493 = alpha_dot.*t492;
t494 = TP1_4.*t13.*t17;
t495 = TP2_4.*t12.*t17;
t496 = t494-t495;
t497 = gamma_dot.*t496;
t498 = -t493+t497+y_dot;
t506 = t458-t459+t460;
t507 = beta_dot.*t506;
t508 = TP3_4.*t11.*t14;
t509 = TP2_4.*t11.*t13.*t17;
t510 = TP1_4.*t11.*t12.*t17;
t511 = -t508+t509+t510;
t512 = alpha_dot.*t511;
t513 = TP1_4.*t36;
t514 = TP2_4.*t39;
t515 = t513+t514;
t516 = gamma_dot.*t515;
t517 = -t507+t512+t516+z_dot;
t518 = t472.*t475.*t517;
t519 = xb4.*2.0;
t520 = TP1_4.*t22.*2.0;
t521 = TP2_4.*t26.*2.0;
t522 = TP3_4.*t15.*t17.*2.0;
t523 = t94-t519+t520-t521+t522;
t524 = t457.*t472.*t473.*t487.*t523.*(1.0./2.0);
t525 = yb4.*2.0;
t526 = TP3_4.*t14.*2.0;
t527 = TP2_4.*t13.*t17.*2.0;
t528 = TP1_4.*t12.*t17.*2.0;
t529 = t101-t525-t526+t527+t528;
t530 = t457.*t472.*t473.*t498.*t529.*(1.0./2.0);
t474 = -t518+t524+t530;
t488 = 1.0./t466;
t533 = t456.*t487.*t488;
t534 = t455.*t488.*t498;
t499 = t533-t534;
t500 = zb4.*-1i;
t501 = TP1_4.*t39.*-1i;
t502 = TP2_4.*t36.*1i;
t503 = TP3_4.*t11.*t17.*1i;
t504 = t74+t475+t500+t501+t502+t503;
t505 = abs(t504);
t531 = t474.^2;
t532 = 1.0./t505.^2;
t535 = t499.^2;
t536 = L_vector1_4.^2;
t537 = L_vector2_4.^2;
t538 = L_vector3_4.^2;
t539 = t536+t537+t538;
t540 = 1.0./sqrt(t539);
t541 = Prmk_4.*d1_4;
t542 = PrmB_4.*d1dot_4;
t543 = r2_4.*t531;
t544 = r2_4.*t466.*t532.*t535;
t545 = t543+t544;
t546 = 1.0./t505;
t547 = Prmg.*t126.*t457.*t546;
t971 = t10.*t545;
t548 = F4+t541+t542+t547-t971;
t549 = r2_4.^2;
t550 = t450.^2;
t551 = xb4-xt4;
t552 = yb4-yt4;
t553 = t551.^2;
t554 = t552.^2;
t555 = zb4-zt4;
t556 = t553+t554;
t557 = t555.^2;
t558 = t553+t554+t557;
t559 = 1.0./t558;
t560 = 1.0./sqrt(t556);
t561 = sqrt(t556);
t568 = xt4.*2.0;
t562 = t519-t568;
t563 = t555.*t559.*t560.*t562.*xt_dot4.*(1.0./2.0);
t569 = yt4.*2.0;
t564 = t525-t569;
t565 = t555.*t559.*t560.*t564.*yt_dot4.*(1.0./2.0);
t567 = t559.*t561.*zt_dot4;
t566 = t563+t565-t567;
t570 = r3_4.^2;
t571 = Prmm1.*t549;
t572 = Prmm2.*t549.*4.0;
t573 = Prmm3.*t570.*4.0;
t981 = Prml2.*Prmm1.*r2_4.*2.0;
t574 = t152+t571+t572+t573-t981;
t575 = 1.0./t574;
t576 = yb4.*1i;
t577 = yt4.*-1i;
t578 = t576+t577+xb4-xt4;
t579 = abs(t578);
t580 = 1.0./t579;
t581 = zt4.*1i;
t582 = t500+t561+t581;
t583 = abs(t582);
t584 = 1.0./t583;
t585 = 1.0./t583.^2;
t586 = cos(t_theta5);
t587 = sin(t_phi5);
t588 = t586.*t587.*t_phi_dot5;
t589 = cos(t_phi5);
t590 = sin(t_theta5);
t591 = t589.*t590.*t_theta_dot5;
t592 = t588+t591;
t593 = Prmlc+d1_5+d2_5;
t597 = TP1_5.*t22;
t598 = TP2_5.*t26;
t599 = TP3_5.*t15.*t17;
t594 = t597-t598+t599+x-xb5;
t601 = TP3_5.*t14;
t602 = TP2_5.*t13.*t17;
t603 = TP1_5.*t12.*t17;
t595 = -t601+t602+t603+y-yb5;
t606 = TP1_5.*t39;
t607 = TP2_5.*t36;
t608 = TP3_5.*t11.*t17;
t596 = -t606+t607+t608+z-zb5;
t600 = t594.^2;
t604 = t595.^2;
t605 = t600+t604;
t609 = t596.^2;
t610 = t600+t604+t609;
t611 = 1.0./t610;
t612 = 1.0./sqrt(t605);
t614 = sqrt(t605);
t615 = -t606+t607+t608;
t616 = beta_dot.*t615;
t617 = TP3_5.*t14.*t15;
t618 = TP2_5.*t13.*t15.*t17;
t619 = TP1_5.*t12.*t15.*t17;
t620 = -t617+t618+t619;
t621 = alpha_dot.*t620;
t622 = TP1_5.*t26;
t623 = TP2_5.*t22;
t624 = t622+t623;
t625 = gamma_dot.*t624;
t626 = t616+t621-t625+x_dot;
t628 = TP3_5.*t17;
t629 = TP2_5.*t13.*t14;
t630 = TP1_5.*t12.*t14;
t631 = t628+t629+t630;
t632 = alpha_dot.*t631;
t633 = TP1_5.*t13.*t17;
t634 = TP2_5.*t12.*t17;
t635 = t633-t634;
t636 = gamma_dot.*t635;
t637 = -t632+t636+y_dot;
t645 = t597-t598+t599;
t646 = beta_dot.*t645;
t647 = TP3_5.*t11.*t14;
t648 = TP2_5.*t11.*t13.*t17;
t649 = TP1_5.*t11.*t12.*t17;
t650 = -t647+t648+t649;
t651 = alpha_dot.*t650;
t652 = TP1_5.*t36;
t653 = TP2_5.*t39;
t654 = t652+t653;
t655 = gamma_dot.*t654;
t656 = -t646+t651+t655+z_dot;
t657 = t611.*t614.*t656;
t658 = xb5.*2.0;
t659 = TP1_5.*t22.*2.0;
t660 = TP2_5.*t26.*2.0;
t661 = TP3_5.*t15.*t17.*2.0;
t662 = t94-t658+t659-t660+t661;
t663 = t596.*t611.*t612.*t626.*t662.*(1.0./2.0);
t664 = yb5.*2.0;
t665 = TP3_5.*t14.*2.0;
t666 = TP2_5.*t13.*t17.*2.0;
t667 = TP1_5.*t12.*t17.*2.0;
t668 = t101-t664-t665+t666+t667;
t669 = t596.*t611.*t612.*t637.*t668.*(1.0./2.0);
t613 = -t657+t663+t669;
t627 = 1.0./t605;
t672 = t595.*t626.*t627;
t673 = t594.*t627.*t637;
t638 = t672-t673;
t639 = zb5.*-1i;
t640 = TP1_5.*t39.*-1i;
t641 = TP2_5.*t36.*1i;
t642 = TP3_5.*t11.*t17.*1i;
t643 = t74+t614+t639+t640+t641+t642;
t644 = abs(t643);
t670 = t613.^2;
t671 = 1.0./t644.^2;
t674 = t638.^2;
t675 = L_vector1_5.^2;
t676 = L_vector2_5.^2;
t677 = L_vector3_5.^2;
t678 = t675+t676+t677;
t679 = 1.0./sqrt(t678);
t680 = Prmk_5.*d1_5;
t681 = PrmB_5.*d1dot_5;
t682 = r2_5.*t670;
t683 = r2_5.*t605.*t671.*t674;
t684 = t682+t683;
t685 = 1.0./t644;
t686 = Prmg.*t126.*t596.*t685;
t1006 = t10.*t684;
t687 = F5+t680+t681+t686-t1006;
t688 = r2_5.^2;
t689 = t589.^2;
t690 = xb5-xt5;
t691 = yb5-yt5;
t692 = t690.^2;
t693 = t691.^2;
t694 = zb5-zt5;
t695 = t692+t693;
t696 = t694.^2;
t697 = t692+t693+t696;
t698 = 1.0./t697;
t699 = 1.0./sqrt(t695);
t700 = sqrt(t695);
t707 = xt5.*2.0;
t701 = t658-t707;
t702 = t694.*t698.*t699.*t701.*xt_dot5.*(1.0./2.0);
t708 = yt5.*2.0;
t703 = t664-t708;
t704 = t694.*t698.*t699.*t703.*yt_dot5.*(1.0./2.0);
t706 = t698.*t700.*zt_dot5;
t705 = t702+t704-t706;
t709 = r3_5.^2;
t710 = Prmm1.*t688;
t711 = Prmm2.*t688.*4.0;
t712 = Prmm3.*t709.*4.0;
t1016 = Prml2.*Prmm1.*r2_5.*2.0;
t713 = t152+t710+t711+t712-t1016;
t714 = 1.0./t713;
t715 = yb5.*1i;
t716 = yt5.*-1i;
t717 = t715+t716+xb5-xt5;
t718 = abs(t717);
t719 = 1.0./t718;
t720 = zt5.*1i;
t721 = t639+t700+t720;
t722 = abs(t721);
t723 = 1.0./t722;
t724 = 1.0./t722.^2;
t725 = cos(t_theta6);
t726 = sin(t_phi6);
t727 = t725.*t726.*t_phi_dot6;
t728 = cos(t_phi6);
t729 = sin(t_theta6);
t730 = t728.*t729.*t_theta_dot6;
t731 = t727+t730;
t732 = Prmlc+d1_6+d2_6;
t736 = TP1_6.*t22;
t737 = TP2_6.*t26;
t738 = TP3_6.*t15.*t17;
t733 = t736-t737+t738+x-xb6;
t740 = TP3_6.*t14;
t741 = TP2_6.*t13.*t17;
t742 = TP1_6.*t12.*t17;
t734 = -t740+t741+t742+y-yb6;
t745 = TP1_6.*t39;
t746 = TP2_6.*t36;
t747 = TP3_6.*t11.*t17;
t735 = -t745+t746+t747+z-zb6;
t739 = t733.^2;
t743 = t734.^2;
t744 = t739+t743;
t748 = t735.^2;
t749 = t739+t743+t748;
t750 = 1.0./t749;
t751 = 1.0./sqrt(t744);
t753 = sqrt(t744);
t754 = -t745+t746+t747;
t755 = beta_dot.*t754;
t756 = TP3_6.*t14.*t15;
t757 = TP2_6.*t13.*t15.*t17;
t758 = TP1_6.*t12.*t15.*t17;
t759 = -t756+t757+t758;
t760 = alpha_dot.*t759;
t761 = TP1_6.*t26;
t762 = TP2_6.*t22;
t763 = t761+t762;
t764 = gamma_dot.*t763;
t765 = t755+t760-t764+x_dot;
t767 = TP3_6.*t17;
t768 = TP2_6.*t13.*t14;
t769 = TP1_6.*t12.*t14;
t770 = t767+t768+t769;
t771 = alpha_dot.*t770;
t772 = TP1_6.*t13.*t17;
t773 = TP2_6.*t12.*t17;
t774 = t772-t773;
t775 = gamma_dot.*t774;
t776 = -t771+t775+y_dot;
t784 = t736-t737+t738;
t785 = beta_dot.*t784;
t786 = TP3_6.*t11.*t14;
t787 = TP2_6.*t11.*t13.*t17;
t788 = TP1_6.*t11.*t12.*t17;
t789 = -t786+t787+t788;
t790 = alpha_dot.*t789;
t791 = TP1_6.*t36;
t792 = TP2_6.*t39;
t793 = t791+t792;
t794 = gamma_dot.*t793;
t795 = -t785+t790+t794+z_dot;
t796 = t750.*t753.*t795;
t797 = xb6.*2.0;
t798 = TP1_6.*t22.*2.0;
t799 = TP2_6.*t26.*2.0;
t800 = TP3_6.*t15.*t17.*2.0;
t801 = t94-t797+t798-t799+t800;
t802 = t735.*t750.*t751.*t765.*t801.*(1.0./2.0);
t803 = yb6.*2.0;
t804 = TP3_6.*t14.*2.0;
t805 = TP2_6.*t13.*t17.*2.0;
t806 = TP1_6.*t12.*t17.*2.0;
t807 = t101-t803-t804+t805+t806;
t808 = t735.*t750.*t751.*t776.*t807.*(1.0./2.0);
t752 = -t796+t802+t808;
t766 = 1.0./t744;
t811 = t734.*t765.*t766;
t812 = t733.*t766.*t776;
t777 = t811-t812;
t778 = zb6.*-1i;
t779 = TP1_6.*t39.*-1i;
t780 = TP2_6.*t36.*1i;
t781 = TP3_6.*t11.*t17.*1i;
t782 = t74+t753+t778+t779+t780+t781;
t783 = abs(t782);
t809 = t752.^2;
t810 = 1.0./t783.^2;
t813 = t777.^2;
t814 = L_vector1_6.^2;
t815 = L_vector2_6.^2;
t816 = L_vector3_6.^2;
t817 = t814+t815+t816;
t818 = 1.0./sqrt(t817);
t819 = Prmk_6.*d1_6;
t820 = PrmB_6.*d1dot_6;
t821 = r2_6.*t809;
t822 = r2_6.*t744.*t810.*t813;
t823 = t821+t822;
t824 = 1.0./t783;
t825 = Prmg.*t126.*t735.*t824;
t1041 = t10.*t823;
t826 = F6+t819+t820+t825-t1041;
t827 = r2_6.^2;
t828 = t728.^2;
t829 = xb6-xt6;
t830 = yb6-yt6;
t831 = t829.^2;
t832 = t830.^2;
t833 = zb6-zt6;
t834 = t831+t832;
t835 = t833.^2;
t836 = t831+t832+t835;
t837 = 1.0./t836;
t838 = 1.0./sqrt(t834);
t839 = sqrt(t834);
t846 = xt6.*2.0;
t840 = t797-t846;
t841 = t833.*t837.*t838.*t840.*xt_dot6.*(1.0./2.0);
t847 = yt6.*2.0;
t842 = t803-t847;
t843 = t833.*t837.*t838.*t842.*yt_dot6.*(1.0./2.0);
t845 = t837.*t839.*zt_dot6;
t844 = t841+t843-t845;
t848 = r3_6.^2;
t849 = Prmm1.*t827;
t850 = Prmm2.*t827.*4.0;
t851 = Prmm3.*t848.*4.0;
t1051 = Prml2.*Prmm1.*r2_6.*2.0;
t852 = t152+t849+t850+t851-t1051;
t853 = 1.0./t852;
t854 = yb6.*1i;
t855 = yt6.*-1i;
t856 = t854+t855+xb6-xt6;
t857 = abs(t856);
t858 = 1.0./t857;
t859 = zt6.*1i;
t860 = t778+t839+t859;
t861 = abs(t860);
t862 = 1.0./t861;
t863 = 1.0./t861.^2;
t864 = t3.*t6.*t_phi_dot1;
t865 = t864-t2.*t5.*t_theta_dot1;
t867 = t40.*t109.*t112;
t868 = t108+t867;
t869 = Prmm3.*r3_1.*t868;
t870 = Fr1_1.*L_vector1_1.*t117;
t871 = Fr2_1.*L_vector2_1.*t117;
t872 = Fr3_1.*L_vector3_1.*t117;
t873 = Prmm3.*t118.*t128;
t1074 = Prmg.*Prmm3.*t19.*t124;
t874 = F1+t869+t870+t871+t872+t873-t1074;
t875 = 1.0./t5;
t877 = Fr2_1.*L_vector1_1.*2.0;
t878 = Prmm1.*d1dot_1.*r2_1.*t73.*t130.*2.0;
t879 = Prmm2.*d1dot_1.*r2_1.*t73.*t130.*4.0;
t880 = Prmm3.*d1dot_1.*r3_1.*t73.*t130.*4.0;
t881 = Prmm3.*d2dot_1.*r3_1.*t73.*t130.*4.0;
t882 = Prmm1.*t3.*t5.*t73.*t131.*t147;
t883 = Prmm1.*t3.*t5.*t73.*t129.*t147;
t884 = Prmm2.*t3.*t5.*t73.*t129.*t147.*4.0;
t885 = Prmm3.*t3.*t5.*t73.*t147.*t151.*4.0;
t886 = t877+t878+t879+t880+t881+t882+t883+t884+t885-Fr1_1.*L_vector2_1.*2.0-Prml2.*Prmm1.*d1dot_1.*t73.*t130.*2.0-Prml2.*Prmm1.*r2_1.*t3.*t5.*t73.*t147.*2.0;
t887 = Prmm1.*d1dot_1.*r2_1.*t147.*2.0;
t888 = Prmm2.*d1dot_1.*r2_1.*t147.*8.0;
t889 = Prmm3.*d1dot_1.*r3_1.*t147.*8.0;
t890 = Prmm3.*d2dot_1.*r3_1.*t147.*8.0;
t891 = Fr1_1.*L_vector3_1.*t132.*t162.*4.0;
t892 = Fr2_1.*L_vector3_1.*t133.*t162.*4.0;
t893 = Prmg.*Prml2.*Prmm1.*t142.*t166.*2.0;
t894 = Prmm1.*t112.*t131.*t136.*t142.*t167;
t895 = Prmm1.*t112.*t129.*t136.*t142.*t167;
t896 = Prmm2.*t112.*t129.*t136.*t142.*t167.*4.0;
t897 = Prmm3.*t112.*t136.*t142.*t151.*t167.*4.0;
t1075 = Prml2.*Prmm1.*d1dot_1.*t147.*2.0;
t1076 = Fr3_1.*L_vector1_1.*t132.*t162.*4.0;
t1077 = Fr3_1.*L_vector2_1.*t133.*t162.*4.0;
t1078 = Prmg.*Prmm1.*r2_1.*t142.*t166.*2.0;
t1079 = Prmg.*Prmm2.*r2_1.*t142.*t166.*4.0;
t1080 = Prmg.*Prmm3.*r3_1.*t142.*t166.*4.0;
t1081 = Prml2.*Prmm1.*r2_1.*t112.*t136.*t142.*t167.*2.0;
t898 = t887+t888+t889+t890+t891+t892+t893+t894+t895+t896+t897-t1075-t1076-t1077-t1078-t1079-t1080-t1081;
t899 = t169.*t172.*t_phi_dot2;
t900 = t899-t168.*t171.*t_theta_dot2;
t902 = t187.*t254.*t257;
t903 = t253+t902;
t904 = Prmm3.*r3_2.*t903;
t905 = Fr1_2.*L_vector1_2.*t262;
t906 = Fr2_2.*L_vector2_2.*t262;
t907 = Fr3_2.*L_vector3_2.*t262;
t908 = Prmm3.*t118.*t270;
t1082 = Prmg.*Prmm3.*t178.*t268;
t909 = F2+t904+t905+t906+t907+t908-t1082;
t910 = 1.0./t171;
t912 = Fr2_2.*L_vector1_2.*2.0;
t913 = Prmm1.*d1dot_2.*r2_2.*t220.*t272.*2.0;
t914 = Prmm2.*d1dot_2.*r2_2.*t220.*t272.*4.0;
t915 = Prmm3.*d1dot_2.*r3_2.*t220.*t272.*4.0;
t916 = Prmm3.*d2dot_2.*r3_2.*t220.*t272.*4.0;
t917 = Prmm1.*t131.*t169.*t171.*t220.*t288;
t918 = Prmm1.*t169.*t171.*t220.*t271.*t288;
t919 = Prmm2.*t169.*t171.*t220.*t271.*t288.*4.0;
t920 = Prmm3.*t169.*t171.*t220.*t288.*t292.*4.0;
t921 = t912+t913+t914+t915+t916+t917+t918+t919+t920-Fr1_2.*L_vector2_2.*2.0-Prml2.*Prmm1.*d1dot_2.*t220.*t272.*2.0-Prml2.*Prmm1.*r2_2.*t169.*t171.*t220.*t288.*2.0;
t922 = Prmm1.*d1dot_2.*r2_2.*t288.*2.0;
t923 = Prmm2.*d1dot_2.*r2_2.*t288.*8.0;
t924 = Prmm3.*d1dot_2.*r3_2.*t288.*8.0;
t925 = Prmm3.*d2dot_2.*r3_2.*t288.*8.0;
t926 = Fr1_2.*L_vector3_2.*t273.*t302.*4.0;
t927 = Fr2_2.*L_vector3_2.*t274.*t302.*4.0;
t928 = Prmg.*Prml2.*Prmm1.*t283.*t306.*2.0;
t929 = Prmm1.*t131.*t257.*t277.*t283.*t307;
t930 = Prmm1.*t257.*t271.*t277.*t283.*t307;
t931 = Prmm2.*t257.*t271.*t277.*t283.*t307.*4.0;
t932 = Prmm3.*t257.*t277.*t283.*t292.*t307.*4.0;
t1083 = Prml2.*Prmm1.*d1dot_2.*t288.*2.0;
t1084 = Fr3_2.*L_vector1_2.*t273.*t302.*4.0;
t1085 = Fr3_2.*L_vector2_2.*t274.*t302.*4.0;
t1086 = Prmg.*Prmm1.*r2_2.*t283.*t306.*2.0;
t1087 = Prmg.*Prmm2.*r2_2.*t283.*t306.*4.0;
t1088 = Prmg.*Prmm3.*r3_2.*t283.*t306.*4.0;
t1089 = Prml2.*Prmm1.*r2_2.*t257.*t277.*t283.*t307.*2.0;
t933 = t922+t923+t924+t925+t926+t927+t928+t929+t930+t931+t932-t1083-t1084-t1085-t1086-t1087-t1088-t1089;
t934 = t309.*t312.*t_phi_dot3;
t935 = t934-t308.*t311.*t_theta_dot3;
t937 = t327.*t393.*t396;
t938 = t392+t937;
t939 = Prmm3.*r3_3.*t938;
t940 = Fr1_3.*L_vector1_3.*t401;
t941 = Fr2_3.*L_vector2_3.*t401;
t942 = Fr3_3.*L_vector3_3.*t401;
t943 = Prmm3.*t118.*t409;
t1090 = Prmg.*Prmm3.*t318.*t407;
t944 = F3+t939+t940+t941+t942+t943-t1090;
t945 = 1.0./t311;
t947 = Fr2_3.*L_vector1_3.*2.0;
t948 = Prmm1.*d1dot_3.*r2_3.*t360.*t411.*2.0;
t949 = Prmm2.*d1dot_3.*r2_3.*t360.*t411.*4.0;
t950 = Prmm3.*d1dot_3.*r3_3.*t360.*t411.*4.0;
t951 = Prmm3.*d2dot_3.*r3_3.*t360.*t411.*4.0;
t952 = Prmm1.*t131.*t309.*t311.*t360.*t427;
t953 = Prmm1.*t309.*t311.*t360.*t410.*t427;
t954 = Prmm2.*t309.*t311.*t360.*t410.*t427.*4.0;
t955 = Prmm3.*t309.*t311.*t360.*t427.*t431.*4.0;
t956 = t947+t948+t949+t950+t951+t952+t953+t954+t955-Fr1_3.*L_vector2_3.*2.0-Prml2.*Prmm1.*d1dot_3.*t360.*t411.*2.0-Prml2.*Prmm1.*r2_3.*t309.*t311.*t360.*t427.*2.0;
t957 = Prmm1.*d1dot_3.*r2_3.*t427.*2.0;
t958 = Prmm2.*d1dot_3.*r2_3.*t427.*8.0;
t959 = Prmm3.*d1dot_3.*r3_3.*t427.*8.0;
t960 = Prmm3.*d2dot_3.*r3_3.*t427.*8.0;
t961 = Fr1_3.*L_vector3_3.*t412.*t441.*4.0;
t962 = Fr2_3.*L_vector3_3.*t413.*t441.*4.0;
t963 = Prmg.*Prml2.*Prmm1.*t422.*t445.*2.0;
t964 = Prmm1.*t131.*t396.*t416.*t422.*t446;
t965 = Prmm1.*t396.*t410.*t416.*t422.*t446;
t966 = Prmm2.*t396.*t410.*t416.*t422.*t446.*4.0;
t967 = Prmm3.*t396.*t416.*t422.*t431.*t446.*4.0;
t1091 = Prml2.*Prmm1.*d1dot_3.*t427.*2.0;
t1092 = Fr3_3.*L_vector1_3.*t412.*t441.*4.0;
t1093 = Fr3_3.*L_vector2_3.*t413.*t441.*4.0;
t1094 = Prmg.*Prmm1.*r2_3.*t422.*t445.*2.0;
t1095 = Prmg.*Prmm2.*r2_3.*t422.*t445.*4.0;
t1096 = Prmg.*Prmm3.*r3_3.*t422.*t445.*4.0;
t1097 = Prml2.*Prmm1.*r2_3.*t396.*t416.*t422.*t446.*2.0;
t968 = t957+t958+t959+t960+t961+t962+t963+t964+t965+t966+t967-t1091-t1092-t1093-t1094-t1095-t1096-t1097;
t969 = t448.*t451.*t_phi_dot4;
t970 = t969-t447.*t450.*t_theta_dot4;
t972 = t466.*t532.*t535;
t973 = t531+t972;
t974 = Prmm3.*r3_4.*t973;
t975 = Fr1_4.*L_vector1_4.*t540;
t976 = Fr2_4.*L_vector2_4.*t540;
t977 = Fr3_4.*L_vector3_4.*t540;
t978 = Prmm3.*t118.*t548;
t1098 = Prmg.*Prmm3.*t457.*t546;
t979 = F4+t974+t975+t976+t977+t978-t1098;
t980 = 1.0./t450;
t982 = Fr2_4.*L_vector1_4.*2.0;
t983 = Prmm1.*d1dot_4.*r2_4.*t499.*t550.*2.0;
t984 = Prmm2.*d1dot_4.*r2_4.*t499.*t550.*4.0;
t985 = Prmm3.*d1dot_4.*r3_4.*t499.*t550.*4.0;
t986 = Prmm3.*d2dot_4.*r3_4.*t499.*t550.*4.0;
t987 = Prmm1.*t131.*t448.*t450.*t499.*t566;
t988 = Prmm1.*t448.*t450.*t499.*t549.*t566;
t989 = Prmm2.*t448.*t450.*t499.*t549.*t566.*4.0;
t990 = Prmm3.*t448.*t450.*t499.*t566.*t570.*4.0;
t991 = t982+t983+t984+t985+t986+t987+t988+t989+t990-Fr1_4.*L_vector2_4.*2.0-Prml2.*Prmm1.*d1dot_4.*t499.*t550.*2.0-Prml2.*Prmm1.*r2_4.*t448.*t450.*t499.*t566.*2.0;
t992 = Prmm1.*d1dot_4.*r2_4.*t566.*2.0;
t993 = Prmm2.*d1dot_4.*r2_4.*t566.*8.0;
t994 = Prmm3.*d1dot_4.*r3_4.*t566.*8.0;
t995 = Prmm3.*d2dot_4.*r3_4.*t566.*8.0;
t996 = Fr1_4.*L_vector3_4.*t551.*t580.*4.0;
t997 = Fr2_4.*L_vector3_4.*t552.*t580.*4.0;
t998 = Prmg.*Prml2.*Prmm1.*t561.*t584.*2.0;
t999 = Prmm1.*t131.*t535.*t555.*t561.*t585;
t1000 = Prmm1.*t535.*t549.*t555.*t561.*t585;
t1001 = Prmm2.*t535.*t549.*t555.*t561.*t585.*4.0;
t1002 = Prmm3.*t535.*t555.*t561.*t570.*t585.*4.0;
t1099 = Prml2.*Prmm1.*d1dot_4.*t566.*2.0;
t1100 = Fr3_4.*L_vector1_4.*t551.*t580.*4.0;
t1101 = Fr3_4.*L_vector2_4.*t552.*t580.*4.0;
t1102 = Prmg.*Prmm1.*r2_4.*t561.*t584.*2.0;
t1103 = Prmg.*Prmm2.*r2_4.*t561.*t584.*4.0;
t1104 = Prmg.*Prmm3.*r3_4.*t561.*t584.*4.0;
t1105 = Prml2.*Prmm1.*r2_4.*t535.*t555.*t561.*t585.*2.0;
t1003 = t992+t993+t994+t995+t996+t997+t998+t999+t1000+t1001+t1002-t1099-t1100-t1101-t1102-t1103-t1104-t1105;
t1004 = t587.*t590.*t_phi_dot5;
t1005 = t1004-t586.*t589.*t_theta_dot5;
t1007 = t605.*t671.*t674;
t1008 = t670+t1007;
t1009 = Prmm3.*r3_5.*t1008;
t1010 = Fr1_5.*L_vector1_5.*t679;
t1011 = Fr2_5.*L_vector2_5.*t679;
t1012 = Fr3_5.*L_vector3_5.*t679;
t1013 = Prmm3.*t118.*t687;
t1106 = Prmg.*Prmm3.*t596.*t685;
t1014 = F5+t1009+t1010+t1011+t1012+t1013-t1106;
t1015 = 1.0./t589;
t1017 = Fr2_5.*L_vector1_5.*2.0;
t1018 = Prmm1.*d1dot_5.*r2_5.*t638.*t689.*2.0;
t1019 = Prmm2.*d1dot_5.*r2_5.*t638.*t689.*4.0;
t1020 = Prmm3.*d1dot_5.*r3_5.*t638.*t689.*4.0;
t1021 = Prmm3.*d2dot_5.*r3_5.*t638.*t689.*4.0;
t1022 = Prmm1.*t131.*t587.*t589.*t638.*t705;
t1023 = Prmm1.*t587.*t589.*t638.*t688.*t705;
t1024 = Prmm2.*t587.*t589.*t638.*t688.*t705.*4.0;
t1025 = Prmm3.*t587.*t589.*t638.*t705.*t709.*4.0;
t1026 = t1017+t1018+t1019+t1020+t1021+t1022+t1023+t1024+t1025-Fr1_5.*L_vector2_5.*2.0-Prml2.*Prmm1.*d1dot_5.*t638.*t689.*2.0-Prml2.*Prmm1.*r2_5.*t587.*t589.*t638.*t705.*2.0;
t1027 = Prmm1.*d1dot_5.*r2_5.*t705.*2.0;
t1028 = Prmm2.*d1dot_5.*r2_5.*t705.*8.0;
t1029 = Prmm3.*d1dot_5.*r3_5.*t705.*8.0;
t1030 = Prmm3.*d2dot_5.*r3_5.*t705.*8.0;
t1031 = Fr1_5.*L_vector3_5.*t690.*t719.*4.0;
t1032 = Fr2_5.*L_vector3_5.*t691.*t719.*4.0;
t1033 = Prmg.*Prml2.*Prmm1.*t700.*t723.*2.0;
t1034 = Prmm1.*t131.*t674.*t694.*t700.*t724;
t1035 = Prmm1.*t674.*t688.*t694.*t700.*t724;
t1036 = Prmm2.*t674.*t688.*t694.*t700.*t724.*4.0;
t1037 = Prmm3.*t674.*t694.*t700.*t709.*t724.*4.0;
t1107 = Prml2.*Prmm1.*d1dot_5.*t705.*2.0;
t1108 = Fr3_5.*L_vector1_5.*t690.*t719.*4.0;
t1109 = Fr3_5.*L_vector2_5.*t691.*t719.*4.0;
t1110 = Prmg.*Prmm1.*r2_5.*t700.*t723.*2.0;
t1111 = Prmg.*Prmm2.*r2_5.*t700.*t723.*4.0;
t1112 = Prmg.*Prmm3.*r3_5.*t700.*t723.*4.0;
t1113 = Prml2.*Prmm1.*r2_5.*t674.*t694.*t700.*t724.*2.0;
t1038 = t1027+t1028+t1029+t1030+t1031+t1032+t1033+t1034+t1035+t1036+t1037-t1107-t1108-t1109-t1110-t1111-t1112-t1113;
t1039 = t726.*t729.*t_phi_dot6;
t1040 = t1039-t725.*t728.*t_theta_dot6;
t1042 = t744.*t810.*t813;
t1043 = t809+t1042;
t1044 = Prmm3.*r3_6.*t1043;
t1045 = Fr1_6.*L_vector1_6.*t818;
t1046 = Fr2_6.*L_vector2_6.*t818;
t1047 = Fr3_6.*L_vector3_6.*t818;
t1048 = Prmm3.*t118.*t826;
t1114 = Prmg.*Prmm3.*t735.*t824;
t1049 = F6+t1044+t1045+t1046+t1047+t1048-t1114;
t1050 = 1.0./t728;
t1052 = Fr2_6.*L_vector1_6.*2.0;
t1053 = Prmm1.*d1dot_6.*r2_6.*t777.*t828.*2.0;
t1054 = Prmm2.*d1dot_6.*r2_6.*t777.*t828.*4.0;
t1055 = Prmm3.*d1dot_6.*r3_6.*t777.*t828.*4.0;
t1056 = Prmm3.*d2dot_6.*r3_6.*t777.*t828.*4.0;
t1057 = Prmm1.*t131.*t726.*t728.*t777.*t844;
t1058 = Prmm1.*t726.*t728.*t777.*t827.*t844;
t1059 = Prmm2.*t726.*t728.*t777.*t827.*t844.*4.0;
t1060 = Prmm3.*t726.*t728.*t777.*t844.*t848.*4.0;
t1061 = t1052+t1053+t1054+t1055+t1056+t1057+t1058+t1059+t1060-Fr1_6.*L_vector2_6.*2.0-Prml2.*Prmm1.*d1dot_6.*t777.*t828.*2.0-Prml2.*Prmm1.*r2_6.*t726.*t728.*t777.*t844.*2.0;
t1062 = Prmm1.*d1dot_6.*r2_6.*t844.*2.0;
t1063 = Prmm2.*d1dot_6.*r2_6.*t844.*8.0;
t1064 = Prmm3.*d1dot_6.*r3_6.*t844.*8.0;
t1065 = Prmm3.*d2dot_6.*r3_6.*t844.*8.0;
t1066 = Fr1_6.*L_vector3_6.*t829.*t858.*4.0;
t1067 = Fr2_6.*L_vector3_6.*t830.*t858.*4.0;
t1068 = Prmg.*Prml2.*Prmm1.*t839.*t862.*2.0;
t1069 = Prmm1.*t131.*t813.*t833.*t839.*t863;
t1070 = Prmm1.*t813.*t827.*t833.*t839.*t863;
t1071 = Prmm2.*t813.*t827.*t833.*t839.*t863.*4.0;
t1072 = Prmm3.*t813.*t833.*t839.*t848.*t863.*4.0;
t1115 = Prml2.*Prmm1.*d1dot_6.*t844.*2.0;
t1116 = Fr3_6.*L_vector1_6.*t829.*t858.*4.0;
t1117 = Fr3_6.*L_vector2_6.*t830.*t858.*4.0;
t1118 = Prmg.*Prmm1.*r2_6.*t839.*t862.*2.0;
t1119 = Prmg.*Prmm2.*r2_6.*t839.*t862.*4.0;
t1120 = Prmg.*Prmm3.*r3_6.*t839.*t862.*4.0;
t1121 = Prml2.*Prmm1.*r2_6.*t813.*t833.*t839.*t863.*2.0;
t1073 = t1062+t1063+t1064+t1065+t1066+t1067+t1068+t1069+t1070+t1071+t1072-t1115-t1116-t1117-t1118-t1119-t1120-t1121;
temp1 = reshape([-d1dot_1.*t8-d2dot_1.*t8-t_phi_dot1.*(d1dot_1.*t2.*t3+d2dot_1.*t2.*t3+t2.*t5.*t9.*t_phi_dot1-t3.*t6.*t9.*t_theta_dot1)-t_theta_dot1.*(d1dot_1.*t5.*t6+d2dot_1.*t5.*t6-t3.*t6.*t9.*t_phi_dot1+t2.*t5.*t9.*t_theta_dot1)-t2.*t5.*t118.*t128+t2.*t5.*t227.*t874-t2.*t3.*t9.*t157.*t898-t6.*t9.*t157.*t875.*t886.*2.0,-d1dot_1.*t865-d2dot_1.*t865-t_phi_dot1.*(d1dot_1.*t3.*t6+d2dot_1.*t3.*t6+t5.*t6.*t9.*t_phi_dot1+t2.*t3.*t9.*t_theta_dot1)+t_theta_dot1.*(d1dot_1.*t2.*t5+d2dot_1.*t2.*t5-t2.*t3.*t9.*t_phi_dot1-t5.*t6.*t9.*t_theta_dot1)-t5.*t6.*t118.*t128+t5.*t6.*t227.*t874-t3.*t6.*t9.*t157.*t898+t2.*t9.*t157.*t875.*t886.*2.0,t_phi_dot1.*(d1dot_1.*t5+d2dot_1.*t5-t3.*t9.*t_phi_dot1)+d1dot_1.*t5.*t_phi_dot1+d2dot_1.*t5.*t_phi_dot1-t3.*t118.*t128+t3.*t227.*t874+t5.*t9.*t157.*t898,-d1dot_2.*t174-d2dot_2.*t174-t_phi_dot2.*(d1dot_2.*t168.*t169+d2dot_2.*t168.*t169+t168.*t171.*t175.*t_phi_dot2-t169.*t172.*t175.*t_theta_dot2)-t_theta_dot2.*(d1dot_2.*t171.*t172+d2dot_2.*t171.*t172-t169.*t172.*t175.*t_phi_dot2+t168.*t171.*t175.*t_theta_dot2)-t118.*t168.*t171.*t270+t168.*t171.*t227.*t909-t168.*t169.*t175.*t297.*t933-t172.*t175.*t297.*t910.*t921.*2.0,-d1dot_2.*t900-d2dot_2.*t900-t_phi_dot2.*(d1dot_2.*t169.*t172+d2dot_2.*t169.*t172+t171.*t172.*t175.*t_phi_dot2+t168.*t169.*t175.*t_theta_dot2)+t_theta_dot2.*(d1dot_2.*t168.*t171+d2dot_2.*t168.*t171-t168.*t169.*t175.*t_phi_dot2-t171.*t172.*t175.*t_theta_dot2)-t118.*t171.*t172.*t270+t171.*t172.*t227.*t909-t169.*t172.*t175.*t297.*t933+t168.*t175.*t297.*t910.*t921.*2.0,t_phi_dot2.*(d1dot_2.*t171+d2dot_2.*t171-t169.*t175.*t_phi_dot2)+d1dot_2.*t171.*t_phi_dot2+d2dot_2.*t171.*t_phi_dot2-t118.*t169.*t270+t169.*t227.*t909+t171.*t175.*t297.*t933,-d1dot_3.*t314-d2dot_3.*t314-t_phi_dot3.*(d1dot_3.*t308.*t309+d2dot_3.*t308.*t309+t308.*t311.*t315.*t_phi_dot3-t309.*t312.*t315.*t_theta_dot3)-t_theta_dot3.*(d1dot_3.*t311.*t312+d2dot_3.*t311.*t312-t309.*t312.*t315.*t_phi_dot3+t308.*t311.*t315.*t_theta_dot3)-t118.*t308.*t311.*t409+t227.*t308.*t311.*t944-t308.*t309.*t315.*t436.*t968-t312.*t315.*t436.*t945.*t956.*2.0,-d1dot_3.*t935-d2dot_3.*t935-t_phi_dot3.*(d1dot_3.*t309.*t312+d2dot_3.*t309.*t312+t311.*t312.*t315.*t_phi_dot3+t308.*t309.*t315.*t_theta_dot3)+t_theta_dot3.*(d1dot_3.*t308.*t311+d2dot_3.*t308.*t311-t308.*t309.*t315.*t_phi_dot3-t311.*t312.*t315.*t_theta_dot3)-t118.*t311.*t312.*t409+t227.*t311.*t312.*t944-t309.*t312.*t315.*t436.*t968+t308.*t315.*t436.*t945.*t956.*2.0,t_phi_dot3.*(d1dot_3.*t311+d2dot_3.*t311-t309.*t315.*t_phi_dot3)+d1dot_3.*t311.*t_phi_dot3+d2dot_3.*t311.*t_phi_dot3-t118.*t309.*t409+t227.*t309.*t944+t311.*t315.*t436.*t968,-d1dot_4.*t453-d2dot_4.*t453-t_phi_dot4.*(d1dot_4.*t447.*t448+d2dot_4.*t447.*t448+t447.*t450.*t454.*t_phi_dot4-t448.*t451.*t454.*t_theta_dot4)-t_theta_dot4.*(d1dot_4.*t450.*t451+d2dot_4.*t450.*t451-t448.*t451.*t454.*t_phi_dot4+t447.*t450.*t454.*t_theta_dot4)-t118.*t447.*t450.*t548+t227.*t447.*t450.*t979-t447.*t448.*t454.*t575.*t1003-t451.*t454.*t575.*t980.*t991.*2.0,-d1dot_4.*t970-d2dot_4.*t970-t_phi_dot4.*(d1dot_4.*t448.*t451+d2dot_4.*t448.*t451+t450.*t451.*t454.*t_phi_dot4+t447.*t448.*t454.*t_theta_dot4)+t_theta_dot4.*(d1dot_4.*t447.*t450+d2dot_4.*t447.*t450-t447.*t448.*t454.*t_phi_dot4-t450.*t451.*t454.*t_theta_dot4)-t118.*t450.*t451.*t548+t227.*t450.*t451.*t979-t448.*t451.*t454.*t575.*t1003+t447.*t454.*t575.*t980.*t991.*2.0,t_phi_dot4.*(d1dot_4.*t450+d2dot_4.*t450-t448.*t454.*t_phi_dot4)+d1dot_4.*t450.*t_phi_dot4+d2dot_4.*t450.*t_phi_dot4-t118.*t448.*t548+t227.*t448.*t979+t450.*t454.*t575.*t1003,-d1dot_5.*t592-d2dot_5.*t592-t_phi_dot5.*(d1dot_5.*t586.*t587+d2dot_5.*t586.*t587+t586.*t589.*t593.*t_phi_dot5-t587.*t590.*t593.*t_theta_dot5)-t_theta_dot5.*(d1dot_5.*t589.*t590+d2dot_5.*t589.*t590-t587.*t590.*t593.*t_phi_dot5+t586.*t589.*t593.*t_theta_dot5)-t118.*t586.*t589.*t687+t227.*t586.*t589.*t1014-t586.*t587.*t593.*t714.*t1038-t590.*t593.*t714.*t1015.*t1026.*2.0,-d1dot_5.*t1005-d2dot_5.*t1005-t_phi_dot5.*(d1dot_5.*t587.*t590+d2dot_5.*t587.*t590+t589.*t590.*t593.*t_phi_dot5+t586.*t587.*t593.*t_theta_dot5)+t_theta_dot5.*(d1dot_5.*t586.*t589+d2dot_5.*t586.*t589-t586.*t587.*t593.*t_phi_dot5-t589.*t590.*t593.*t_theta_dot5)-t118.*t589.*t590.*t687+t227.*t589.*t590.*t1014-t587.*t590.*t593.*t714.*t1038+t586.*t593.*t714.*t1015.*t1026.*2.0,t_phi_dot5.*(d1dot_5.*t589+d2dot_5.*t589-t587.*t593.*t_phi_dot5)+d1dot_5.*t589.*t_phi_dot5+d2dot_5.*t589.*t_phi_dot5-t118.*t587.*t687+t227.*t587.*t1014+t589.*t593.*t714.*t1038,-d1dot_6.*t731-d2dot_6.*t731-t_phi_dot6.*(d1dot_6.*t725.*t726+d2dot_6.*t725.*t726+t725.*t728.*t732.*t_phi_dot6-t726.*t729.*t732.*t_theta_dot6)-t_theta_dot6.*(d1dot_6.*t728.*t729+d2dot_6.*t728.*t729-t726.*t729.*t732.*t_phi_dot6+t725.*t728.*t732.*t_theta_dot6)-t118.*t725.*t728.*t826+t227.*t725.*t728.*t1049-t725.*t726.*t732.*t853.*t1073-t729.*t732.*t853.*t1050.*t1061.*2.0,-d1dot_6.*t1040-d2dot_6.*t1040-t_phi_dot6.*(d1dot_6.*t726.*t729+d2dot_6.*t726.*t729+t728.*t729.*t732.*t_phi_dot6+t725.*t726.*t732.*t_theta_dot6)+t_theta_dot6.*(d1dot_6.*t725.*t728+d2dot_6.*t725.*t728-t725.*t726.*t732.*t_phi_dot6-t728.*t729.*t732.*t_theta_dot6)-t118.*t728.*t729.*t826+t227.*t728.*t729.*t1049-t726.*t729.*t732.*t853.*t1073+t725.*t732.*t853.*t1050.*t1061.*2.0,t_phi_dot6.*(d1dot_6.*t728+d2dot_6.*t728-t726.*t732.*t_phi_dot6)+d1dot_6.*t728.*t_phi_dot6+d2dot_6.*t728.*t_phi_dot6-t118.*t726.*t826+t227.*t726.*t1049+t728.*t732.*t853.*t1073],[3,6]);