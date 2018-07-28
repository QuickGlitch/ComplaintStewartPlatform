function temp2 = Calculate_F_lock(F1,F2,F3,F4,F5,F6,Fr1_1,Fr1_2,Fr1_3,Fr1_4,Fr1_5,Fr1_6,Fr2_1,Fr2_2,Fr2_3,Fr2_4,Fr2_5,Fr2_6,Fr3_1,Fr3_2,Fr3_3,Fr3_4,Fr3_5,Fr3_6,L_vector1_1,L_vector1_2,L_vector1_3,L_vector1_4,L_vector1_5,L_vector1_6,L_vector2_1,L_vector2_2,L_vector2_3,L_vector2_4,L_vector2_5,L_vector2_6,L_vector3_1,L_vector3_2,L_vector3_3,L_vector3_4,L_vector3_5,L_vector3_6,Prmg,Prmm1,Prmm2,Prmm3,PrmB_1,PrmB_2,PrmB_3,PrmB_4,PrmB_5,PrmB_6,Prmk_1,Prmk_2,Prmk_3,Prmk_4,Prmk_5,Prmk_6,TP1_1,TP1_2,TP1_3,TP1_4,TP1_5,TP1_6,TP2_1,TP2_2,TP2_3,TP2_4,TP2_5,TP2_6,TP3_1,TP3_2,TP3_3,TP3_4,TP3_5,TP3_6,alpha,alpha_dot,beta,beta_dot,d1_1,d1_2,d1_3,d1_4,d1_5,d1_6,d1dot_1,d1dot_2,d1dot_3,d1dot_4,d1dot_5,d1dot_6,gamma,gamma_dot,r2_1,r2_2,r2_3,r2_4,r2_5,r2_6,r3_1,r3_2,r3_3,r3_4,r3_5,r3_6,x,x_dot,xb1,xb2,xb3,xb4,xb5,xb6,y,y_dot,yb1,yb2,yb3,yb4,yb5,yb6,z,z_dot,zb1,zb2,zb3,zb4,zb5,zb6)
%CALCULATE_F_LOCK
%    TEMP2 = CALCULATE_F_LOCK(F1,F2,F3,F4,F5,F6,FR1_1,FR1_2,FR1_3,FR1_4,FR1_5,FR1_6,FR2_1,FR2_2,FR2_3,FR2_4,FR2_5,FR2_6,FR3_1,FR3_2,FR3_3,FR3_4,FR3_5,FR3_6,L_VECTOR1_1,L_VECTOR1_2,L_VECTOR1_3,L_VECTOR1_4,L_VECTOR1_5,L_VECTOR1_6,L_VECTOR2_1,L_VECTOR2_2,L_VECTOR2_3,L_VECTOR2_4,L_VECTOR2_5,L_VECTOR2_6,L_VECTOR3_1,L_VECTOR3_2,L_VECTOR3_3,L_VECTOR3_4,L_VECTOR3_5,L_VECTOR3_6,PRMG,PRMM1,PRMM2,PRMM3,PRMB_1,PRMB_2,PRMB_3,PRMB_4,PRMB_5,PRMB_6,PRMK_1,PRMK_2,PRMK_3,PRMK_4,PRMK_5,PRMK_6,TP1_1,TP1_2,TP1_3,TP1_4,TP1_5,TP1_6,TP2_1,TP2_2,TP2_3,TP2_4,TP2_5,TP2_6,TP3_1,TP3_2,TP3_3,TP3_4,TP3_5,TP3_6,ALPHA,ALPHA_DOT,BETA,BETA_DOT,D1_1,D1_2,D1_3,D1_4,D1_5,D1_6,D1DOT_1,D1DOT_2,D1DOT_3,D1DOT_4,D1DOT_5,D1DOT_6,GAMMA,GAMMA_DOT,R2_1,R2_2,R2_3,R2_4,R2_5,R2_6,R3_1,R3_2,R3_3,R3_4,R3_5,R3_6,X,X_DOT,XB1,XB2,XB3,XB4,XB5,XB6,Y,Y_DOT,YB1,YB2,YB3,YB4,YB5,YB6,Z,Z_DOT,ZB1,ZB2,ZB3,ZB4,ZB5,ZB6)

%    This function was generated by the Symbolic Math Toolbox version 6.3.
%    21-Sep-2016 12:23:42

t2 = L_vector1_1.^2;
t3 = L_vector2_1.^2;
t4 = L_vector3_1.^2;
t5 = t2+t3+t4;
t6 = sqrt(t5);
t7 = cos(beta);
t8 = sin(gamma);
t9 = cos(gamma);
t10 = sin(alpha);
t11 = sin(beta);
t13 = cos(alpha);
t16 = t7.*t9;
t17 = t8.*t10.*t11;
t18 = t16+t17;
t19 = TP1_1.*t18;
t20 = t7.*t8;
t21 = t9.*t10.*t11;
t22 = t20-t21;
t23 = TP2_1.*t22;
t24 = TP3_1.*t11.*t13;
t12 = t19-t23+t24+x-xb1;
t26 = TP3_1.*t10;
t27 = TP2_1.*t9.*t13;
t28 = TP1_1.*t8.*t13;
t14 = -t26+t27+t28+y-yb1;
t30 = t8.*t11;
t31 = t7.*t9.*t10;
t32 = t30+t31;
t33 = t9.*t11;
t34 = t7.*t8.*t10;
t35 = t33-t34;
t37 = TP1_1.*t35;
t38 = TP2_1.*t32;
t39 = TP3_1.*t7.*t13;
t15 = -t37+t38+t39+z-zb1;
t25 = t12.^2;
t29 = t14.^2;
t36 = t25+t29;
t40 = t15.^2;
t41 = t25+t29+t40;
t42 = 1.0./t41;
t43 = 1.0./sqrt(t36);
t45 = sqrt(t36);
t46 = t19-t23+t24;
t47 = beta_dot.*t46;
t48 = TP3_1.*t7.*t10;
t49 = TP2_1.*t7.*t9.*t13;
t50 = TP1_1.*t7.*t8.*t13;
t51 = -t48+t49+t50;
t52 = alpha_dot.*t51;
t53 = TP1_1.*t32;
t54 = TP2_1.*t35;
t55 = t53+t54;
t56 = gamma_dot.*t55;
t57 = -t47+t52+t56+z_dot;
t58 = t42.*t45.*t57;
t59 = -t37+t38+t39;
t60 = beta_dot.*t59;
t61 = TP3_1.*t10.*t11;
t62 = TP2_1.*t9.*t11.*t13;
t63 = TP1_1.*t8.*t11.*t13;
t64 = -t61+t62+t63;
t65 = alpha_dot.*t64;
t66 = TP1_1.*t22;
t67 = TP2_1.*t18;
t68 = t66+t67;
t69 = gamma_dot.*t68;
t70 = t60+t65-t69+x_dot;
t71 = x.*2.0;
t72 = xb1.*2.0;
t73 = TP1_1.*t18.*2.0;
t74 = TP2_1.*t22.*2.0;
t75 = TP3_1.*t11.*t13.*2.0;
t76 = t71-t72+t73-t74+t75;
t77 = t15.*t42.*t43.*t70.*t76.*(1.0./2.0);
t78 = TP3_1.*t13;
t79 = TP2_1.*t9.*t10;
t80 = TP1_1.*t8.*t10;
t81 = t78+t79+t80;
t82 = alpha_dot.*t81;
t83 = TP1_1.*t9.*t13;
t84 = TP2_1.*t8.*t13;
t85 = t83-t84;
t86 = gamma_dot.*t85;
t87 = -t82+t86+y_dot;
t88 = y.*2.0;
t89 = yb1.*2.0;
t90 = TP3_1.*t10.*2.0;
t91 = TP2_1.*t9.*t13.*2.0;
t92 = TP1_1.*t8.*t13.*2.0;
t93 = t88-t89-t90+t91+t92;
t94 = t15.*t42.*t43.*t87.*t93.*(1.0./2.0);
t44 = -t58+t77+t94;
t95 = t44.^2;
t96 = z.*1i;
t97 = zb1.*-1i;
t98 = TP1_1.*t35.*-1i;
t99 = TP2_1.*t32.*1i;
t100 = TP3_1.*t7.*t13.*1i;
t101 = t45+t96+t97+t98+t99+t100;
t102 = abs(t101);
t103 = 1.0./t36;
t106 = t14.*t70.*t103;
t107 = t12.*t87.*t103;
t104 = t106-t107;
t105 = 1.0./t102.^2;
t108 = t104.^2;
t109 = Prmm1+Prmm2+Prmm3;
t110 = 1.0./t109;
t111 = L_vector1_2.^2;
t112 = L_vector2_2.^2;
t113 = L_vector3_2.^2;
t114 = t111+t112+t113;
t115 = sqrt(t114);
t119 = TP1_2.*t18;
t120 = TP2_2.*t22;
t121 = TP3_2.*t11.*t13;
t116 = t119-t120+t121+x-xb2;
t123 = TP3_2.*t10;
t124 = TP2_2.*t9.*t13;
t125 = TP1_2.*t8.*t13;
t117 = -t123+t124+t125+y-yb2;
t128 = TP1_2.*t35;
t129 = TP2_2.*t32;
t130 = TP3_2.*t7.*t13;
t118 = -t128+t129+t130+z-zb2;
t122 = t116.^2;
t126 = t117.^2;
t127 = t122+t126;
t131 = t118.^2;
t132 = t122+t126+t131;
t133 = 1.0./t132;
t134 = 1.0./sqrt(t127);
t136 = sqrt(t127);
t137 = t119-t120+t121;
t138 = beta_dot.*t137;
t139 = TP3_2.*t7.*t10;
t140 = TP2_2.*t7.*t9.*t13;
t141 = TP1_2.*t7.*t8.*t13;
t142 = -t139+t140+t141;
t143 = alpha_dot.*t142;
t144 = TP1_2.*t32;
t145 = TP2_2.*t35;
t146 = t144+t145;
t147 = gamma_dot.*t146;
t148 = -t138+t143+t147+z_dot;
t149 = t133.*t136.*t148;
t150 = -t128+t129+t130;
t151 = beta_dot.*t150;
t152 = TP3_2.*t10.*t11;
t153 = TP2_2.*t9.*t11.*t13;
t154 = TP1_2.*t8.*t11.*t13;
t155 = -t152+t153+t154;
t156 = alpha_dot.*t155;
t157 = TP1_2.*t22;
t158 = TP2_2.*t18;
t159 = t157+t158;
t160 = gamma_dot.*t159;
t161 = t151+t156-t160+x_dot;
t162 = xb2.*2.0;
t163 = TP1_2.*t18.*2.0;
t164 = TP2_2.*t22.*2.0;
t165 = TP3_2.*t11.*t13.*2.0;
t166 = t71-t162+t163-t164+t165;
t167 = t118.*t133.*t134.*t161.*t166.*(1.0./2.0);
t168 = TP3_2.*t13;
t169 = TP2_2.*t9.*t10;
t170 = TP1_2.*t8.*t10;
t171 = t168+t169+t170;
t172 = alpha_dot.*t171;
t173 = TP1_2.*t9.*t13;
t174 = TP2_2.*t8.*t13;
t175 = t173-t174;
t176 = gamma_dot.*t175;
t177 = -t172+t176+y_dot;
t178 = yb2.*2.0;
t179 = TP3_2.*t10.*2.0;
t180 = TP2_2.*t9.*t13.*2.0;
t181 = TP1_2.*t8.*t13.*2.0;
t182 = t88-t178-t179+t180+t181;
t183 = t118.*t133.*t134.*t177.*t182.*(1.0./2.0);
t135 = -t149+t167+t183;
t184 = t135.^2;
t185 = zb2.*-1i;
t186 = TP1_2.*t35.*-1i;
t187 = TP2_2.*t32.*1i;
t188 = TP3_2.*t7.*t13.*1i;
t189 = t96+t136+t185+t186+t187+t188;
t190 = abs(t189);
t191 = 1.0./t127;
t194 = t117.*t161.*t191;
t195 = t116.*t177.*t191;
t192 = t194-t195;
t193 = 1.0./t190.^2;
t196 = t192.^2;
t197 = L_vector1_3.^2;
t198 = L_vector2_3.^2;
t199 = L_vector3_3.^2;
t200 = t197+t198+t199;
t201 = sqrt(t200);
t205 = TP1_3.*t18;
t206 = TP2_3.*t22;
t207 = TP3_3.*t11.*t13;
t202 = t205-t206+t207+x-xb3;
t209 = TP3_3.*t10;
t210 = TP2_3.*t9.*t13;
t211 = TP1_3.*t8.*t13;
t203 = -t209+t210+t211+y-yb3;
t214 = TP1_3.*t35;
t215 = TP2_3.*t32;
t216 = TP3_3.*t7.*t13;
t204 = -t214+t215+t216+z-zb3;
t208 = t202.^2;
t212 = t203.^2;
t213 = t208+t212;
t217 = t204.^2;
t218 = t208+t212+t217;
t219 = 1.0./t218;
t220 = 1.0./sqrt(t213);
t222 = sqrt(t213);
t223 = t205-t206+t207;
t224 = beta_dot.*t223;
t225 = TP3_3.*t7.*t10;
t226 = TP2_3.*t7.*t9.*t13;
t227 = TP1_3.*t7.*t8.*t13;
t228 = -t225+t226+t227;
t229 = alpha_dot.*t228;
t230 = TP1_3.*t32;
t231 = TP2_3.*t35;
t232 = t230+t231;
t233 = gamma_dot.*t232;
t234 = -t224+t229+t233+z_dot;
t235 = t219.*t222.*t234;
t236 = -t214+t215+t216;
t237 = beta_dot.*t236;
t238 = TP3_3.*t10.*t11;
t239 = TP2_3.*t9.*t11.*t13;
t240 = TP1_3.*t8.*t11.*t13;
t241 = -t238+t239+t240;
t242 = alpha_dot.*t241;
t243 = TP1_3.*t22;
t244 = TP2_3.*t18;
t245 = t243+t244;
t246 = gamma_dot.*t245;
t247 = t237+t242-t246+x_dot;
t248 = xb3.*2.0;
t249 = TP1_3.*t18.*2.0;
t250 = TP2_3.*t22.*2.0;
t251 = TP3_3.*t11.*t13.*2.0;
t252 = t71-t248+t249-t250+t251;
t253 = t204.*t219.*t220.*t247.*t252.*(1.0./2.0);
t254 = TP3_3.*t13;
t255 = TP2_3.*t9.*t10;
t256 = TP1_3.*t8.*t10;
t257 = t254+t255+t256;
t258 = alpha_dot.*t257;
t259 = TP1_3.*t9.*t13;
t260 = TP2_3.*t8.*t13;
t261 = t259-t260;
t262 = gamma_dot.*t261;
t263 = -t258+t262+y_dot;
t264 = yb3.*2.0;
t265 = TP3_3.*t10.*2.0;
t266 = TP2_3.*t9.*t13.*2.0;
t267 = TP1_3.*t8.*t13.*2.0;
t268 = t88-t264-t265+t266+t267;
t269 = t204.*t219.*t220.*t263.*t268.*(1.0./2.0);
t221 = -t235+t253+t269;
t270 = t221.^2;
t271 = zb3.*-1i;
t272 = TP1_3.*t35.*-1i;
t273 = TP2_3.*t32.*1i;
t274 = TP3_3.*t7.*t13.*1i;
t275 = t96+t222+t271+t272+t273+t274;
t276 = abs(t275);
t277 = 1.0./t213;
t280 = t203.*t247.*t277;
t281 = t202.*t263.*t277;
t278 = t280-t281;
t279 = 1.0./t276.^2;
t282 = t278.^2;
t283 = L_vector1_4.^2;
t284 = L_vector2_4.^2;
t285 = L_vector3_4.^2;
t286 = t283+t284+t285;
t287 = sqrt(t286);
t291 = TP1_4.*t18;
t292 = TP2_4.*t22;
t293 = TP3_4.*t11.*t13;
t288 = t291-t292+t293+x-xb4;
t295 = TP3_4.*t10;
t296 = TP2_4.*t9.*t13;
t297 = TP1_4.*t8.*t13;
t289 = -t295+t296+t297+y-yb4;
t300 = TP1_4.*t35;
t301 = TP2_4.*t32;
t302 = TP3_4.*t7.*t13;
t290 = -t300+t301+t302+z-zb4;
t294 = t288.^2;
t298 = t289.^2;
t299 = t294+t298;
t303 = t290.^2;
t304 = t294+t298+t303;
t305 = 1.0./t304;
t306 = 1.0./sqrt(t299);
t308 = sqrt(t299);
t309 = t291-t292+t293;
t310 = beta_dot.*t309;
t311 = TP3_4.*t7.*t10;
t312 = TP2_4.*t7.*t9.*t13;
t313 = TP1_4.*t7.*t8.*t13;
t314 = -t311+t312+t313;
t315 = alpha_dot.*t314;
t316 = TP1_4.*t32;
t317 = TP2_4.*t35;
t318 = t316+t317;
t319 = gamma_dot.*t318;
t320 = -t310+t315+t319+z_dot;
t321 = t305.*t308.*t320;
t322 = -t300+t301+t302;
t323 = beta_dot.*t322;
t324 = TP3_4.*t10.*t11;
t325 = TP2_4.*t9.*t11.*t13;
t326 = TP1_4.*t8.*t11.*t13;
t327 = -t324+t325+t326;
t328 = alpha_dot.*t327;
t329 = TP1_4.*t22;
t330 = TP2_4.*t18;
t331 = t329+t330;
t332 = gamma_dot.*t331;
t333 = t323+t328-t332+x_dot;
t334 = xb4.*2.0;
t335 = TP1_4.*t18.*2.0;
t336 = TP2_4.*t22.*2.0;
t337 = TP3_4.*t11.*t13.*2.0;
t338 = t71-t334+t335-t336+t337;
t339 = t290.*t305.*t306.*t333.*t338.*(1.0./2.0);
t340 = TP3_4.*t13;
t341 = TP2_4.*t9.*t10;
t342 = TP1_4.*t8.*t10;
t343 = t340+t341+t342;
t344 = alpha_dot.*t343;
t345 = TP1_4.*t9.*t13;
t346 = TP2_4.*t8.*t13;
t347 = t345-t346;
t348 = gamma_dot.*t347;
t349 = -t344+t348+y_dot;
t350 = yb4.*2.0;
t351 = TP3_4.*t10.*2.0;
t352 = TP2_4.*t9.*t13.*2.0;
t353 = TP1_4.*t8.*t13.*2.0;
t354 = t88-t350-t351+t352+t353;
t355 = t290.*t305.*t306.*t349.*t354.*(1.0./2.0);
t307 = -t321+t339+t355;
t356 = t307.^2;
t357 = zb4.*-1i;
t358 = TP1_4.*t35.*-1i;
t359 = TP2_4.*t32.*1i;
t360 = TP3_4.*t7.*t13.*1i;
t361 = t96+t308+t357+t358+t359+t360;
t362 = abs(t361);
t363 = 1.0./t299;
t366 = t289.*t333.*t363;
t367 = t288.*t349.*t363;
t364 = t366-t367;
t365 = 1.0./t362.^2;
t368 = t364.^2;
t369 = L_vector1_5.^2;
t370 = L_vector2_5.^2;
t371 = L_vector3_5.^2;
t372 = t369+t370+t371;
t373 = sqrt(t372);
t377 = TP1_5.*t18;
t378 = TP2_5.*t22;
t379 = TP3_5.*t11.*t13;
t374 = t377-t378+t379+x-xb5;
t381 = TP3_5.*t10;
t382 = TP2_5.*t9.*t13;
t383 = TP1_5.*t8.*t13;
t375 = -t381+t382+t383+y-yb5;
t386 = TP1_5.*t35;
t387 = TP2_5.*t32;
t388 = TP3_5.*t7.*t13;
t376 = -t386+t387+t388+z-zb5;
t380 = t374.^2;
t384 = t375.^2;
t385 = t380+t384;
t389 = t376.^2;
t390 = t380+t384+t389;
t391 = 1.0./t390;
t392 = 1.0./sqrt(t385);
t394 = sqrt(t385);
t395 = t377-t378+t379;
t396 = beta_dot.*t395;
t397 = TP3_5.*t7.*t10;
t398 = TP2_5.*t7.*t9.*t13;
t399 = TP1_5.*t7.*t8.*t13;
t400 = -t397+t398+t399;
t401 = alpha_dot.*t400;
t402 = TP1_5.*t32;
t403 = TP2_5.*t35;
t404 = t402+t403;
t405 = gamma_dot.*t404;
t406 = -t396+t401+t405+z_dot;
t407 = t391.*t394.*t406;
t408 = -t386+t387+t388;
t409 = beta_dot.*t408;
t410 = TP3_5.*t10.*t11;
t411 = TP2_5.*t9.*t11.*t13;
t412 = TP1_5.*t8.*t11.*t13;
t413 = -t410+t411+t412;
t414 = alpha_dot.*t413;
t415 = TP1_5.*t22;
t416 = TP2_5.*t18;
t417 = t415+t416;
t418 = gamma_dot.*t417;
t419 = t409+t414-t418+x_dot;
t420 = xb5.*2.0;
t421 = TP1_5.*t18.*2.0;
t422 = TP2_5.*t22.*2.0;
t423 = TP3_5.*t11.*t13.*2.0;
t424 = t71-t420+t421-t422+t423;
t425 = t376.*t391.*t392.*t419.*t424.*(1.0./2.0);
t426 = TP3_5.*t13;
t427 = TP2_5.*t9.*t10;
t428 = TP1_5.*t8.*t10;
t429 = t426+t427+t428;
t430 = alpha_dot.*t429;
t431 = TP1_5.*t9.*t13;
t432 = TP2_5.*t8.*t13;
t433 = t431-t432;
t434 = gamma_dot.*t433;
t435 = -t430+t434+y_dot;
t436 = yb5.*2.0;
t437 = TP3_5.*t10.*2.0;
t438 = TP2_5.*t9.*t13.*2.0;
t439 = TP1_5.*t8.*t13.*2.0;
t440 = t88-t436-t437+t438+t439;
t441 = t376.*t391.*t392.*t435.*t440.*(1.0./2.0);
t393 = -t407+t425+t441;
t442 = t393.^2;
t443 = zb5.*-1i;
t444 = TP1_5.*t35.*-1i;
t445 = TP2_5.*t32.*1i;
t446 = TP3_5.*t7.*t13.*1i;
t447 = t96+t394+t443+t444+t445+t446;
t448 = abs(t447);
t449 = 1.0./t385;
t452 = t375.*t419.*t449;
t453 = t374.*t435.*t449;
t450 = t452-t453;
t451 = 1.0./t448.^2;
t454 = t450.^2;
t455 = L_vector1_6.^2;
t456 = L_vector2_6.^2;
t457 = L_vector3_6.^2;
t458 = t455+t456+t457;
t459 = sqrt(t458);
t463 = TP1_6.*t18;
t464 = TP2_6.*t22;
t465 = TP3_6.*t11.*t13;
t460 = t463-t464+t465+x-xb6;
t467 = TP3_6.*t10;
t468 = TP2_6.*t9.*t13;
t469 = TP1_6.*t8.*t13;
t461 = -t467+t468+t469+y-yb6;
t472 = TP1_6.*t35;
t473 = TP2_6.*t32;
t474 = TP3_6.*t7.*t13;
t462 = -t472+t473+t474+z-zb6;
t466 = t460.^2;
t470 = t461.^2;
t471 = t466+t470;
t475 = t462.^2;
t476 = t466+t470+t475;
t477 = 1.0./t476;
t478 = 1.0./sqrt(t471);
t480 = sqrt(t471);
t481 = t463-t464+t465;
t482 = beta_dot.*t481;
t483 = TP3_6.*t7.*t10;
t484 = TP2_6.*t7.*t9.*t13;
t485 = TP1_6.*t7.*t8.*t13;
t486 = -t483+t484+t485;
t487 = alpha_dot.*t486;
t488 = TP1_6.*t32;
t489 = TP2_6.*t35;
t490 = t488+t489;
t491 = gamma_dot.*t490;
t492 = -t482+t487+t491+z_dot;
t493 = t477.*t480.*t492;
t494 = -t472+t473+t474;
t495 = beta_dot.*t494;
t496 = TP3_6.*t10.*t11;
t497 = TP2_6.*t9.*t11.*t13;
t498 = TP1_6.*t8.*t11.*t13;
t499 = -t496+t497+t498;
t500 = alpha_dot.*t499;
t501 = TP1_6.*t22;
t502 = TP2_6.*t18;
t503 = t501+t502;
t504 = gamma_dot.*t503;
t505 = t495+t500-t504+x_dot;
t506 = xb6.*2.0;
t507 = TP1_6.*t18.*2.0;
t508 = TP2_6.*t22.*2.0;
t509 = TP3_6.*t11.*t13.*2.0;
t510 = t71-t506+t507-t508+t509;
t511 = t462.*t477.*t478.*t505.*t510.*(1.0./2.0);
t512 = TP3_6.*t13;
t513 = TP2_6.*t9.*t10;
t514 = TP1_6.*t8.*t10;
t515 = t512+t513+t514;
t516 = alpha_dot.*t515;
t517 = TP1_6.*t9.*t13;
t518 = TP2_6.*t8.*t13;
t519 = t517-t518;
t520 = gamma_dot.*t519;
t521 = -t516+t520+y_dot;
t522 = yb6.*2.0;
t523 = TP3_6.*t10.*2.0;
t524 = TP2_6.*t9.*t13.*2.0;
t525 = TP1_6.*t8.*t13.*2.0;
t526 = t88-t522-t523+t524+t525;
t527 = t462.*t477.*t478.*t521.*t526.*(1.0./2.0);
t479 = -t493+t511+t527;
t528 = t479.^2;
t529 = zb6.*-1i;
t530 = TP1_6.*t35.*-1i;
t531 = TP2_6.*t32.*1i;
t532 = TP3_6.*t7.*t13.*1i;
t533 = t96+t480+t529+t530+t531+t532;
t534 = abs(t533);
t535 = 1.0./t471;
t538 = t461.*t505.*t535;
t539 = t460.*t521.*t535;
t536 = t538-t539;
t537 = 1.0./t534.^2;
t540 = t536.^2;
temp2 = [-1.0./sqrt(t5).*t110.*(Fr1_1.*L_vector1_1.*Prmm1+Fr1_1.*L_vector1_1.*Prmm2+Fr2_1.*L_vector2_1.*Prmm1+Fr2_1.*L_vector2_1.*Prmm2+Fr3_1.*L_vector3_1.*Prmm1+Fr3_1.*L_vector3_1.*Prmm2+F1.*Prmm3.*t6+Prmm3.*Prmk_1.*d1_1.*t6.*2.0+Prmm3.*PrmB_1.*d1dot_1.*t6.*2.0-Prmm1.*Prmm3.*r2_1.*t6.*t95.*2.0-Prmm2.*Prmm3.*r2_1.*t6.*t95.*2.0+Prmm1.*Prmm3.*r3_1.*t6.*t95+Prmm2.*Prmm3.*r3_1.*t6.*t95+(Prmg.*Prmm2.*Prmm3.*t6.*t15)./t102-Prmm1.*Prmm3.*r2_1.*t6.*t36.*t105.*t108.*2.0-Prmm2.*Prmm3.*r2_1.*t6.*t36.*t105.*t108.*2.0+Prmm1.*Prmm3.*r3_1.*t6.*t36.*t105.*t108+Prmm2.*Prmm3.*r3_1.*t6.*t36.*t105.*t108);-t110.*1.0./sqrt(t114).*(Fr1_2.*L_vector1_2.*Prmm1+Fr1_2.*L_vector1_2.*Prmm2+Fr2_2.*L_vector2_2.*Prmm1+Fr2_2.*L_vector2_2.*Prmm2+Fr3_2.*L_vector3_2.*Prmm1+Fr3_2.*L_vector3_2.*Prmm2+F2.*Prmm3.*t115+Prmm3.*Prmk_2.*d1_2.*t115.*2.0+Prmm3.*PrmB_2.*d1dot_2.*t115.*2.0-Prmm1.*Prmm3.*r2_2.*t115.*t184.*2.0-Prmm2.*Prmm3.*r2_2.*t115.*t184.*2.0+Prmm1.*Prmm3.*r3_2.*t115.*t184+Prmm2.*Prmm3.*r3_2.*t115.*t184+(Prmg.*Prmm2.*Prmm3.*t115.*t118)./t190-Prmm1.*Prmm3.*r2_2.*t115.*t127.*t193.*t196.*2.0-Prmm2.*Prmm3.*r2_2.*t115.*t127.*t193.*t196.*2.0+Prmm1.*Prmm3.*r3_2.*t115.*t127.*t193.*t196+Prmm2.*Prmm3.*r3_2.*t115.*t127.*t193.*t196);-t110.*1.0./sqrt(t200).*(Fr1_3.*L_vector1_3.*Prmm1+Fr1_3.*L_vector1_3.*Prmm2+Fr2_3.*L_vector2_3.*Prmm1+Fr2_3.*L_vector2_3.*Prmm2+Fr3_3.*L_vector3_3.*Prmm1+Fr3_3.*L_vector3_3.*Prmm2+F3.*Prmm3.*t201+Prmm3.*Prmk_3.*d1_3.*t201.*2.0+Prmm3.*PrmB_3.*d1dot_3.*t201.*2.0-Prmm1.*Prmm3.*r2_3.*t201.*t270.*2.0-Prmm2.*Prmm3.*r2_3.*t201.*t270.*2.0+Prmm1.*Prmm3.*r3_3.*t201.*t270+Prmm2.*Prmm3.*r3_3.*t201.*t270+(Prmg.*Prmm2.*Prmm3.*t201.*t204)./t276-Prmm1.*Prmm3.*r2_3.*t201.*t213.*t279.*t282.*2.0-Prmm2.*Prmm3.*r2_3.*t201.*t213.*t279.*t282.*2.0+Prmm1.*Prmm3.*r3_3.*t201.*t213.*t279.*t282+Prmm2.*Prmm3.*r3_3.*t201.*t213.*t279.*t282);-t110.*1.0./sqrt(t286).*(Fr1_4.*L_vector1_4.*Prmm1+Fr1_4.*L_vector1_4.*Prmm2+Fr2_4.*L_vector2_4.*Prmm1+Fr2_4.*L_vector2_4.*Prmm2+Fr3_4.*L_vector3_4.*Prmm1+Fr3_4.*L_vector3_4.*Prmm2+F4.*Prmm3.*t287+Prmm3.*Prmk_4.*d1_4.*t287.*2.0+Prmm3.*PrmB_4.*d1dot_4.*t287.*2.0-Prmm1.*Prmm3.*r2_4.*t287.*t356.*2.0-Prmm2.*Prmm3.*r2_4.*t287.*t356.*2.0+Prmm1.*Prmm3.*r3_4.*t287.*t356+Prmm2.*Prmm3.*r3_4.*t287.*t356+(Prmg.*Prmm2.*Prmm3.*t287.*t290)./t362-Prmm1.*Prmm3.*r2_4.*t287.*t299.*t365.*t368.*2.0-Prmm2.*Prmm3.*r2_4.*t287.*t299.*t365.*t368.*2.0+Prmm1.*Prmm3.*r3_4.*t287.*t299.*t365.*t368+Prmm2.*Prmm3.*r3_4.*t287.*t299.*t365.*t368);-t110.*1.0./sqrt(t372).*(Fr1_5.*L_vector1_5.*Prmm1+Fr1_5.*L_vector1_5.*Prmm2+Fr2_5.*L_vector2_5.*Prmm1+Fr2_5.*L_vector2_5.*Prmm2+Fr3_5.*L_vector3_5.*Prmm1+Fr3_5.*L_vector3_5.*Prmm2+F5.*Prmm3.*t373+Prmm3.*Prmk_5.*d1_5.*t373.*2.0+Prmm3.*PrmB_5.*d1dot_5.*t373.*2.0-Prmm1.*Prmm3.*r2_5.*t373.*t442.*2.0-Prmm2.*Prmm3.*r2_5.*t373.*t442.*2.0+Prmm1.*Prmm3.*r3_5.*t373.*t442+Prmm2.*Prmm3.*r3_5.*t373.*t442+(Prmg.*Prmm2.*Prmm3.*t373.*t376)./t448-Prmm1.*Prmm3.*r2_5.*t373.*t385.*t451.*t454.*2.0-Prmm2.*Prmm3.*r2_5.*t373.*t385.*t451.*t454.*2.0+Prmm1.*Prmm3.*r3_5.*t373.*t385.*t451.*t454+Prmm2.*Prmm3.*r3_5.*t373.*t385.*t451.*t454);-t110.*1.0./sqrt(t458).*(Fr1_6.*L_vector1_6.*Prmm1+Fr1_6.*L_vector1_6.*Prmm2+Fr2_6.*L_vector2_6.*Prmm1+Fr2_6.*L_vector2_6.*Prmm2+Fr3_6.*L_vector3_6.*Prmm1+Fr3_6.*L_vector3_6.*Prmm2+F6.*Prmm3.*t459+Prmm3.*Prmk_6.*d1_6.*t459.*2.0+Prmm3.*PrmB_6.*d1dot_6.*t459.*2.0-Prmm1.*Prmm3.*r2_6.*t459.*t528.*2.0-Prmm2.*Prmm3.*r2_6.*t459.*t528.*2.0+Prmm1.*Prmm3.*r3_6.*t459.*t528+Prmm2.*Prmm3.*r3_6.*t459.*t528+(Prmg.*Prmm2.*Prmm3.*t459.*t462)./t534-Prmm1.*Prmm3.*r2_6.*t459.*t471.*t537.*t540.*2.0-Prmm2.*Prmm3.*r2_6.*t459.*t471.*t537.*t540.*2.0+Prmm1.*Prmm3.*r3_6.*t459.*t471.*t537.*t540+Prmm2.*Prmm3.*r3_6.*t459.*t471.*t537.*t540)];