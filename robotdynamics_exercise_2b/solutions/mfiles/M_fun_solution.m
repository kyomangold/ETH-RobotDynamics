function M = M_fun(in1)
%M_FUN
%    M = M_FUN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    17-Oct-2017 14:53:27

phi1 = in1(1,:);
phi2 = in1(2,:);
phi3 = in1(3,:);
phi4 = in1(4,:);
phi5 = in1(5,:);
phi6 = in1(6,:);
t3 = phi2+phi3;
t2 = cos(t3);
t4 = sin(t3);
t5 = sin(phi6);
t6 = sin(phi5);
t7 = t4.*t6;
t8 = cos(phi4);
t9 = cos(phi5);
t13 = t2.*t8.*t9;
t10 = t7-t13;
t11 = cos(phi6);
t12 = sin(phi4);
t14 = cos(phi2);
t15 = sin(phi2);
t16 = cos(phi1);
t17 = sin(phi1);
t18 = t12.*t17;
t19 = t4.*t8.*t16;
t20 = t18+t19;
t21 = t2.*1.51e2;
t22 = t4.*3.5e1;
t23 = t15.*1.35e2;
t24 = t21+t22+t23;
t25 = t16.*t24.*(1.0./5.0e2);
t26 = t4.*t9;
t27 = t2.*t6.*t8;
t28 = t26+t27;
t29 = t12.*t16;
t31 = t4.*t8.*t17;
t30 = t29-t31;
t32 = t17.*t24.*(1.0./5.0e2);
t33 = t2.^2;
t37 = t2.*5.0e1;
t38 = t4.*3.0;
t39 = t15.*2.7e1;
t34 = t37+t38+t39;
t35 = t2.*6.7e1;
t36 = t22+t23+t35;
t40 = t34.^2;
t41 = t16.^2;
t42 = t15.^2;
t43 = t17.^2;
t44 = sin(phi3);
t45 = cos(phi3);
t46 = t9.^2;
t47 = t11.^2;
t48 = t8.^2;
t49 = t12.*t14.*t45.*6.37e-3;
t50 = t8.*t12.*t14.*t45.*8.842576e-3;
t51 = t9.*t12.*t14.*t45.*1.155e-3;
t52 = t6.*t12.*t14.*t44.*3.04416e-4;
t53 = t6.*t12.*t15.*t45.*3.04416e-4;
t54 = t6.*t12.*t15.*t44.*7.056e-5;
t55 = t8.*t12.*t15.*t44.*t46.*7.7576e-5;
t56 = t8.*t12.*t15.*t44.*t47.*(1.0./1.0e3);
t57 = t5.*t9.*t11.*t14.*t45.*(1.0./1.0e3);
t58 = t6.*t9.*t12.*t14.*t44.*7.7576e-5;
t59 = t6.*t9.*t12.*t15.*t45.*7.7576e-5;
t60 = t8.*t12.*t15.*t44.*t46.*t47.*(1.0./1.0e3);
t61 = t5.*t6.*t8.*t11.*t14.*t44.*(1.0./1.0e3);
t62 = t5.*t6.*t8.*t11.*t15.*t45.*(1.0./1.0e3);
t63 = t6.*t9.*t12.*t14.*t44.*t47.*(1.0./1.0e3);
t64 = t6.*t9.*t12.*t15.*t45.*t47.*(1.0./1.0e3);
t65 = t5.*t9.*t11.*t15.*t44.*t48.*(1.0./5.0e2);
t66 = t5.*t10.*(1.0./5.0e2);
t81 = t2.*t11.*t12.*(1.0./5.0e2);
t67 = t66-t81;
t68 = t10.*t11.*(3.0./1.0e3);
t69 = t2.*t5.*t12.*(3.0./1.0e3);
t70 = t68+t69;
t71 = t8.*t17;
t189 = t4.*t12.*t16;
t72 = t71-t189;
t73 = t9.*t20;
t74 = t2.*t6.*t16;
t75 = t73+t74;
t76 = t4.*t9.*(1.0./5.0e2);
t77 = t2.*t6.*t8.*(1.0./5.0e2);
t78 = t76+t77;
t79 = t6.*t20;
t109 = t2.*t9.*t16;
t80 = t79-t109;
t82 = t8.*t16;
t83 = t4.*t12.*t17;
t84 = t82+t83;
t85 = t9.*t30;
t196 = t2.*t6.*t17;
t86 = t85-t196;
t87 = t6.*t30;
t88 = t2.*t9.*t17;
t89 = t87+t88;
t90 = t5.*t10;
t229 = t2.*t11.*t12;
t91 = t90-t229;
t92 = t67.*t91;
t93 = t10.*t11;
t94 = t2.*t5.*t12;
t95 = t93+t94;
t96 = t70.*t95;
t97 = t28.*t78;
t98 = t12.*t14.*2.457e-2;
t99 = t9.*t12.*t14.*4.455e-3;
t110 = t12.*t15.*t44.*6.37e-3;
t111 = t12.*t14.*t44.*1.2194e-2;
t112 = t12.*t15.*t45.*1.2194e-2;
t113 = t6.*t12.*t14.*t45.*7.056e-5;
t114 = t9.*t12.*t14.*t44.*4.983e-3;
t115 = t9.*t12.*t15.*t45.*4.983e-3;
t116 = t8.*t12.*t15.*t44.*8.842576e-3;
t117 = t9.*t12.*t15.*t44.*1.155e-3;
t118 = t5.*t9.*t11.*t15.*t44.*(1.0./1.0e3);
t119 = t8.*t12.*t14.*t45.*t46.*7.7576e-5;
t120 = t8.*t12.*t14.*t45.*t47.*(1.0./1.0e3);
t121 = t8.*t12.*t14.*t45.*t46.*t47.*(1.0./1.0e3);
t122 = t5.*t9.*t11.*t14.*t45.*t48.*(1.0./5.0e2);
t100 = t49+t50+t51+t52+t53+t54+t55+t56+t57+t58+t59+t60+t61+t62+t63+t64+t65+t98+t99-t110-t111-t112-t113-t114-t115-t116-t117-t118-t119-t120-t121-t122-t6.*t12.*t14.*2.7216e-4;
t101 = t8.*1.274e-2;
t102 = t9.*6.08832e-4;
t103 = t6.*9.966e-3;
t104 = t8.*t9.*2.31e-3;
t105 = t48.*8.842576e-3;
t106 = t46.*7.7576e-5;
t107 = t46.*t47.*(1.0./1.0e3);
t108 = t5.*t8.*t9.*t11.*t12.*(1.0./5.0e2);
t123 = t45.*5.87196e-2;
t124 = t8.*t45.*2.457e-2;
t125 = t8.*t9.*t45.*4.455e-3;
t127 = t6.*t8.*1.4112e-4;
t126 = t44.*(-4.8452256e-1)+t101+t102+t103+t104+t105+t106+t107+t108+t123+t124+t125-t127-t6.*t44.*4.455e-3-t9.*t44.*2.7216e-4-t46.*t48.*7.7576e-5-t47.*t48.*(1.0./1.0e3)-t6.*t8.*t45.*2.7216e-4-t46.*t47.*t48.*(1.0./1.0e3)+8.31420456e-1;
t128 = phi6.*2.0;
t129 = cos(t128);
t130 = t12.*1.2194e-2;
t131 = t9.*t12.*4.983e-3;
t132 = t8.*4.367576e-3;
t133 = t9.*1.155e-3;
t134 = t8.*t9.*3.04416e-4;
t135 = t6.*t8.*4.983e-3;
t136 = t5.*t9.*t11.*t12.*(1.0./1.0e3);
t137 = t2.*t8.*t12.*(1.0./1.0e3);
t138 = t2.*t5.*t9.*t11.*(1.0./1.0e3);
t139 = t4.*t5.*t6.*t8.*t11.*(1.0./1.0e3);
t140 = t4.*t6.*t9.*t12.*t47.*(1.0./1.0e3);
t258 = t2.*t8.*t12.*t47.*(1.0./1.0e3);
t259 = t2.*t8.*t12.*t46.*t47.*(1.0./1.0e3);
t260 = t2.*t5.*t9.*t11.*t48.*(1.0./5.0e2);
t141 = t137+t138+t139+t140-t258-t259-t260;
t142 = t6.*t12.*t17;
t143 = t4.*t6.*t8.*t16;
t144 = -t109+t142+t143;
t145 = t17.*(1.0./5.0e2);
t146 = t17.*t48.*(1.0./1.0e3);
t147 = t17.*t46.*t47.*(1.0./1.0e3);
t148 = t4.*t8.*t12.*t16.*t47.*(1.0./1.0e3);
t149 = t4.*t8.*t12.*t16.*t46.*t47.*(1.0./1.0e3);
t150 = t2.*t5.*t6.*t8.*t11.*t16.*(1.0./1.0e3);
t151 = t5.*t8.*t9.*t11.*t12.*t17.*(1.0./5.0e2);
t152 = t2.*t6.*t9.*t12.*t16.*t47.*(1.0./1.0e3);
t153 = t4.*t5.*t9.*t11.*t16.*t48.*(1.0./5.0e2);
t262 = t17.*t47.*t48.*(1.0./1.0e3);
t263 = t4.*t8.*t12.*t16.*(1.0./1.0e3);
t264 = t17.*t46.*t47.*t48.*(1.0./1.0e3);
t265 = t4.*t5.*t9.*t11.*t16.*(1.0./1.0e3);
t154 = t145+t146+t147+t148+t149+t150+t151+t152+t153-t262-t263-t264-t265;
t155 = t144.*t154;
t156 = t6.*t12.*t16;
t194 = t4.*t6.*t8.*t17;
t157 = t88+t156-t194;
t158 = t16.*t47.*t48.*(1.0./1.0e3);
t159 = t16.*t46.*t47.*t48.*(1.0./1.0e3);
t160 = t4.*t8.*t12.*t17.*t47.*(1.0./1.0e3);
t161 = t4.*t8.*t12.*t17.*t46.*t47.*(1.0./1.0e3);
t162 = t2.*t5.*t6.*t8.*t11.*t17.*(1.0./1.0e3);
t163 = t2.*t6.*t9.*t12.*t17.*t47.*(1.0./1.0e3);
t164 = t4.*t5.*t9.*t11.*t17.*t48.*(1.0./5.0e2);
t266 = t16.*(1.0./5.0e2);
t267 = t16.*t48.*(1.0./1.0e3);
t268 = t16.*t46.*t47.*(1.0./1.0e3);
t269 = t4.*t8.*t12.*t17.*(1.0./1.0e3);
t270 = t4.*t5.*t9.*t11.*t17.*(1.0./1.0e3);
t271 = t5.*t8.*t9.*t11.*t12.*t16.*(1.0./5.0e2);
t165 = t158+t159+t160+t161+t162+t163+t164-t266-t267-t268-t269-t270-t271;
t261 = t28.*t141;
t272 = t157.*t165;
t166 = t155-t261-t272;
t167 = t14.*t44.*t46.*7.7576e-5;
t168 = t15.*t45.*t46.*7.7576e-5;
t169 = t8.*t15.*t44.*1.2194e-2;
t170 = t6.*t8.*t15.*2.7216e-4;
t171 = t14.*t44.*t46.*t47.*(1.0./1.0e3);
t172 = t15.*t45.*t46.*t47.*(1.0./1.0e3);
t173 = t6.*t8.*t14.*t45.*3.04416e-4;
t174 = t6.*t8.*t14.*t44.*7.056e-5;
t175 = t6.*t8.*t15.*t45.*7.056e-5;
t176 = t8.*t9.*t15.*t44.*4.983e-3;
t177 = t6.*t8.*t9.*t14.*t45.*7.7576e-5;
t178 = t5.*t6.*t11.*t12.*t15.*t44.*(1.0./1.0e3);
t179 = t6.*t8.*t9.*t14.*t45.*t47.*(1.0./1.0e3);
t180 = t167+t168+t169+t170+t171+t172+t173+t174+t175+t176+t177+t178+t179-t8.*t15.*2.457e-2-t14.*t44.*1.2342576e-2-t15.*t45.*1.2342576e-2-t8.*t9.*t15.*4.455e-3-t8.*t14.*t44.*6.37e-3-t8.*t14.*t45.*1.2194e-2-t8.*t15.*t45.*6.37e-3-t14.*t44.*t47.*(1.0./1.0e3)-t15.*t45.*t47.*(1.0./1.0e3)-t6.*t8.*t15.*t44.*3.04416e-4-t8.*t9.*t14.*t44.*1.155e-3-t8.*t9.*t14.*t45.*4.983e-3-t8.*t9.*t15.*t45.*1.155e-3-t6.*t8.*t9.*t15.*t44.*7.7576e-5-t5.*t6.*t11.*t12.*t14.*t45.*(1.0./1.0e3)-t6.*t8.*t9.*t15.*t44.*t47.*(1.0./1.0e3);
t181 = t6.*t12.*t44.*2.7216e-4;
t183 = t6.*t12.*3.04416e-4;
t184 = t6.*t9.*t12.*7.7576e-5;
t185 = t6.*t9.*t12.*t47.*(1.0./1.0e3);
t186 = t5.*t6.*t8.*t11.*(1.0./1.0e3);
t182 = t130+t131+t181-t183-t184-t185-t186-t12.*t44.*2.457e-2-t9.*t12.*t44.*4.455e-3;
t187 = phi5.*2.0;
t188 = cos(t187);
t190 = t5.*t72;
t191 = t9.*t12.*t17;
t192 = t4.*t8.*t9.*t16;
t193 = t74+t191+t192;
t195 = t5.*t84;
t197 = t6.*t12.*t15.*4.455e-3;
t198 = t12.*t14.*t45.*4.367576e-3;
t199 = t9.*t12.*t15.*2.7216e-4;
t200 = t12.*t15.*t44.*t47.*(1.0./1.0e3);
t201 = t9.*t12.*t14.*t45.*3.04416e-4;
t202 = t6.*t12.*t14.*t45.*4.983e-3;
t203 = t9.*t12.*t14.*t44.*7.056e-5;
t204 = t9.*t12.*t15.*t45.*7.056e-5;
t205 = t6.*t12.*t14.*t44.*1.155e-3;
t206 = t6.*t12.*t15.*t45.*1.155e-3;
t207 = t5.*t6.*t11.*t14.*t44.*(1.0./1.0e3);
t208 = t5.*t6.*t11.*t15.*t45.*(1.0./1.0e3);
t209 = t5.*t8.*t9.*t11.*t15.*t44.*(1.0./1.0e3);
t210 = t197+t198+t199+t200+t201+t202+t203+t204+t205+t206+t207+t208+t209-t12.*t15.*t44.*4.367576e-3-t6.*t12.*t15.*t44.*4.983e-3-t9.*t12.*t15.*t44.*3.04416e-4-t12.*t14.*t45.*t47.*(1.0./1.0e3)-t5.*t8.*t9.*t11.*t14.*t45.*(1.0./1.0e3);
t211 = t9.*t45.*4.455e-3;
t213 = t6.*7.056e-5;
t214 = t8.*t47.*(1.0./1.0e3);
t212 = t132+t133+t134+t135+t136+t211-t213-t214-t6.*t45.*2.7216e-4-t6.*t8.*t44.*4.455e-3-t8.*t9.*t44.*2.7216e-4;
t215 = t4.*t8.*t9.*t17;
t220 = t9.*t12.*t16;
t216 = t196+t215-t220;
t221 = t11.*t216;
t217 = t195-t221;
t218 = t11.*t193;
t219 = t190+t218;
t222 = t11.*t72;
t240 = t5.*t193;
t223 = t222-t240;
t224 = t8.*t11.*t16;
t225 = t4.*t11.*t12.*t17;
t226 = t2.*t5.*t6.*t17;
t227 = t4.*t5.*t8.*t9.*t17;
t230 = t5.*t9.*t12.*t16;
t228 = t224+t225+t226+t227-t230;
t231 = t4.*t6.*t11;
t235 = t2.*t8.*t9.*t11;
t232 = t94+t231-t235;
t233 = t72.*t219.*(3.0./1.0e3);
t234 = t84.*t217.*(3.0./1.0e3);
t236 = t2.*t12.*t232.*(3.0./1.0e3);
t237 = t233+t234+t236;
t238 = t2.*t5.*t8.*t9;
t243 = t4.*t5.*t6;
t239 = t229+t238-t243;
t241 = t72.*t223.*(1.0./5.0e2);
t242 = t84.*t228.*(1.0./5.0e2);
t244 = t2.*t12.*t239.*(1.0./5.0e2);
t245 = t241+t242+t244;
t246 = t5.*t75;
t247 = t11.*t75;
t248 = t190+t247;
t249 = t70.*t248;
t250 = t5.*t86;
t299 = t11.*t84;
t251 = t250-t299;
t252 = t67.*t251;
t253 = t11.*t86;
t254 = t195+t253;
t255 = t70.*t254;
t256 = t89.*(t252+t255-t78.*t89);
t257 = t92+t96+t97;
t273 = t9.*t144.*(1.0./5.0e2);
t274 = t5.*t6.*t223.*(1.0./5.0e2);
t275 = t273+t274-t6.*t11.*t219.*(3.0./1.0e3);
t276 = t144.*t275;
t277 = t9.*t157.*(1.0./5.0e2);
t278 = t5.*t6.*t228.*(1.0./5.0e2);
t279 = t277+t278-t6.*t11.*t217.*(3.0./1.0e3);
t280 = t157.*t279;
t281 = t4.*(1.0./5.0e2);
t282 = t4.*t47.*(1.0./1.0e3);
t283 = t2.*t5.*t6.*t11.*t12.*(1.0./1.0e3);
t284 = t281+t282+t283-t4.*t46.*t47.*(1.0./1.0e3)-t2.*t6.*t8.*t9.*t47.*(1.0./1.0e3);
t285 = t28.*t284;
t286 = t276+t280+t285;
t287 = t217.*t237;
t288 = t228.*t245;
t289 = t287+t288;
t290 = t157.*t289;
t291 = t232.*t237;
t292 = t239.*t245;
t293 = t291+t292;
t294 = t219.*t237;
t295 = t223.*t245;
t296 = t294+t295;
t297 = t144.*t296;
t298 = t290+t297-t28.*t293;
t300 = t222-t246;
t301 = -t250+t299;
t302 = t80.*t300.*(1.0./5.0e2);
t303 = t89.*t301.*(1.0./5.0e2);
t304 = t28.*t91.*(1.0./5.0e2);
t305 = t302+t303+t304;
t306 = t80.*t248.*(3.0./1.0e3);
t307 = t89.*t254.*(3.0./1.0e3);
t316 = t28.*t95.*(3.0./1.0e3);
t308 = t306+t307-t316;
t309 = t28.^2;
t310 = t309.*(1.0./5.0e2);
t311 = t80.^2;
t312 = t311.*(1.0./5.0e2);
t313 = t89.^2;
t314 = t313.*(1.0./5.0e2);
t315 = t310+t312+t314;
M = reshape([t33.*(1.7e1./1.0e3)+t42.*(3.0./5.0e1)+t92+t96+t97+t28.*(t4.*t9.*4.0e-4+t2.*t6.*t8.*4.0e-4)+t10.*(t4.*t6.*9.0e-4-t2.*t8.*t9.*9.0e-4)+t33.*t48.*(1.0./2.5e2)+t40.*t41.*2.9e-4+t40.*t43.*2.9e-4+t41.*t42.*5.616e-2+t42.*t43.*5.616e-2+t12.^2.*t33.*5.8e-3+(t12.*t17.*(7.0./1.0e2)+t16.*t36.*(1.0./5.0e2)+t4.*t8.*t16.*(7.0./1.0e2)).*(t12.*t17.*(9.1e1./1.0e3)+t16.*t36.*2.6e-3+t4.*t8.*t16.*(9.1e1./1.0e3))+(t12.*t16.*(-7.0./1.0e2)+t17.*t36.*(1.0./5.0e2)+t4.*t8.*t17.*(7.0./1.0e2)).*(t12.*t16.*(-9.1e1./1.0e3)+t17.*t36.*2.6e-3+t4.*t8.*t17.*(9.1e1./1.0e3))+t4.^2.*(1.1e1./1.0e3)+t14.^2.*(2.1e1./5.0e2)+(t9.*t20.*1.65e-2+t16.*t24.*1.1e-3+t2.*t6.*t16.*1.65e-2).*(t25+t9.*t20.*(3.0./1.0e2)+t2.*t6.*t16.*(3.0./1.0e2))+(t9.*t30.*(-1.65e-2)+t17.*t24.*1.1e-3+t2.*t6.*t17.*1.65e-2).*(t32-t9.*t30.*(3.0./1.0e2)+t2.*t6.*t17.*(3.0./1.0e2))+(t6.*t20.*(-1.008e-3)+t16.*t24.*2.8e-5+t2.*t9.*t16.*1.008e-3).*(t25-t6.*t20.*(9.0./1.25e2)+t2.*t9.*t16.*(9.0./1.25e2))+(t6.*t30.*1.008e-3+t17.*t24.*2.8e-5+t2.*t9.*t17.*1.008e-3).*(t32+t6.*t30.*(9.0./1.25e2)+t2.*t9.*t17.*(9.0./1.25e2))+7.0./5.0e2,t100,t49+t50+t51+t52+t53+t54+t55+t56+t57+t58+t59+t60+t61+t62+t63+t64+t65-t110-t111-t112-t113-t114-t115-t116-t117-t118-t119-t120-t121-t122,t180,t210,t256-t28.*t257-t80.*(-t249+t78.*t80+t67.*t300),t100,t44.*(-9.6904512e-1)+t45.*1.174392e-1+t101+t102+t103+t104+t105+t106+t107+t108-t6.*t8.*1.4112e-4-t6.*t44.*8.91e-3+t8.*t45.*4.914e-2-t9.*t44.*5.4432e-4-t46.*t48.*7.7576e-5-t47.*t48.*(1.0./1.0e3)-t6.*t8.*t45.*5.4432e-4+t8.*t9.*t45.*8.91e-3-t46.*t47.*t48.*(1.0./1.0e3)+1.260876056,t126,t182,t212,t166,t49+t50+t51+t52+t53+t54+t55+t56+t57+t58+t59+t60+t61+t62+t63+t64+t65-t12.*t14.*t44.*1.2194e-2-t12.*t15.*t44.*6.37e-3-t12.*t15.*t45.*1.2194e-2-t6.*t12.*t14.*t45.*7.056e-5-t8.*t12.*t15.*t44.*8.842576e-3-t9.*t12.*t14.*t44.*4.983e-3-t9.*t12.*t15.*t44.*1.155e-3-t9.*t12.*t15.*t45.*4.983e-3-t5.*t9.*t11.*t15.*t44.*(1.0./1.0e3)-t8.*t12.*t14.*t45.*t46.*7.7576e-5-t8.*t12.*t14.*t45.*t47.*(1.0./1.0e3)-t5.*t9.*t11.*t14.*t45.*t48.*(1.0./5.0e2)-t8.*t12.*t14.*t45.*t46.*t47.*(1.0./1.0e3),t126,t46.*5.77576e-4+t48.*8.342576e-3+t101+t102+t103+t104-t127-t46.*t48.*5.77576e-4+t46.*t129.*5.0e-4-t48.*t129.*5.0e-4-t46.*t48.*t129.*5.0e-4+t8.*t9.*t12.*sin(t128).*(1.0./1.0e3)+8.31420456e-1,t130+t131-t183-t184-t185-t186,t132+t133+t134+t135+t136-t213-t214,t166,t180,t182,t130+t131-t6.*t12.*3.04416e-4-t6.*t9.*t12.*7.7576e-5-t5.*t6.*t8.*t11.*(1.0./1.0e3)-t6.*t9.*t12.*t47.*(1.0./1.0e3),t129.*2.5e-4-t188.*2.88788e-4-t129.*t188.*2.5e-4+1.2553788e-2,t5.*t6.*t11.*(-1.0./1.0e3),t286,t210,t212,t6.*(-7.056e-5)+t132+t133+t134+t135+t136-t8.*t47.*(1.0./1.0e3),t5.*t6.*t11.*(-1.0./1.0e3),t129.*(-5.0e-4)+3.867576e-3,t298,t256-t28.*t257+t80.*(t249-t78.*t80+t67.*(t246-t11.*t72)),t166,t166,t286,t298,t28.*(t28.*t315+t91.*t305-t95.*t308)+t80.*(t80.*t315+t248.*t308+t300.*t305)+t89.*(t89.*t315+t254.*t308+t301.*t305)],[6,6]);
