function gc_new = draw_robot(gc, kin, params)


% Compute kinematicss
T_I1 = eval(subs(kin.T_Ik{1}, {'q1' 'q2' 'q3'}, {gc.q(1) gc.q(2) gc.q(3)}));
T_I2 = eval(subs(kin.T_Ik{2}, {'q1' 'q2' 'q3'}, {gc.q(1) gc.q(2) gc.q(3)}));
T_I3 = eval(subs(kin.T_Ik{3}, {'q1' 'q2' 'q3'}, {gc.q(1) gc.q(2) gc.q(3)}));
T_IG = eval(subs(kin.T_IG, {'q1' 'q2' 'q3'}, {gc.q(1) gc.q(2) gc.q(3)}));
% T_IK_3 = eval(subs(kin.T_Ik{3}, {'q1' 'q2' 'q3'}, {gc.q(1) gc.q(2) gc.q(3)}));
% I_r_IG = eval(subs(kin.I_r_IG, {'q1' 'q2' 'q3'}, {gc.q(1) gc.q(2) gc.q(3)}));

% p_3Croot_3 = [params.l31; 0; 0];
% T_3Croot = [eye(3) p_3Croot_3;
%         0 0 0 1];
% T_ICroot = T_IK_3 * T_3Croot;   

I_p_I1 = T_I1(2:3,4);
I_p_I2 = T_I2(2:3,4);
I_p_I3 = T_I3(2:3,4);
I_p_IG = T_IG(2:3,4);
l0 = params.l0;
l1 = params.l1;


% compute additional positions to visualize gripper
T_IG1 = T_IG * kin.T_GG1;
I_p_IG1 = T_IG1(2:3, 4);

T_IG2 = T_IG * kin.T_GG2;
I_p_IG2 = T_IG2(2:3, 4);

T_IG3 = T_IG * kin.T_GG3;
I_p_IG3 = T_IG3(2:3, 4);

T_IG4 = T_IG * kin.T_GG4;
I_p_IG4 = T_IG4(2:3, 4);

plot([-l1 l1], [l0 l0], 'k--', ...
     [I_p_I1(1) I_p_I2(1)], [I_p_I1(2) I_p_I2(2)], 'b', ...
           [I_p_I2(1) I_p_I3(1)], [I_p_I2(2) I_p_I3(2)], 'b', ...
           [I_p_I3(1) I_p_IG(1)], [I_p_I3(2) I_p_IG(2)], 'b',...
           [I_p_IG(1) I_p_IG1(1)], [I_p_IG(2) I_p_IG1(2)], 'b', ...
           [I_p_IG1(1) I_p_IG2(1)], [I_p_IG1(2) I_p_IG2(2)], 'b', ...
           [I_p_IG(1) I_p_IG3(1)], [I_p_IG(2) I_p_IG3(2)], 'b', ...
           [I_p_IG3(1) I_p_IG4(1)], [I_p_IG3(2) I_p_IG4(2)], 'b','LineWidth', 1.5);

axis([-0.2, 1.1, 0.2, 1.4]);
% width = 3.0;
% height = 2.0;
% xlim([-width/2, width/2])
% ylim([-width/2, width/2])
hold off

gc_new = gc;
end