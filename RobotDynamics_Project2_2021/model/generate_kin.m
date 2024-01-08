% generate kinematics
function kin = generate_kin(q, params)
%% Create kinematics container
kin = struct();

% homogeneous transformations from frame k to the inertial frame
kin.T_Ik = cell(3,1);

% rotation matrices from frame k to the inertial frame
kin.R_Ik = cell(3,1);

%% Homogeneous transformations

% kinematic params
l0 = params.l0;
l1 = params.l1;
l2 = params.l2;
l3 = params.l3;
l4 = params.l4;

% Joint positions
q1 = q(1);
q2 = q(2);
q3 = q(3);

% compute T_I0
p_I0_I = [0; 0; l0];
C_I0 = [0 1 0;
        1 0 0;
        0 0 -1];
T_I0 = [C_I0 p_I0_I;
        0 0 0 1];

% compute T_01
p_01_0 = [l1; 0; 0];
C_01 = [cos(q1) 0 sin(q1);
        0 1 0;
        -sin(q1) 0 cos(q1)];
T_01 = [C_01 p_01_0;
        0 0 0 1];

% compute T_12
p_12_1 = [0; 0; l2];
C_12 = [cos(q2), 0, sin(q2);
        0, 1, 0;
        -sin(q2), 0, cos(q2)];
T_12 = [C_12 p_12_1;
        0 0 0 1];

% compute T_23
p_23_2 = [0; 0; l3];
C_23 = [cos(q3), 0, sin(q3);
        0, 1, 0;
        -sin(q3), 0, cos(q3)];
T_23 = [C_23 p_23_2;
        0 0 0 1];

% compute T 3G
p_3G_3 = [0; 0; l4];
C_3G = eye(3,3);
T_3G = [C_3G p_3G_3;
        0 0 0 1];
        
% homogeneous transformations from frame k to frame I
kin.T_I0 = T_I0;
kin.T_Ik{1} = simplify(T_I0 * T_01);
kin.T_Ik{2} = simplify(kin.T_Ik{1}*T_12);
kin.T_Ik{3} = simplify(kin.T_Ik{2}*T_23);

% rotation matrices from frame k to frame I
kin.R_Ik{1} = kin.T_Ik{1}(1:3,1:3);
kin.R_Ik{2} = kin.T_Ik{2}(1:3,1:3);
kin.R_Ik{3} = kin.T_Ik{3}(1:3,1:3);

%% Endeffector
% end-effector homogeneous transformation and position
kin.T_IG = simplify(kin.T_Ik{3} * T_3G);
kin.R_IG =  kin.T_IG(1:3,1:3);
kin.I_r_IG = kin.T_IG(1:3,4);

% constant tranformations related to the gripper
lg = 0.06;
kin.T_GG1 = [eye(3,3), [lg;0;0];
             zeros(1,3), 1];
    
kin.T_GG2 = [eye(3,3), [lg;0;lg];
             zeros(1,3), 1];
         
kin.T_GG3 = [eye(3,3), [-lg;0;0];
             zeros(1,3), 1];
         
kin.T_GG4 = [eye(3,3), [-lg;0;lg];
             zeros(1,3), 1];


%% Matlab functions
% fname = mfilename;
% fpath = mfilename('fullpath');
% dpath = strrep(fpath, fname, '');
% 
% fprintf('Generating transformation file... ');
% matlabFunction(kin.T_IG, 'vars', {q}, 'file', strcat(dpath,'/T_IG_fun'));
% fprintf('done!\n')

end
