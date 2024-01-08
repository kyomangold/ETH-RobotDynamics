function [tau, dyn] = Q4_task_space_control( params, gc, I_r_IGd, I_v_Gd, I_a_Gd, C_IG_des)
% Task-space inverse dynamics controller tracking a desired end-effector motion
% with a PD stabilizing feedback terms.
%
% Inputs:
%   - params    : struct with parameters
%   - gc        : Current generalized coordinates (q, dq)
%   - I_r_IGd   : the desired position (3x1) of the gripper w.r.t. the inertial frame expressed in the inertial frame.
%   - I_v_Gd    : the desired linear velocity (3x1) of the gripper in the inertial frame.
%   - I_a_Gd    : the desired linear acceleration (3x1) of the gripper in the inertial frame.
%   - C_IG_des  : the desired orientation of the gripper as a rotation matrix (3x3)
% Output:
%   - tau       : computed control torque per joint (3x1)
%
%% Setup
q = gc.q;      % Generalized coordinates (3x1)
dq = gc.dq;    % Generalized velocities (3x1)

M = M_fun_solution(q); % Mass matrix
b = b_fun_solution(q, dq); % Nonlinear term
g = g_fun_solution(q); % Gravity term

% Find jacobians, positions and orientation based on the current
I_Jp_G = I_Jp_G_fun(q); % Positional Jacobian of end effector
I_Jr_G = I_Jr_G_fun(q); % Rotational Jacobian of end effector
I_dJp_G = I_dJp_G_fun(q, dq); % Time derivative of the position Jacobian of the end-effector (3x3)
I_dJr_G = I_dJr_G_fun(q, dq); % Time derivative of the Rotational Jacobian of the end-effector (3x3)

% Geometrical Jacobian
I_J_G = [I_Jp_G; I_Jr_G];
I_dJ_G = [I_dJp_G; I_dJr_G];

T_IG = T_IG_fun(q); % Homogeneous transformation from frame C to frame I
I_r_IG = T_IG(1:3, 4);


%% Project the joint-space dynamics to the task space
% Note: use pseudoInverseMat() function for lambda for stability 
JMinv = I_J_G / M;
 
% TODO: Implement end-effector dynamics
lambda = pseudoInverseMat(JMinv * I_J_G');
mu = lambda * (JMinv * b - I_dJ_G * dq);
p = lambda * JMinv * g;
 
%% Compute torque
% Note: desired angular velocity & acceleration are zero.
 
% Gains !!! Please do not modify these gains !!!
kp = params.kp_task; % P gain matrix for gripper position (6x6 diagonal matrix)
kd = params.kd_task; % D gain matrix for gripper velocity  (6x6 diagonal matrix)

C_err = C_IG_des * T IG(1:3, 1:3)';
orientation_error = rotMatToRotVec(C_err);

chi_err = [I_r_IGd − I_r_IG;
orientation error];
dchi_err = [I_v_Gd; zeros(3, 1)] − I_J_G * dq;

dw_e = [I_a_Gd; zeros(3,1)] + kp * chi_err + kd * dchi_err;

tau = I_J_G' * (lambda * dw_e + mu + p); %I_Fe_des = (lambda * dw_e + mu + p)

%% Return for evaluation
dyn.lambda =lambda;
dyn.mu = mu;
dyn.b = b;
end
