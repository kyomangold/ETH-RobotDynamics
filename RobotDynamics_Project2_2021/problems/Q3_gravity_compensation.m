function [ tau ] = Q3_gravity_compensation(params, gc, q_des, dq_des)
% Joint space PD controller with gravity compensation.
%
% Inputs:
%   - params    : struct with parameters
%   - gc        : Current generalized coordinates (q, dq)
%   - q_des     : the desired joint positions (3x1)
%   - dq_des    : the desired joint velocities (3x1)
% Output:
%   - tau       : computed control torque per joint (3x1)
%
%% Setup
q = gc.q;      % Generalized coordinates (3x1)
dq = gc.dq;    % Generalized velocities (3x1)

M = M_fun_solution(q);% Mass matrix
b = b_fun_solution(q, dq); % Nonlinear term
g = g_fun_solution(q); % Gravity term

% Gains !!! Please do not modify these gains !!!
kp = params.kp_joint; % P gain matrix for joints (3x3 diagonal matrix)
kd = params.kd_joint; % D gain matrix for joints (3x3 diagonal matrix)

% The control action has a gravity compensation term, as well as a PD
% feedback action which depends on the current state and the desired
% configuration.

%% Compute torque
tau = kp * (q_des - q) + kd * (dq_des - dq) + g;

end
