function [gc_next, ddq] = Q2_forward_dynamics(gc, tau, params)
% Compute generalized acceleration of the system
%
% Inputs:
%   - tau       : Current joint torques (3x1)
%   - gc        : Current generalized coordinates (q, dq)
%   - params    : Struct with parameters
%   
% Output:
%   - gc_next   : Forward simulated generalized coordinates (q, dq)
%   - ddq       : Generalized accelerations (3x1) for the current simulation step.
%

if any(isnan(gc.q))
    error('q contains NaN');
end
if any(isnan(gc.dq))
    error('dq contains NaN');
end
if any(isnan(tau))
    error('tau contains NaN');
end

%% Setup
q = gc.q;      % Generalized coordinates (3x1)
dq = gc.dq;    % Generalized velocities (3x1)

tau = max(-params.tau_max, min(tau, params.tau_max)); % Joint torques within limits

M = M_fun_solution(q); 		% Mass matrix
b = b_fun_solution(q, dq); 	% Nonlinear term
g = g_fun_solution(q); 		% Gravity term

%% Compute generalized acceleration of the system

ddq = M \ (tau - (b + g));

%% Integration !!NOTE: Do not modify this part.!!
gc_next.dq = (dq + ddq * params.simulation_dt);
gc_next.q = (q + (dq + gc_next.dq) * 0.5 * params.simulation_dt);

end
