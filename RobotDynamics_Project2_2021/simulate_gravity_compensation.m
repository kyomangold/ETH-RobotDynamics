clear; clc; close all;

init_workspace

%% Setup
use_solution = 0; % Use solution (1) or user implementation (0)

% generalized coordinates
gc = generate_gc;

% Initialize the parameters for the mid-term exam.
params = init_params;

% Forward Kinematics
kin = generate_kin(gc.q, params);

% Forward Differential Kinematics
jac = generate_jac(gc, kin, params);

% Simulation
T_sim = 3.0;
N_sim = round(T_sim / params.control_dt);

%% Gravity Compensation
disp('Gravity Compensation...');
gc.q = [0.0; pi/2; pi/4];
gc.dq = [0.0; 0.0; 0.0];
tau = [0.0; 0.0; 0.0];

% reference
q_des = gc.q + [pi/5; +pi/10; -pi/10]; 
dq_des = [0.0; 0.0; 0.0];
target =  double(subs(kin.I_r_IG, {'q1' 'q2' 'q3'}, {q_des(1) q_des(2) q_des(3)}));

figure(1);

for sim_step = 1:N_sim

  
   %% control input
   if use_solution == 1
   	tau = Q3_gravity_compensation_solution(params, gc, q_des, dq_des);
   else
    tau = Q3_gravity_compensation(params, gc, q_des, dq_des); 
   end

   %% Visualize
   plot(target(2), target(3), 'ro', 'MarkerSize', 10);
   hold on; 
   gc =  draw_robot(gc, kin, params);
   drawnow()
   
   %% Simulator Loop
   for j = 1:params.N_sim_decimation
      [gc, ~] = Q2_forward_dynamics_solution(gc, tau, params); 
   end

   pause(params.control_dt);
   refresh
end

