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
T_sim = 10.0;
N_sim = round(T_sim / params.control_dt);

%% Task Space Control
disp('Task Space Control...');
gc.q = [0.8; 1.2; 1.2];
gc.dq = [0.0; 0.0; 0.0];
tau = [0.0; 0.0; 0.0];

start =  double(subs(kin.I_r_IG, {'q1' 'q2' 'q3'}, {gc.q(1) gc.q(2) gc.q(3)}));

% reference trajectory
t = 0:params.control_dt:T_sim;
N = length(t);

radius = 0.2;
T_period = 1.0 * pi;
omega = 2*pi/T_period;
omega_t = omega * t;
cos_array = cos(omega_t);
sin_array = sin(omega_t);

x_ref   = 0.0;
y_ref   = ones(N, 1) * start(2);
z_ref   = radius * sin_array + start(3);
v_y_ref = zeros(N, 1);
v_z_ref = radius * omega * cos_array;
a_y_ref = zeros(N, 1);
a_z_ref = -radius * omega * omega * sin_array;

angle_ref = 0.2 + 0.2 * sin_array;
C_I0 = kin.T_I0(1:3, 1:3);

figure(1);

for sim_step = 1:N_sim
   %% reference
   I_r_IGd = [x_ref; y_ref(sim_step); z_ref(sim_step)];
   I_v_Gd = [x_ref; v_y_ref(sim_step); v_z_ref(sim_step)];
   I_a_Gd = [x_ref; a_y_ref(sim_step); a_z_ref(sim_step)];
   target_angle = angle_ref(sim_step);
   C_IG_des = C_I0 * eulAngXyzToRotMat([0.0; pi + target_angle; 0.0]);

   %% control input
   if use_solution == 1
   	[tau, dyn] = Q4_task_space_control_solution(params, gc, I_r_IGd, I_v_Gd, I_a_Gd, C_IG_des);
   else
    [tau, dyn] = Q4_task_space_control(params, gc, I_r_IGd, I_v_Gd, I_a_Gd, C_IG_des); 
   end

   %% Visualize
   plot(y_ref, z_ref, 'g','LineWidth', 1.5);
   hold on;
   plot(I_r_IGd(2), I_r_IGd(3), 'ro', 'MarkerSize', 10);
   plot([I_r_IGd(2)-0.1*cos(target_angle)  I_r_IGd(2)+0.1*cos(target_angle)],...
       [I_r_IGd(3)-0.1*sin(target_angle) I_r_IGd(3)+0.1*sin(target_angle)],...
       'r', 'MarkerSize', 10);

   gc =  draw_robot(gc, kin, params);
   drawnow()
   
   %% Simulator Loop
   for j = 1:params.N_sim_decimation
      [gc, ~] = Q2_forward_dynamics_solution(gc, tau, params); 
   end
   
   pause(params.control_dt);
   refresh
end

