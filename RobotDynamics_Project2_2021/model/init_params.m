function params = init_params()
% Initialize a struct containing the body lengths in meters.
params = struct;
params.l0 = 0.8;
params.l1 = 0.3;
params.l2 = 0.4;
params.l3 = 0.3;
params.l4 = 0.2;
params.alpha = 70 * pi/180;


%% Gains
params.kp_joint = diag([10.0 5.0 2.5]);
params.kd_joint =  diag([5.0 1.5 0.5]);

params.kp_task = diag([50.0 50.0 50.0 25.0 25.0 25.0]);
params.kd_task = diag([5.0 5.0 5.0 5.0 5.0 5.0]);

params.tau_max = 20.0;

%% Link properties
R = 0.05; % link radius
params.m = cell(3,1);
params.k_r_ks = cell(3,1); 
params.k_I_s = cell(3,1);

% link 1
params.m{1} = 4.5;
params.k_r_ks{1} = [0.0; 0.0; 0.5 * params.l2];
params.k_I_s{1} = diag([params.m{1}*params.l2^2/12.0, ...
                        params.m{1}*params.l2^2/12.0, ...
                        params.m{1}*R^2/2.0]);
% link2 
params.m{2} = 1.0;
params.k_r_ks{2} = [0.0; 0.0; 0.5 * params.l3];
params.k_I_s{2} = diag([params.m{2}*params.l3^2/12.0, ...
                        params.m{2}*params.l3^2/12.0, ...
                        params.m{2}*R^2/2.0]);

% link3 (ignore sensor weight)
params.m{3} = 1.0;
params.k_r_ks{3} = [0.0; 0.0; 0.5 *  params.l4];
params.k_I_s{3} = diag([params.m{3}* params.l4^2/12.0, ...
                        params.m{3}* params.l4^2/12.0, ...
                        params.m{3}*R^2/2.0]);
                    
%% Gravity
params.I_g_acc = [0; 0; -9.81];

%% Simulation params
params.control_dt = 0.01;       % Control and visualization steps
params.N_sim_decimation = 2;    % Amount simulation steps called between control updates

% Sampling time used for discrete integration steps
params.simulation_dt = params.control_dt / params.N_sim_decimation; 

end
