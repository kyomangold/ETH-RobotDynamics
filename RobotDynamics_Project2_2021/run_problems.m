% Initialize the workspace.
init_workspace

%% Setup
% generalized coordinates
gc = generate_gc();
q_eval = [0.1, 0.2, 0.3]';
dq_eval = [0.4, 0.5, 0.6]';

% Initialize the parameters for the mid-term exam.
params = init_params();

% Forward Kinematics
kin = generate_kin(gc.q, params);

% Forward Differential Kinematics
jac = generate_jac(gc, kin, params);

%% Q1.
disp('==============');
disp('Running Q1 ...');

try
  eom = Q1_generate_eom(gc, kin, params, jac);
  disp('eom generated.');
  
  disp('Running eom.M ...');
  M = eval(subs(eom.M, gc.q, q_eval));
  disp('Running eom.g ...');
  g = eval(subs(eom.g, gc.q, q_eval));
  disp('Running eom.b ...');
  b = eval(subs(eom.b, [gc.q, gc.dq], [q_eval, dq_eval]));
  
  disp('Done.');
  
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

%% Q2.
disp('==============');
disp('Running Q2 ...');

try
  gc_eval.q = q_eval;
  gc_eval.dq = dq_eval;
  tau_eval = ones(3, 1);
  user_tau = Q2_forward_dynamics(gc_eval, tau_eval, params);
  
  disp('Done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end


%% Q3.
disp('==============');
disp('Running Q3 ...');

try
  gc_eval.q = q_eval;
  gc_eval.dq = dq_eval;
  q_des = q_eval / 2.0;
  dq_des = dq_eval / 2.0;
  user_tau = Q3_gravity_compensation(params, gc_eval, q_des, dq_des);
  
  disp('Done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end


%% Q4.
disp('==============');
disp('Running Q4 ...');

try
  gc_eval.q = q_eval;
  gc_eval.dq = dq_eval;
  q_des = q_eval / 2.0;
  dq_des = dq_eval / 2.0;
  T_IGd = T_IG_fun(q_des);
  I_Jp_G = I_Jp_G_fun(q_des);
  I_r_IGd = T_IGd(1:3, 4);
  I_v_Gd = I_Jp_G * dq_des;
  I_a_Gd = [0.01, 0.02, 0.03]';
  
  tau = Q4_task_space_control(params, gc_eval, I_r_IGd, I_v_Gd, I_a_Gd, 0.0);
  
  disp('Done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

