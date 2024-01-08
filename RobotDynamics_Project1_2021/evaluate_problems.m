% Initialize the workspace.
init_workspace;
init_params;

%% Exercise 1.
disp('Running exercise 1...');
try
  T_IS = jointToSensorPose(q, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end


%% Exercise 2.
disp('Running exercise 2...');
try
  Jp = jointToPositionJacobian(q, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

%% Exercise 3.
disp('Running exercise 3...');
try
  Jr = jointToRotationJacobian(q, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

%% Exercise 4.
disp('Running exercise 4...');
try
  q0 = [0.5968; 1.0816; 0.8688];
  p_des = [0.0; 0.0; 0.0];
  w_des = [0.0; 0.0; 0.0];
  dq_des = kinematicTrajectoryControl(q0, p_des, w_des, params);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

%% Exercise 5.
disp('Running exercise 5...');
try
  thetaZ = 0.0;
  thetaY = 0.0;
  thetaX = 0.0;
  I_p_IC = zeros(3,1);
  T_IG = eye(4,4);
  T_CG = gripperToCameraPose(thetaZ, thetaY, thetaX, I_p_IC, T_IG);
  disp('done.');
catch ME
  disp('Please check your implementation!');
  disp('The following errors occured:');
  disp(ME.message);
  disp(ME.stack(1));
end

