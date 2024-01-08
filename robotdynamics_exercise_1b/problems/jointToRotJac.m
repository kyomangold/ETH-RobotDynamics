function J_R = jointToRotJac(q)
  % Input: vector of generalized coordinates (joint angles)
  % Output: Jacobian of the end-effector orientation which maps joint
  % velocities to end-effector angular velocities in I frame.

  
  % compute relative homogeneous transformation matrix
  T_I0 = getTransformI0();
  T_01 = jointToTransform01(q(1));
  T_12 = jointToTransform12(q(2));
  T_23 = jointToTransform23(q(3));
  T_34 = jointToTransform34(q(4));
  T_45 = jointToTransform45(q(5));
  T_56 = jointToTransform56(q(6));
 
  % compute homogeneous transformation matrices from frame K to inertial
  % frame I
  T_I1 = T_I0 * T_01;
  T_I2 = T_I1 * T_12;
  T_I3 = T_I2 * T_23;
  T_I4 = T_I3 * T_34;
  T_I5 = T_I4 * T_45;
  T_I6 = T_I5 * T_56;
  
  % extract rotation matrices from homogeneous transformation matrix
  R_I1 = T_I1(1:3, 1:3);
  R_I2 = T_I2(1:3, 1:3);
  R_I3 = T_I3(1:3, 1:3);
  R_I4 = T_I4(1:3, 1:3);
  R_I5 = T_I5(1:3, 1:3);
  R_I6 = T_I6(1:3, 1:3);
  
  % extract position vector from homogeneous transformation matrix
  r_I_I1 = T_I1(1:3, 4);
  r_I_I2 = T_I2(1:3, 4);
  r_I_I3 = T_I3(1:3, 4);
  r_I_I4 = T_I4(1:3, 4);
  r_I_I5 = T_I5(1:3, 4);
  r_I_I6 = T_I6(1:3, 4);
  
  % compute EE position vector
  r_I_IE = jointToPosition(q);
  
  % define unit vectors with direction of axis of rotation
  n_1 = [0 0 1]';
  n_2 = [0 1 0]';
  n_3 = [0 1 0]';
  n_4 = [1 0 0]';
  n_5 = [0 1 0]';
  n_6 = [1 0 0]';
    
  % Compute the rotational jacobian.
  J_R = [  R_I1 * n_1 ...
           R_I2 * n_2 ...
           R_I3 * n_3 ...
           R_I4 * n_4 ...
           R_I5 * n_5 ...
           R_I6 * n_6 ...
        ];

end
