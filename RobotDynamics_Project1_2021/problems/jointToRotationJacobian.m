function [ J_I2_r ] = jointToRotationJacobian(q, params)
  % q: a 3x1 vector of generalized coordinates
  % params: a struct of parameters

  % joint versors
  n1_I = [1; 0; 0]; 
  n2_I = [1; 0; 0];

  % rotation Jacobian
  J_I2_r = [n1 I, n2 I, zeros(3,1)]; 

  
 end