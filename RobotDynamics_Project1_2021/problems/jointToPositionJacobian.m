function [ J_03_p ] = jointToPositionJacobian(q, params)
  % q: a 3x1 vector of generalized coordinates
  % params: a struct of parameters
  
   % Link lengths (meters)
   l2 = params.l2;
   l3 = params.l3;
   % Joint positions
   q1 = q(1);
   q2 = q(2);
   % Implement your solution here...
   % joint versors
   n1_0 = [0; 1; 0]; n2 0 = [0; 1; 0];
   % compute position vector from P1 to P2 in frame 0
   p_12_1 = [0; 0; l2];
   C_01 = [cos(q1), 0, sin(q1);
           0, 1, 0;
           −sin(q1), 0, cos(q1)]; p120 = C01 * p121;
   % compute position vector from P2 to P3 in frame 0
   p_23_2 = [0; 0; l3];
   C_12 = [cos(q2), 0, sin(q2);
           0, 1, 0;
          −sin(q2), 0, cos(q2)]; 
   C_02 = C_01 * C_12;
   p_23_0 = C02 * p_23_2;
   % compute position vector from P1 to P3 in frame 0
   p_13_0 = p_12_0 + p_23_0;
   % linear Jacobian
   J_03_p = [cross(n1 0, p 13 0), cross(n2 0, p 23 0), zeros(3,1)];
  
end

