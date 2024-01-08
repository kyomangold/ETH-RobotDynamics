function [ T_IS ] = jointToSensorPose( q, params )
  % q: a 3x1 vector of generalized coordinates
  % params: a struct of parameters
    
  % Link lengths (meters)
  l0 = params.l0;
  l1 = params.l1;
  l2 = params.l2;
  l3 = params.l3;
  l41 = params.l41;
  l42 = params.l42;
  l5 = params.l5;
  l6 = params.l6;
  alpha = params.alpha;
  
  % Joint positions
  q1 = q(1);
  q2 = q(2);
  q3 = q(3);

  % Computing the sensor pose
%   % Approach 1
%   T_I0 = [0,   1,     0,        0; 
%          1,   0,     0,         0;
%          0,   0,     -1, params.l0;
%          0,   0,     0,         1];
     
%   T_01 = [cos(q(1)),   0,     sin(q(1)),   params.l1; 
%          0,   1,     0,         0;
%          -sin(q(1)),   0,     cos(q(1)), 0;
%          0,   0,     0,         1];
         
%   T_12 = [cos(q(2)),   0,     sin(q(2)),       0; 
%          0,   1,     0,         0;
%          -sin(q(2)),   0,     cos(q(2)),  params.l2;
%          0,   0,     0,         1];
     
%   T_23 = [cos(q(3)),   0,     sin(q(3)),       0; 
%          0,   1,     0,         0;
%          -sin(q(3)),   0,     cos(q(3)),  params.l3;
%          0,   0,     0,         1];
         
%   T_3P = [1,   0,     0,         0; 
%           0,   1,     0,         0;
%           0,   0,     1,  params.l41;
%           0,   0,     0,         1];
         
%   T_PQ = [cos(params.alpha),0,  -sin(params.alpha), params.l5; 
%            0, 1, 0, 0;
%             -sin(params.alpha), 0, cos(params.alpha), 0;
%            0, 0, 0, 1];
%   T_QS = [1,   0,     0,         0; 
%            0,   1,     0,         0;
%            0,   0,     1,  params.l6;
%            0,   0,     0,         1];
         
%   T_IS = T_I0 * T_01 * T_12 * T_23 * T_3P * T_PQ * T_QS; 

   % Approach 2
p_I0_I = [0; 0; l0]; 

C_I0 = [0 1 0;
       1 0 0;
       0 0 −1];

T_I0 = [C_I0 p_I0_I;
       zeros(1,3), 1];

p_01_0 = [l1;0; 0];


C_01 = [cos(q1), 0, sin(q1);
       0,1,0;
       −sin(q1), 0, cos(q1)];

T_01 = [C_01 p_01_0;
       zeros(1,3), 1];

p_12_1 = [0; 0; l2];

C_12 = [cos(q2), 0, sin(q2);
       0, 1, 0;
       −sin(q2), 0, cos(q2)]; 

T_12 = [C_12, p_12_1;
          zeros(1,3), 1];

p_23_2 = [0; 0; l3];

C_23 = [cos(q3), 0, sin(q3);
       0, 1, 0;
       −sin(q3), 0, cos(q3)]; 

T_23 = [C_23, p_23_2;
          zeros(1,3), 1];

p_3S_3 = [l5−l6*cos(alpha); 0; l41+l6*sin(alpha)]; 

C_3S = [cos(alpha−pi/2), 0, sin(alpha−pi/2);
       0, 1, 0;
        −sin(alpha−pi/2), 0, cos(alpha−pi/2)];

T_3S = [C_3S, p_3S_3; zeros(1,3), 1];

T_IS = T_I0 * T_01 * T_12 * T_23 * T_3S;
end