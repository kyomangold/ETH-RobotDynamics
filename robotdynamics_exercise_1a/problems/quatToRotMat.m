function R = quatToRotMat(quat)
  % Input: quaternion [w x y z]
  % Output: corresponding rotation matrix

  
  % extract real part (scalar) of quaternion
  quat_r = quat(1);
  % extract imaginary part (vector) of quaternion 
  quat_i = quat(2:4);
  
  % mapping unit quaternion to rotation matrix
  R = (2* quat_r^2 -1)*eye(3) + 2* quat_r * skewMatrix(quat_i) + 2*quat_i*quat_i';
  
end

function A = skewMatrix(q_n)
 A = [0, -q_n(3),  q_n(2);...
         q_n(3), 0, -q_n(1);...
        -q_n(2), q_n(1), 0];
end
