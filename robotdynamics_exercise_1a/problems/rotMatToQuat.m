function q = rotMatToQuat(R)
  % Input: rotation matrix
  % Output: corresponding quaternion [w x y z]
  
  q = 0.5*[sqrt(trace(R)+1);
           sign(R(3,2)-R(2,3))*sqrt(R(1,1)-R(2,2)-R(3,3)+1);
           sign(R(1,3)-R(3,1))*sqrt(R(2,2)-R(1,1)-R(3,3)+1);
           sign(R(2,1)-R(1,2))*sqrt(R(3,3)-R(1,1)-R(2,2)+1)];
end