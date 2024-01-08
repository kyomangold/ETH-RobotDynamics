function quat = jointToQuat(q)
  % Input: joint angles
  % Output: quaternion representing the orientation of the end-effector 
  % q_IE.
  C = jointToRotMat(q);
  quat = rotMatToQuat(C);
end

