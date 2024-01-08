function T23 = jointToTransform23(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 3 to frame 2. T_23
  
  if (length(q)>1)
      q = q(3);
  end
  T23 = [cos(q), 0, sin(q), 0; 0, 1, 0, 0; -sin(q), 0, cos(q), .270; 0, 0, 0, 1;];
end
