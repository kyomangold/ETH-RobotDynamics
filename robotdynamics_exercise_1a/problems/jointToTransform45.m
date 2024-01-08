function T45 = jointToTransform45(q)
  % Input: joint angles
  % Output: homogeneous transformation Matrix from frame 5 to frame 4. T_45
  
  if (length(q)>1)
      q = q(5);
  end
  T45 = [cos(q), 0, sin(q), .168; 0, 1, 0, 0; -sin(q), 0, cos(q), 0; 0, 0, 0, 1;];
end

