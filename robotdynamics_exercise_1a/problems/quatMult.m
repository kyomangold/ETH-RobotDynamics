function q_AC = quatMult(q_AB,q_BC)
  % Input: two quaternions to be multiplied
  % Output: output of the multiplication
  
  q = q_AB;
  p = q_BC;
  
  q_w = q(1); q_n = q(2:4);
  p_w = p(1); p_n = p(2:4);

  q_AC = [q_w*p_w - q_n'*p_n;
             q_w*p_n + p_w*q_n + skewMatrix(q_n)*p_n];
end

function A = skewMatrix(q_n)
 A = [0, -q_n(3),  q_n(2);...
         q_n(3), 0, -q_n(1);...
        -q_n(2), q_n(1), 0];
end
