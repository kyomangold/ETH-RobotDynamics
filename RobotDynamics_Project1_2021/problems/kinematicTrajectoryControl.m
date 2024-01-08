function [ Dq ] = kinematicTrajectoryControl( q, p_des, w_des, params )  
    % Inputs:
    %  q             : current joint angles (3x1)
    %  p_des         : desired gripper pose (3x1)
    %  w_des         : desired gripper twist (3x1)
    %  params        : a struct of parameters

    % Output:
    %  Dq            : joint velocity command (3x1)
    
    % Choose a proportional controller gain
    K_p = 20; % initial: 5

    % Choose a pseudo_inverse damping coefficient
    lambda = 1e-2; % initial: 0.1
    
    % current gripper pose
    p_current_pos_yz = [jointTo2DGripperPosition solution(q, params); sum(q)];
    % commanded velocity
    wcmd = wdes + Kp * (pdes − pcur);
    % analytic Jacobian
    Ja = jointToGripperAnalyticalJacobian_solution(q, params);
    % pseudo−inverse of the analytic Jacobian
    Ja_pinv = pseudoInverseMat_solution(Ja, lambda);
    % desired joint velocities
    Dq = Japinv * wcmd;

    
end