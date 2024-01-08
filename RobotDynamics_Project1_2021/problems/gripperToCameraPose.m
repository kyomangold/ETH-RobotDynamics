function [ T_CG ] = gripperToCameraPose(thetaZ, thetaY, thetaX, I_p_IC, T_IG)
  % thetaZ, thetaY, thetaX: ZYX Euler angles
  % I_p_IC : 3 x 1 camera position vector, expressed in the inertial frame
  % T_IG: 4 x 4 homogeneous transformation matrix of G with respect to I

  % C_IG = [T_IG(1:3,1:3)];
  % z = atan2(T_IG(2,1), T_IG(1,1));
  % y = atan2(-T_IG(3,1), sqrt((T_IG(3,2)*T_IG(3,2)) + (T_IG(3,3)*T_IG(3,3))));
  % x = atan2(T_IG(3,2),T_IG(3,3));
  
  % C_CG = [cos(thetaY + thetaZ) cos(thetaZ)*sin(thetaX+thetaY)-cos(thetaX)*sin(thetaZ) sin(thetaX+thetaZ)+cos(thetaX+thetaZ)*sin(thetaY);
  %         cos(thetaY)*sin(thetaZ) cos(thetaX+thetaZ)+sin(thetaX+thetaY+thetaZ) cos(thetaX)*sin(thetaY+thetaZ)-cos(thetaZ)*sin(thetaX);
  %         -sin(thetaY) cos(thetaY)*sin(thetaX) cos(thetaX+thetaY)];
  
    Cz = [cos(thetaZ), −sin(thetaZ), 0;
          sin(thetaZ), cos(thetaZ), 0;
          0, 0, 1];
    Cy = [cos(thetaY), 0, sin(thetaY);
          0, 1, 0;
          −sin(thetaY), 0, cos(thetaY)]; 
    
    Cx=[1,0,0;
        0, cos(thetaX), −sin(thetaX);
        0, sin(thetaX), cos(thetaX)]; 
  CIC=Cz*Cy*Cx;

  TIC=[CIC,IpIC; 
      zeros(1,3), 1];

  TCG = inv(TIC) * TIG;

end