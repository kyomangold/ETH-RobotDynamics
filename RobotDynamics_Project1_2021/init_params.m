% Initialize a struct containing the body lengths in meters.
params = struct;
params.l0 = 0.8;
params.l1 = 0.4;
params.l2 = 0.3;
params.l3 = 0.2;
params.l41 = 0.2;
params.l42 = 0.05;
params.l5 = 0.25;
params.l6 = 0.15;
params.alpha = 70 * pi/180;

% Initialize a random vector of joint positions.
q = rand(3,1);