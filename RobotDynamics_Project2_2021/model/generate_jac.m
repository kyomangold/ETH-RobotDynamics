% generate jacobians
function jac = generate_jac(gc, kin, params)
% By calling:
%   jac = generate_jac(gen_cor, kin, dyn)
% a struct 'jac' is returned that contains the translation and rotation
% jacobians of the center of masses

%% Setup
q = gc.q;
dq = gc.dq;

T_Ik = kin.T_Ik;
R_Ik = kin.R_Ik;
I_r_IG = kin.I_r_IG;
T_IG = kin.T_IG;

%relative positions
k_r_ks = params.k_r_ks;

%% Compute link jacobians
I_Jp = cell(3,1); % gc dim
I_Jr = cell(3,1);

for k= 1:3
    % create containers
    I_Jp{k} = sym(zeros(3,3));
    I_Jr{k} = sym(zeros(3,3));
    
    % translational jacobian at the center of gravity s in frame I
    I_r_ks = [eye(3) zeros(3,1)]*T_Ik{k}*[k_r_ks{k};1]; % COM location in I
    I_Jp{k} = jacobian(I_r_ks, q);
    
    % rotational jacobian in frame I (_{I}n_{k})
    if k == 1
        I_Jr{k}(1:3,1) = R_Ik{1} * [0; 1; 0];
    else
        % copy columns of k-1 jacobian
        I_Jr{k} = I_Jr{k-1};
        
        % evaluate new column
        I_Jr{k}(1:3,k) = R_Ik{k} * [0; 1; 0];
    end
    
    % simplify expressions
    I_Jp{k} = simplify(I_Jp{k});
    I_Jr{k} = simplify(I_Jr{k});
end

% Compute the gripper jacobians in frame I
I_Jp_G = simplify(jacobian(I_r_IG, q));
I_Jr_G = I_Jr{3};

% Compute the time derivative of the gripper Jacobians
I_dJp_G = simplify(dAdt(I_Jp_G,q,dq));
I_dJr_G = simplify(dAdt(I_Jr_G,q,dq));


%% Generate function files from symbolic expressions
% fname = mfilename;
% fpath = mfilename('fullpath');
% dpath = strrep(fpath, fname, '');
% 
% fprintf('Generating Jacobian files... ');
% matlabFunction(I_Jp_G, 'vars', {q,dq}, 'file', strcat(dpath,'/I_Jp_G_fun'));
% matlabFunction(I_dJp_G, 'vars', {q,dq}, 'file', strcat(dpath,'/I_dJp_G_fun'));
% 
% matlabFunction(I_Jr_G, 'vars', {q,dq}, 'file', strcat(dpath,'/I_Jr_G_fun'));
% matlabFunction(I_dJr_G, 'vars', {q,dq}, 'file', strcat(dpath,'/I_dJr_G_fun'));
% % 
% fprintf('done!\n')

%% Store jacobians in output struct
jac.I_Jp = I_Jp;
jac.I_Jr = I_Jr;
jac.I_Jp_G = I_Jp_G;
jac.I_Jr_G = I_Jr_G;
jac.I_dJp_G = I_dJp_G;
jac.I_dJr_G = I_dJr_G;

end
