function demo()

%rng('default')

%% Install dependencies
% direct linear transformation
addpath('C:/jizhao/npt-relative-pose/dlt');
% some functions from openGV
addpath('C:/jizhao/npt-relative-pose/helpers');
% install at least one of the solvers: CVX, SeDuMi, SDPT3, SDPA, or CSDP.
if ~exist('cvx_version', 'file')
  run('C:/jizhao/non-minimal/cvx-w64/cvx/cvx_setup.m');
end
if ~exist('sedumi', 'file')
    addpath('C:/jizhao/non-minimal/sedumi-master');
end
if ~exist('sdpt3', 'file')
    run('C:/jizhao/non-minimal/SDPT3-4.0/startup');
end
if ~exist('sdpam', 'file')
    addpath('C:/jizhao/non-minimal/sdpam7-windows');
end
if ~exist('csdp', 'file')
    addpath('C:/jizhao/non-minimal/csdp6.2.0win64/matlab');
    addpath('C:/jizhao/non-minimal/csdp6.2.0win64/bin');
end

%% parameters for data generation and solver
cam_number = 1;
n_obs = 100;
noise_level = 1;
outlier_fraction = 0;
% I recommend SDPT3 and SDPA
solvers = {'cvx-sedumi', 'cvx-sdpt3', 'cvx-mosek', 'sedumi', 'sdpt3', 'sdpa', 'sdpa-block', 'csdp'};
solver_name = solvers{6};

%% prepare synthetic data
% P1: points in view 1
% P2: points in view 2 
% Each column corresponds to a point expressed by normalized image coordinates.
% Optionally, it can be normalized to represent the bearing vector.
[P1,P2,T_gt,R_gt] = create2D2DExperiment(n_obs, cam_number, noise_level, outlier_fraction);
T_gt = T_gt / norm(T_gt(:));

%% our method, by QCQP and semidefinite relaxation for essential matrix
% A & b are independent of observations, so we caculate it only once
[A, b] = construct_coeff_constraint();
tic
[R_est, T_est] = npt_pose(P1, P2, A, b, solver_name);
toc

%% evaluation
R_gt, R_est
T_gt, T_est

