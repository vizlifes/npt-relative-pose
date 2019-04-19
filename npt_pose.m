function [R_true, T_true, C] = npt_pose(P1, P2, A, b, solver_str, weight)
%
% Author: Ji Zhao
% date: 12/24/2018

if nargin < 5
    solver_str = 'cvx';
end
if nargin < 6
    weight = [];
end

C = construct_coeff_objective(P1, P2, false, weight);

%% optimization by CVX
if strcmpi(solver_str, 'cvx')
    n = size(C, 1); 
%    cvx_precision('best');
    cvx_precision('default');
    cvx_quiet(true)
    cvx_solver sedumi
%    cvx_solver sdpt3
%    cvx_solver mosek
    cvx_begin
        variable X(n,n) symmetric
        minimize(trace(C*X));
        subject to
          X == semidefinite(n);
          for i = 1:size(A, 3)
              trace(A(:,:,i)*X) == b(i);
          end
    cvx_end
end

%% use SeDuMi directly
if strcmpi(solver_str, 'sedumi')
    m = size(A, 3);
    n = size(A, 1);
    At = zeros(n^2, m);
    for ii = 1:m
        tmp = A(:, :, ii);
        At(:, ii) = tmp(:);
    end
    bt = b;
    ct = C(:);
    K.s = size(C,1);

    opts = [];
    opts.fid = 0; % runs quietly, i.e., there is no screen output.
    opts.eps = 0;
    [x,y,out] = sedumi(At, bt, ct, K, opts);
    X = reshape(x, [n,n]);
end

%% use SDPT3 directly
if strcmpi(solver_str, 'sdpt3')
    m = size(A, 3);
    n = size(A, 1);
    AA = zeros(n^2, m);
    for ii = 1:m
        tmp = A(:, :, ii);
        AA(:, ii) = tmp(:);
    end
    bt = b;
    ct = C(:);
    K.s = size(C,1);
    [blk,At,C2,b] = read_sedumi(AA, bt, ct, K);

    % line 362, cvx_sdpt3
    opts = [];
    opts.printlevel = 0; % runs quietly, i.e., there is no screen output.
    opts.gaptol = 0;
    opts.inftol = 1e-8;
    opts.steptol = 1e-6;
    opts.stoplevel = 1;
    [obj,X,y,Z] = sdpt3(blk, At, C2, b, opts);
    X = X{1};
end

%% pick the right solution
if issparse(X)
    X = full(X);
end
[v, d_max] = eigs(X(1:9, 1:9), 1);
E = reshape(v, [3 3]);
[R_true, T_true] = recoverPoseFromEssential(E, P1, P2);
