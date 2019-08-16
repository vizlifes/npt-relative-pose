function [R_true, T_true, C] = npt_pose(P1, P2, A, b, solver_str, weight)
%
% Author: Ji Zhao
% date: 12/24/2018

if nargin < 5
    solver_str = 'cvx-sedumi';
end
if nargin < 6
    weight = [];
end

C = construct_coeff_objective(P1, P2, false, weight);

%% optimization by CVX
if strcmpi(solver_str(1:3), 'cvx')
    n = size(C, 1); 
    cvx_precision('best');
    cvx_precision('default');
    cvx_quiet(true)
    if strcmpi(solver_str, 'cvx-sedumi')
        cvx_solver sedumi
    end
    if strcmpi(solver_str, 'cvx-sdpt3')
        cvx_solver sdpt3
    end
    if strcmpi(solver_str, 'cvx-mosek')
        cvx_solver mosek
    end
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

%% by SeDuMi
if strcmpi(solver_str, 'sedumi')
    m = size(A, 3);
    n = size(A, 1);
    At = zeros(n^2, m);
    for ii = 1:m
        tmp = A(:, :, ii);
        At(:, ii) = tmp(:);
    end
    c = C(:);
    K.s = size(C,1);

    opts = [];
    opts.fid = 0; % runs quietly, i.e., there is no screen output.
    opts.eps = 0;
    [x,y,out] = sedumi(At, b, c, K, opts);
    X = reshape(x, [n,n]);
end

%% by SDPT3
if strcmpi(solver_str, 'sdpt3')
    m = size(A, 3);
    n = size(A, 1);
    AA = zeros(n^2, m);
    for ii = 1:m
        tmp = A(:, :, ii);
        AA(:, ii) = tmp(:);
    end
    c = C(:);
    K.s = size(C,1);
    [blk,At,C2,b] = read_sedumi(AA, b, c, K);

    % line 362, cvx_sdpt3
    opts = [];
    opts.printlevel = 0; % runs quietly, i.e., there is no screen output.
    opts.gaptol = 0;
    opts.inftol = 1e-8;
    opts.steptol = 1e-6;
    opts.stoplevel = 1;
    [obj, X, y, Z] = sdpt3(blk, At, C2, b, opts);
    X = X{1};
end

%% by SDPA
if strcmpi(solver_str, 'sdpa')
    mDIM = numel(b);
    nBLOCK = 1;
    bLOCKsTRUCT = size(A, 1);
    c = -b;
    m = size(A, 3);
    F = cell(1, m+1);
    F{1, 1} = -sparse(C);
    for ii = 1:m
        F{1, ii+1} = -sparse(A(:, :, ii));
    end
    [objVal, x, X, Y, INFO] = sdpam(mDIM, nBLOCK, bLOCKsTRUCT, c, F);
    X = Y{1};
end

%% by SDPA with block matrices
if strcmpi(solver_str, 'sdpa-block')
    mDIM = numel(b);
    nBLOCK = 2;
    bLOCKsTRUCT = [9, 3];
    c = -b;
    m = size(A, 3);
    F = cell(2, m+1);
    F{1, 1} = -sparse(C(1:9, 1:9));
    for ii = 1:m
        F{1, ii+1} = -sparse(A(1:9, 1:9, ii));
        F{2, ii+1} = -sparse(A(10:12, 10:12, ii));
    end
    [objVal, x, X, Y, INFO] = sdpam(mDIM, nBLOCK, bLOCKsTRUCT, c, F);
    X = Y{1};
end

%% by CSDP
if strcmpi(solver_str, 'csdp')
    m = size(A, 3);
    n = size(A, 1);
    At = zeros(n^2, m);
    for ii = 1:m
        tmp = A(:, :, ii);
        At(:, ii) = tmp(:);
    end
    c = C(:);
    K.s = size(C,1);
    
    pars.objtol = 1.0e-9;
    [x, y, z, info] = csdp(At, b, c, K, pars);
    if (numel(x) == n^2)
        X = reshape(x, [n, n]);
    else
        return;
    end
end

%% pick the right solution
if issparse(X)
    X = full(X);
end
[v, d_max] = eigs(X(1:9, 1:9), 1);
E = reshape(v, [3 3]);
[R_true, T_true] = recoverPoseFromEssential(E, P1, P2);
