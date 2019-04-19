function C0 = construct_coeff_objective(P1, P2, is_normalize, weight)
% 
% P1: points in view 1, each column is a point
% P2: points in view 2, each column is a point
%
% Author: Ji Zhao
% date: 12/24/2018

if nargin < 3
    is_normalize = false;
end

N_obs = size(P1, 2);
if (N_obs==0) || (size(P2, 2)~=N_obs)
    retun;
end

if nargin < 4 || isempty(weight)
    weight = ones(N_obs, 1);
end

if 1
    % normalization, unit norm
    s1 = sqrt(sum(P1.^2, 1));
    P1 = bsxfun(@rdivide, P1, s1);
    s2 = sqrt(sum(P2.^2, 1));
    P2 = bsxfun(@rdivide, P2, s2);
else
    % normalization, z coordinate is 1
    P1 = bsxfun(@rdivide, P1, P1(3,:));
    P2 = bsxfun(@rdivide, P2, P2(3,:));
end

% number of unknowns
N_var = 12;
C0 = zeros(N_var, N_var);
C = zeros(9, 9);

%% C
for ii = 1:N_obs
    f = P1(:, ii);
    f_p = P2(:, ii);
    g = kron(f_p, f);
    C = C + weight(ii)*g*g';
end

%% normalize C
if is_normalize
    s = std(C(:));
    C = C / s;
end
C0(1:9, 1:9) = C;

