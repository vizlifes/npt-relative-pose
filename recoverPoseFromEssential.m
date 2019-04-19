function [R_true, T_true, R_false, T_false] = recoverPoseFromEssential(E, P1, P2)
%
% Author: Ji Zhao
% date: 12/24/2018

%% pick the physically realizable pose
if 1
    [R1, R2, T1, T2]  = PoseEMat(E);
else
    [R1, R2, T1, T2]  = decomposeEssentialMatrix(E);
end

n = size(P1, 2);
X = zeros(n, 9);
for ii = 1:n
    X(ii, :) = kron(P2(:,ii)', P1(:,ii)');
end
M_r1 = X*reshape(E*E'*R1, [9 1]);
M_r2 = X*reshape(E*E'*R2, [9 1]);
n1 = numel(find(M_r1>0));
n2 = numel(find(M_r2>0));
if n1 >= n2
    R_true = R1;
    R_false = R2;
else
    R_true = R2;
    R_false = R1;
end

X = zeros(n, 6);
for ii = 1:n
    X(ii, :) = [norm(P1(:,ii))*P2(:,ii)', norm(P2(:,ii))*P1(:,ii)'];
end
M_t1 = X * [-R_true'; eye(3)] * T1;
M_t2 = X * [-R_true'; eye(3)] * T2;
n1 = numel(find(M_t1>0));
n2 = numel(find(M_t2>0));
if n1 >= n2
    T_true = T1;
    T_false = T2;
else
    T_true = T2;
    T_false = T1;
end
