function [A, b] = construct_coeff_constraint()
% 
% Author: Ji Zhao
% date: 12/24/2018

% number of unknowns
n = 12;
% number of equations
m = 7;
% trace(A_i * X) = b_i
A = zeros(n, n, m);
b = zeros(m, 1);

e11_idx = 1; e21_idx = 2; e31_idx = 3;
e12_idx = 4; e22_idx = 5; e32_idx = 6;
e13_idx = 7; e23_idx = 8; e33_idx = 9;
t1_idx = 10; t2_idx = 11; t3_idx = 12;

%% t^T * E = 0; E*E^T = [t]x * [t]x^T
t = zeros(n, n); t(e11_idx, e11_idx) = 1; t(e12_idx, e12_idx) = 1; t(e13_idx, e13_idx) = 1; t(t3_idx, t3_idx) = -1; t(t2_idx, t2_idx) = -1; A(:,:, 1) = t;
t = zeros(n, n); t(e21_idx, e21_idx) = 1; t(e22_idx, e22_idx) = 1; t(e23_idx, e23_idx) = 1; t(t3_idx, t3_idx) = -1; t(t1_idx, t1_idx) = -1; A(:,:, 2) = t;
t = zeros(n, n); t(e31_idx, e31_idx) = 1; t(e32_idx, e32_idx) = 1; t(e33_idx, e33_idx) = 1; t(t2_idx, t2_idx) = -1; t(t1_idx, t1_idx) = -1; A(:,:, 3) = t;
t = zeros(n, n); t(e11_idx, e21_idx) = 1; t(e12_idx, e22_idx) = 1; t(e13_idx, e23_idx) = 1; t(t2_idx, t1_idx) = 1; A(:,:, 4) = (t+t')/2;
t = zeros(n, n); t(e11_idx, e31_idx) = 1; t(e12_idx, e32_idx) = 1; t(e13_idx, e33_idx) = 1; t(t3_idx, t1_idx) = 1; A(:,:, 5) = (t+t')/2;
t = zeros(n, n); t(e21_idx, e31_idx) = 1; t(e22_idx, e32_idx) = 1; t(e23_idx, e33_idx) = 1; t(t3_idx, t2_idx) = 1; A(:,:, 6) = (t+t')/2;

%% translation vector
t = zeros(n, n); t(t1_idx, t1_idx) = 1; t(t2_idx, t2_idx) = 1; t(t3_idx, t3_idx) = 1; A(:,:, 7) = t; b(7) = 1;
