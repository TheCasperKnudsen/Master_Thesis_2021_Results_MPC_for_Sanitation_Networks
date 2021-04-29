% Using Brisons rule for tuning LQR
max_deviation_from_mean_x = 0.1;
max_allowed_u = 0.5/60;
Q = diag([1/(max_deviation_from_mean_x^2), 0.001*ones(1,8), 1/(max_deviation_from_mean_x^2)]);
R = diag([1/(max_allowed_u^2) 1/(max_allowed_u^2)]);

% Fetching system paramters from workspace vairable sys
A = full(sys.A(dT));
B = full(sys.B(dT));

% Find K
[K,S,e] = dlqr(A,B,Q,R);