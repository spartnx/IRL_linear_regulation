function X_dot = dynamics(t,X,A,B,Q,R,P)
    % inputs
    %  > t = time at which the dynamics are evaluated
    %  > X = system and controller states ((n+1)-by-1)
    %  > A = internal dynamics matrix (n-by-n)
    %  > B = input dynamisc matrix (n-by-1)
    %  > Q = LQR cost matrix on the system states (n-by-n)
    %  > R = LQR cost matrix on the control input (1-by-1)
    %  > P = estimate of the solution to Lyapunov Equation (n-by-n)

    % unpack the states
    x = X(1:end-1); % system states (n-by-1)
    V = X(end); % controller state (not used; 1-by-1)

    % control policy
    K = R\B'*P; % feedback gain associated with P (1-by-n)
    u = -K*x; % control input (1-by-1)

    % state derivatives
    x_dot = A*x + B*u; % system evolution (n-by-1)
    V_dot = x'*Q*x + u'*R*u; % cost evolution (1-by-1)
    X_dot = [x_dot; V_dot]; % ((n+1)-by-1)
end