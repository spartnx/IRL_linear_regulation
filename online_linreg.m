function [P] = online_linreg(dyn_inputs, algo_inputs)
% This function implements the adaptive optimal control algorithm designed 
% by Vrabie et al. in "Adaptive Optimal Control for Continuous-Time Linear
% Systems Based on Policy Iteration" (2007).

%% Dynamics-related inputs
% sequence of true internal dynamics matrices
Aseq = dyn_inputs{1}; % {A1, N2, A2, N3, A3, ...}

% input dynamics matrix
B = dyn_inputs{2};

% LQR cost matrices
Q = dyn_inputs{3};
R = dyn_inputs{4};

% internal dynamics matrix (estimated, used to derive inital stabilizing
% control policy)
Anom = dyn_inputs{5};

%% Algorithm-related inputs
solve_initial_lqr = algo_inputs{1}; % use initial stabilizing control input as solution of the LQR with Anom?
x0 = algo_inputs{2}; % initial states 
T = algo_inputs{3}; % sampling time (sec)
tf = algo_inputs{4}; % time horizon, over which the system is simulated (sec)
Nupd = algo_inputs{5}; % number of samples to collect before updating the critic weights
eps = algo_inputs{6}; % threshold for algorithm convergence

%% Algorithm
% initial stabilizing solution
n = size(B, 1);
if solve_initial_lqr == true
    % use solution to LQR applied to Anom
    [P] = care(Anom,B,Q,R);
else
    % if the (unknown) system A to regulate is known to be stable
    P = zeros(n);
end

% convert time inputs into samples
Nf = floor(tf/T);

% simulation initialization
p = n*(n+1)/2;
V0 = 0; % initial state of the dynamical controller
j = 0; % counter to know when to next update critic weights
Z = zeros(p,Nupd); % stores xbar(t) - xbar(t+T) at each t=0,T,2T,...
Y = zeros(Nupd, 1); % stores V(t+T) - V(t) at each t=0,T,2T,...

% simulation loop
A = Aseq{1}; % initial true dynamics matrix
q = length(Aseq);
N = 0;
l = 2;
if q >= l
    N = Aseq{l};
end

for k = 1:1:Nf
    % modify true dynamics according to Aseq specified in dyn_inputs
    % this is to simulate changing dynamics over time
    if k==N
        A = Aseq{l+1};
        l = l+2;
        if q >= l
            N = Aseq{l};
        end
    end

    % simulate true system's dynamics
    X0 = [x0; V0]; % initial augmented states
    tspan = [(k-1)*T k*T];
    [t,X] = ode45(@(t,X) dynamics(t,X,A,B,Q,R,P), tspan, X0);

    % retrieve final states
    XT = X(end,:)';
    xT = XT(1:end-1);
    VT = XT(end);

    % update loop variables
    xbar0 = xtransform(x0);
    xbarT = xtransform(xT);
    j = j+1;
    Z(:,j) = xbar0 - xbarT;
    Y(j) = VT - V0;

    % update critic weights
    if j == Nupd
        disp(['iter ', num2str(k), ', time ', num2str((k-1)*T)])
        % Temporal Difference error
        measured_cost = VT - V0;
        expected_cost = x0'*P*x0 - xT'*P*xT;
        TD_error = measured_cost - expected_cost;
        % update weights (batch least squares)
        if abs(TD_error) > eps
            disp(['  > Update: TD error ', num2str(TD_error)])
            disp(' ')
            Pbar = Z'\Y; % same as Pbar = (Z*Z')\Z*Y
            P = Pbartransform(Pbar);
        end
        % re-initialize loop variables
        j = 0;
        Z = zeros(p,Nupd);
        Y = zeros(Nupd, 1);
    end

    % update initial states
    x0 = xT;
    V0 = VT;
end

end