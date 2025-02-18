clc;
clearvars;
close all;

exp = 2;

%% experiment 1
if exp==1

    % internal dynamics matrix (true, used for simulation)
    A1 = [-0.0665, 11.5,       0,       0;
                0, -2.5,     2.5,       0;
             -9.5,    0, -13.736, -13.736;
              0.6,    0,       0,       0];
    
    % input dynamics matrix
    B = [0; 0; 13.736; 0];
    
    % LQR cost matrices
    Q = eye(4);
    R = 1;
    
    % internal dynamics matrix (estimated, used to derive inital stabilizing control policy
    Anom = [-0.0665,      8,       0,       0;
                  0, -3.663,   3.663,       0;
              -6.86,      0, -13.736, -13.736;
                0.6,      0,       0,       0];
    
    % algorithm inputs
    solve_initial_lqr = true; % use initial stabilizing control input as solution of the LQR with Anom?
    x0 = [0; 0.1; 0; 0]; % initial states 
    T = 0.05; % sampling time (sec)
    tf = 20; % time horizon, over which the system is simulated (sec)
    Nupd = 20; % number of samples to collect before updating the critic weights
    eps = 1e-14; % threshold for algorithm convergence
    
    % pack inputs
    dyn_inputs = {{A1}, B, Q, R, Anom};
    algo_inputs = {solve_initial_lqr, x0, T, tf, Nupd, eps};
    
    % solve ARE with true dynamics
    [Ktrue,Ptrue] = lqr(A1,B,Q,R);
    ol_true = ss(A1, B, eye(4), zeros(4,1)); % open-loop
    cl_true = ss(A1-B*Ktrue, B, eye(4), zeros(4,1)); % closed-loop
    figure; initial(ol_true, x0); title('True System Open-Loop');
    figure; initial(cl_true, x0); title('True System Optimal Closed-Loop');

    % solve ARE with estimated dynamics
    [Knom, Pnom] = lqr(Anom,B,Q,R);
    ol_est = ss(Anom, B, eye(4), zeros(4,1)); % open-loop (estimated)
    cl_est = ss(Anom-B*Knom, B, eye(4), zeros(4,1)); % closed-loop (estimated)
    figure; initial(ol_est, x0); title('Estimated System Open-Loop');
    figure; initial(cl_est, x0); title('Estimated System Optimal Closed-Loop');

    % run IRL algorithm
    [P] = online_linreg(dyn_inputs, algo_inputs);

    % compare IRL and ARE solutions
    disp(Ptrue)
    disp(P)

%% experiment 2
elseif exp==2
    % dynamics-related inputs
    A1 =[-1.01887,    0.90506,   -0.00215;
          0.82225,   -1.07741,   -0.17555;    
                0,          0,      -20.2];
    % N2 = 21;
    % A2 =[   -0.69,    0.90506,   -0.00215;
    %       0.82225,   -1.07741,   -0.17555;    
    %             0,          0,      -20.2];
    B = [0; 0; 20.2];
    Q=eye(3);
    R=1;
    Anom = zeros(3);

    % algorithm inputs
    solve_initial_lqr = false; % use initial stabilizing control input as solution of the LQR with Anom?
    x0 = [0.1; 0.2; 0.1]; % initial states 
    T = 0.05; % sampling time (sec)
    tf = 4; % time horizon, over which the system is simulated (sec)
    Nupd = 6; % number of samples to collect before updating the critic weights
    eps = 1e-5; % threshold for algorithm convergence

    % pack inputs
    dyn_inputs = {{A1}, B, Q, R, Anom};
    algo_inputs = {solve_initial_lqr, x0, T, tf, Nupd, eps};

    % solve ARE with true dynamics
    [Ktrue,Ptrue] = lqr(A1,B,Q,R);
    ol_true = ss(A1, B, eye(3), zeros(3,1)); % open-loop
    cl_true = ss(A1-B*Ktrue, B, eye(3), zeros(3,1)); % closed-loop
    figure; initial(ol_true, x0); title('True System Open-Loop');
    figure; initial(cl_true, x0); title('True System Optimal Closed-Loop');

    % run IRL algorithm
    [P] = online_linreg(dyn_inputs, algo_inputs);

    % compare IRL and ARE solutions
    disp(Ptrue)
    disp(P)

end

