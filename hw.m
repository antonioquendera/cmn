clear

% Load state model
load('IP_MODEL.mat')

%% Eigenvalues and analyses

values = eig(A);

%analysez

%{


Eigenvalues provide insight into the properties of a matrix, such as stability, definiteness, and dimensionality. Based on your eigenvalues:

0,−737.32,−19.37,−5.76,6.99 

Here’s what you can conclude:

Zero Eigenvalue:

The eigenvalue 0 suggests that the matrix is singular and not invertible. This indicates that the matrix has linearly dependent rows or columns, meaning it has reduced rank.

Negative Eigenvalues:

The negative eigenvalues −737.32, −19.37, and −5.76 imply that the matrix has some concave directions if it represents a quadratic form. In terms of stability, they suggest stable directions.

Positive Eigenvalue:

The positive eigenvalue 6.99 indicates an unstable direction if the matrix represents a dynamic system. If this matrix is associated with a Hessian, it means the quadratic form is not positive definite but rather indefinite.

Definiteness:

Since the matrix has both positive and negative eigenvalues, it is classified as an indefinite matrix. This often occurs in saddle points of optimization problems.

Rank:

The zero eigenvalue suggests that the matrix has rank deficiency. If this is a 5×5 matrix, its rank is likely 4 instead of 5.
%}

%% Controllability
% Compute the controllability matrix
C_matrix = ctrb(A, B);
disp('Controllability Matrix:');
disp(C);

% Check the rank of the controllability matrix
n = size(A, 1);  % Number of states
rank_C = rank(C_matrix);

if rank_C == n
    disp('The system is controllable.');
else
    disp('The system is NOT controllable.');
end

%Sistem is controllable

%% Question 3

% Case 1: Only pendulum angle (x₃) is measured
C_case1 = [0 0 1 0 0];  % Matrix C when only measuring x₃

% Compute the observability matrix
O_case1 = obsv(A, C_case1);

% Check the rank of the observability matrix
rank_O_case1 = rank(O_case1);
state_dim = size(A, 1);

% Check if the system is observable
if rank_O_case1 == state_dim
    disp('Case 1: System is observable when measuring only the pendulum angle (x₃)')
else
    disp(['Case 1: System is NOT observable when measuring only the pendulum angle (x₃)'])
    disp(['Rank of observability matrix: ', num2str(rank_O_case1)])
    disp(['State dimension: ', num2str(state_dim)])
end

% % Case 2: Both horizontal bar angle (x₁) and pendulum angle (x₃) are measured
% C_case2 = [1 0 0 0 0; 0 0 1 0 0];  % Matrix C when measuring x₁ and x₃
% 
% % Compute the observability matrix
% O_case2 = obsv(A, C_case2);
% 
% % Check the rank of the observability matrix
% rank_O_case2 = rank(O_case2);
% 
% % Check if the system is observable
% if rank_O_case2 == state_dim
%     disp('Case 2: System is observable when measuring both angles (x₁ and x₃)')
% else
%     disp('Case 2: System is NOT observable when measuring both angles (x₁ and x₃)')
%     disp(['Rank of observability matrix: ', num2str(rank_O_case2)])
%     disp(['State dimension: ', num2str(state_dim)])
% end

%% Question 4

% Create a state-space model from the matrices
sys = ss(A, B, C, D);

% Convert the state-space model to transfer function
% We'll consider the transfer function from input u to the pendulum angle (x₃)
% So we'll use the third row of C matrix for the output

sys_pendulum = ss(A, B, C, D);
[num, den] = ss2tf(A, B, C, D);
tf_pendulum = tf(num(1,:), den);

% Define the frequency range for the Bode plot (in rad/s)
% Using a logarithmic scale from 0.1 to 100 rad/s
w = logspace(-1, 2, 1000);

% Plot the Bode diagram
% figure;
% bode(sys_pendulum, w);
% grid on;
% title('Bode Diagram of the Open-Loop System (Pendulum Angle)');

% Comment on the diagram shape
disp('Comments on the Bode diagram:');
disp('1. The transfer function is expected to show instability due to the inverted pendulum nature.');
disp('2. The system likely shows a -40 dB/decade roll-off at high frequencies due to double integration.');
disp('3. There might be a phase shift crossing -180° which indicates the instability point.');
disp('4. The system''s unstable pole will cause the magnitude to increase at low frequencies. ');


%% Question 5

% Define different weight matrices for testing
% Case 1: Focus on pendulum angle (x₃) and minimal control effort
Qr1 = diag([1, 0, 10, 0, 0]);  % Prioritize pendulum angle (x₃)
Rr1 = 1;                      % Moderate penalty on control effort

% Case 2: Focus on both bar angle (x₁) and pendulum angle (x₃)
% Qr2 = diag([10, 0, 10, 0, 0]); % Equal priority on both angles
% Rr2 = 1;
% 
% % Case 3: Focus primarily on pendulum angle with some weight on velocities
% Qr3 = diag([10, 0.1, 10, 0.1, 0]); % More comprehensive weighting
% Rr3 = 0.5;                       % Lower penalty on control effort
% 
% % Case 4: High priority on stability with aggressive control
% Qr4 = diag([5, 1, 20, 1, 0]);    % High weight on pendulum angle
% Rr4 = 0.1;                      % Lower penalty allows more aggressive control

% Compute the controller gains for each case using LQR
K1 = lqr(A, B, Qr1, Rr1);
% K2 = lqr(A, B, Qr2, Rr2);
% K3 = lqr(A, B, Qr3, Rr3);
% K4 = lqr(A, B, Qr4, Rr4);

% Display the computed gains
disp('Case 1: Focus on pendulum angle');
disp(['K1 = ', mat2str(K1, 5)]);
disp(' ');

% disp('Case 2: Focus on both bar angle and pendulum angle');
% disp(['K2 = ', mat2str(K2, 5)]);
% disp(' ');
% 
% disp('Case 3: Comprehensive weighting with moderate control');
% disp(['K3 = ', mat2str(K3, 5)]);
% disp(' ');
% 
% disp('Case 4: High priority on stability with aggressive control');
% disp(['K4 = ', mat2str(K4, 5)]);
% disp(' ');

% Calculate and display closed-loop poles for each case
disp('Closed-loop poles for Case 1:');
eig1 = eig(A - B*K1);
disp(eig1);
disp(' ');

% disp('Closed-loop poles for Case 2:');
% eig2 = eig(A - B*K2);
% disp(eig2);
% disp(' ');
% 
% disp('Closed-loop poles for Case 3:');
% eig3 = eig(A - B*K3);
% disp(eig3);
% disp(' ');
% 
% disp('Closed-loop poles for Case 4:');
% eig4 = eig(A - B*K4);
% disp(eig4);
% disp(' ');

% Get and display the damping ratios and natural frequencies
[zeta1, wn1] = get_damping_natfreq(eig1);
% [zeta2, wn2] = get_damping_natfreq(eig2);
% [zeta3, wn3] = get_damping_natfreq(eig3);
% [zeta4, wn4] = get_damping_natfreq(eig4);

disp('Case 1 complex poles - Damping ratios and natural frequencies:');
for i = 1:length(zeta1)
    disp(['Pole pair ', num2str(i), ': damping ratio = ', num2str(zeta1(i)), ', natural frequency = ', num2str(wn1(i)), ' rad/s']);
end
disp(' ');

% disp('Case 2 complex poles - Damping ratios and natural frequencies:');
% for i = 1:length(zeta2)
%     disp(['Pole pair ', num2str(i), ': damping ratio = ', num2str(zeta2(i)), ', natural frequency = ', num2str(wn2(i)), ' rad/s']);
% end
% disp(' ');
% 
% disp('Case 3 complex poles - Damping ratios and natural frequencies:');
% for i = 1:length(zeta3)
%     disp(['Pole pair ', num2str(i), ': damping ratio = ', num2str(zeta3(i)), ', natural frequency = ', num2str(wn3(i)), ' rad/s']);
% end
% disp(' ');
% 
% disp('Case 4 complex poles - Damping ratios and natural frequencies:');
% for i = 1:length(zeta4)
%     disp(['Pole pair ', num2str(i), ': damping ratio = ', num2str(zeta4(i)), ', natural frequency = ', num2str(wn4(i)), ' rad/s']);
% end

%% Question 6

clear; clc; close all;

QR = diag([100, 0 , 1 , 0, 0]);

% Define the control penalty
Rr = 0.3;

% Load your system model (ensure A, B, C, D are in the workspace)
load('IP_MODEL.mat');

D = [0 0 0 0 0]';

% Compute the LQR gain
K = lqr(A, B, QR, Rr);

% Define simulation parameters
T = 5; % Simulation time in seconds
% Set initial conditions (modify as needed)
x0 = [1; 0; 1; 0; 0];

q6 = sim("question6.slx",T);

x=q6.x;
u=q6.u;
t=q6.t;

% Define a threshold for settling (for states 1 and 3)
settle_threshold = 0.05;  

% Extract states 1 and 3
state13 = x(:, [1, 3]);

% Initialize settling time as the simulation time.
settlingTime = T;  

% Loop over simulation time to find when both states are within the threshold
for k = 1:length(t)
    if all(abs(state13(k, :)) < settle_threshold)
        settlingTime = t(k);
        break;
    end
end

% Compute the maximum current magnitude from state 5
% (Here we use absolute value; adjust if your model has a sign convention)
maxCurrent = max(abs(x(:,5)));

%% Plot the Results

% figure;
% subplot(2,1,1);
% plot(t, x, 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('States');
% legend('x1','x2','x3','x4','x5');
% title('Closed-loop Response');
% grid on;
% 
% subplot(2,1,2);
% plot(t, x(:,5), 'LineWidth', 1.5);
% xlabel('Time (s)');
% ylabel('Current (A)');
% title('Current Response (State 5)');
% grid on;

%% Display Performance Metrics
% fprintf('LQR Gain K:\n');
% disp(K);
% fprintf('Settling time (states 1 & 3): %.3f s\n', settlingTime);
% fprintf('Maximum current (state 5): %.3f A\n', maxCurrent);

%% Optimal finder with Q6 candidates
% This script finds Q and R values that yield faster convergence for states 1 and 3,
% while ensuring that the current (state 5) stays within a desired range.
% Candidate parameters: Q1, Q2, Q3, Q4, Q5, and R are varied.

% clear; clc; close all;
% 
% % Load system model (A, B, C, D, etc.)
% load('IP_MODEL.mat')  % Ensure A, B, C, D are in the workspace
% 
% D = [0 0 0 0 0]';
% % Define candidate ranges for weights
% Q1_candidates = [100, 150, 200];   % Candidate weights for state 1
% Q2_candidates = [0.5, 1, 5];              % Candidate weights for state 2
% Q3_candidates = [1, 5, 10, 20, 50, 80, 100]; % Candidate weights for state 3
% Q4_candidates = [0.5, 1, 5];              % Candidate weights for state 4
% Q5_candidates = [0];                % Candidate weights for state 5 (current)
% R_candidates  = [0.3];  % Candidate control penalties
% 
% % Simulation parameters
% T = 2;   % Simulation time in seconds
% % Define initial condition (modify as needed)
% x0 = [1; 0; 1; 4; 0];  
% 
% % Threshold for considering convergence (for states 1 and 3)
% threshold = 0.05;  % You may adjust this value
% 
% % Preallocate result storage
% results = [];
% idx = 1;
% 
% % Loop over candidate Q1, Q2, Q3, Q4, Q5, and R values
% for Q1 = Q1_candidates
%     for Q2 = Q2_candidates
%         for Q3 = Q3_candidates
%             for Q4 = Q4_candidates
%                 for Q5 = Q5_candidates
%                     for R_val = R_candidates
%                         % Build the Q matrix (diagonal)
%                         Qr = diag([Q1, Q2, Q3, Q4, Q5]);
%                         Rr = R_val;
% 
%                         % Compute LQR gain
%                         K = lqr(A, B, Qr, Rr);
% 
%                         % Simulate the closed-loop system:
%
                           % q6 = sim("question6.slx",T);
                           % x=q6.x;
                           % u=q6.u;
                           % t=q6.t;

%                         % Calculate performance metrics:
%                         % 1. Settling time for states 1 and 3 (when absolute error is below 'threshold')
%                         state13 = x(:, [1, 3]);
%                         settle_time = T;
%                         for k = 1:length(t)
%                             if all(abs(state13(k, :)) < threshold)
%                                 settle_time = t(k);
%                                 break;
%                             end
%                         end
% 
%                         % 2. Maximum current reached (state 5)
%                         % (If state 5 is expected to be negative, we use min, else adjust as needed)
%                         max_current = min(x(:,5));  
% 
%                         % Check if the current constraint is satisfied 
%                         % (modify the condition as needed; here we check that the minimum current is not too low)
%                         if max_current >= -50  
%                             % Store candidate result if the constraint is met
%                             results(idx).Q1 = Q1;
%                             results(idx).Q2 = Q2;
%                             results(idx).Q3 = Q3;
%                             results(idx).Q4 = Q4;
%                             results(idx).Q5 = Q5;
%                             results(idx).R  = R_val;
%                             results(idx).settlingTime = settle_time;
%                             results(idx).maxCurrent = max_current;
%                             results(idx).K = K;
%                             idx = idx + 1;
%                         end
% 
%                     end
%                 end
%             end
%         end
%     end
% end
% 
% %% Select the candidate with the minimum settling time
% if isempty(results)
%     error('No candidate found that meets the current constraint.');
% else
%     settlingTimes = [results.settlingTime];
%     [minTime, bestIdx] = min(settlingTimes);
%     bestCandidate = results(bestIdx);
% 
%     fprintf('Best candidate parameters:\n');
%     fprintf('  Q(1,1) = %.2f\n', bestCandidate.Q1);
%     fprintf('  Q(2,2) = %.2f\n', bestCandidate.Q2);
%     fprintf('  Q(3,3) = %.2f\n', bestCandidate.Q3);
%     fprintf('  Q(4,4) = %.2f\n', bestCandidate.Q4);
%     fprintf('  Q(5,5) = %.2f\n', bestCandidate.Q5);
%     fprintf('  R = %.2f\n', bestCandidate.R);
%     fprintf('Settling time (states 1 & 3): %.3f s\n', bestCandidate.settlingTime);
%     fprintf('Max current (state 5): %.3f A\n', bestCandidate.maxCurrent);
%     fprintf('LQR Gain K = [%s]\n', num2str(bestCandidate.K, '%0.4f '));
% end
% 
% 
% subplot(2,1,2)
% plot(t_best, x_best(:,5), 'LineWidth', 1.5);
% xlabel('Time (s)'); ylabel('Current (A)');
% title('Current Response (State 5)');
% grid on;

%% Question 7
% Define the process noise gain. Here, we assume the noise enters each state directly.
G = eye(size(A));

% Define the process noise covariance matrix Qe.
% This represents the variance of the process (model) errors.
% The value 10 is chosen here as an example. You may adjust this value as needed.
Qe = 10 * eye(size(A));

% Define the measurement noise covariance matrix Re.
% The number of outputs is given by the number of rows in C.
Re = eye(size(C, 1));

% Compute the observer gain using the lqe function.
% lqe returns the Kalman filter gain matrix L.
L = lqe(A, G, C, Qe, Re);

% Display the computed observer gain.
% disp('Computed observer (estimator) gain L:');
% disp(L);

%% Question 8

Qr = diag([300, 5, 150 , 0.5, 0]); % Weight matrix for states
Rr = 0.3;                       % Weight for input variabl
K = lqr(A, B, Qr, Rr);

A_c = A - B*K - L*C;  % Controller dynamics matrix
B_c = L;              % Controller input matrix
C_c = -K;             % Controller output matrix
D_c = zeros(size(C_c,1), size(B_c,2)); 

% Initial conditions - small perturbation in the horizontal bar angle
x0 = [0.1 0 0.1 1.2  0]';
D =[0 0]';
% Simulation parameters
T = 30;  % Simulation time in seconds

% Run simulation
out = sim('question8.slx', T);
u8 = out.u;
y8 = out.y;
t8 = out.t;

error_metric = trapz(t8, sum(y8(:,1:2).^2, 2));

max_alpha = max(abs(y8(:,1)));

fprintf('Integrated squared error: %.3f\n', error_metric);
fprintf("Max_alpha: %.4f\n",max_alpha)

% Define a threshold for settling (for states 1 and 3)
settle_threshold = 0.05;  

% Extract states 1 and 3
state_y = y8(:, [1,2]);

% Initialize settling time as the simulation time.
settlingTime = T;  

% Loop over simulation time to find when both states are within the threshold
for k = 1:length(t)
    if all(abs(state_y(k, :)) < settle_threshold)
        settlingTime = t(k);
        break;
    end
end

figure;
plot(t8, u8, 'LineWidth', 1.5); % Dotted line
xlabel('Time (s)', 'FontSize', 14);
ylabel('U', 'FontSize', 14);
title('Inverted Pendulum State Feedback Control', 'FontSize', 14);
legend(); % Adjust legend
grid on;

figure;
plot(t8, y8(:, 1), '-.', 'LineWidth', 1.5); % Dash-dot line for column 1
hold on;
plot(t8, y8(:, 2), '-.', 'LineWidth', 1.5); % Dash-dot line for column 3
hold off;

xlabel('Time (s)', 'FontSize', 14);
ylabel('States', 'FontSize', 14);
title('Inverted Pendulum State Feedback Control', 'FontSize', 14);

legend('\alpha', '\beta', 'FontSize', 12, 'Location', 'best');
grid on;

fprintf('Settling time (states 1 and 3): %.3f s\n', settlingTime);

return;

%% Question 8 - Grid Search Minimizing Integrated Output Error
% Define candidate weight ranges for states 1 to 4 (with Q5 fixed at 0)
Q1_candidates = [200, 300, 400];
Q2_candidates = [5, 10, 15];
Q3_candidates = [150, 200, 300, 400];
Q4_candidates = [0.5, 1, 2];
Q5 = 0;  % Fixed weight for state 5
Rr = 0.3; % Control input weight (fixed)

% Simulation parameters
T = 10;                           % Simulation time (seconds)
x0 = [1; 0; 1; 0; 0];          % Initial condition (small perturbation)
D  = [0; 0];                     % (if needed by your model)

% Preallocate results storage
results = [];
idx = 1;

% Grid search over candidate Q weights (for states 1-4)
for Q1 = Q1_candidates
    for Q2 = Q2_candidates
        for Q3 = Q3_candidates
            for Q4 = Q4_candidates
                
                % Build the state weight matrix
                Qr = diag([Q1, Q2, Q3, Q4, Q5]);
                
                % Compute LQR gain with candidate Qr and fixed Rr
                K = lqr(A, B, Qr, Rr);
                
                % Update controller matrices (Assuming L and C are defined)
                A_c = A - B*K - L*C;
                B_c = L;
                C_c = -K;
                D_c = zeros(size(C_c,1), size(B_c,2));
                
                % (Optionally, assign these variables to the base workspace if needed)
                % assignin('base','K',K); assignin('base','A_c',A_c);
                % assignin('base','B_c',B_c); assignin('base','C_c',C_c);
                % assignin('base','D_c',D_c);
                
                % Run the simulation for the current candidate (using question8.slx)
                out = sim('question8.slx', T);
                y8 = out.y;   % Assuming y8(:,1) = α and y8(:,2) = β
                t8 = out.t;
                
                % Compute performance metric:
                % Integrated Squared Error of outputs from zero.
                % This sums the squared values of α and β over time.
                error_metric = trapz(t8, sum(y8(:,1:2).^2, 2));
                
                % Store candidate results
                results(idx).Q1 = Q1;
                results(idx).Q2 = Q2;
                results(idx).Q3 = Q3;
                results(idx).Q4 = Q4;
                results(idx).R  = Rr;
                results(idx).error_metric = error_metric;
                results(idx).K = K;
                idx = idx + 1;
                
            end
        end
    end
end

%% Select the candidate with the minimum integrated output error
if isempty(results)
    error('No candidate found.');
else
    errorMetrics = [results.error_metric];
    [minError, bestIdx] = min(errorMetrics);
    bestCandidate = results(bestIdx);
    
    fprintf('Best candidate parameters:\n');
    fprintf('  Q(1,1) = %.2f\n', bestCandidate.Q1);
    fprintf('  Q(2,2) = %.2f\n', bestCandidate.Q2);
    fprintf('  Q(3,3) = %.2f\n', bestCandidate.Q3);
    fprintf('  Q(4,4) = %.2f\n', bestCandidate.Q4);
    fprintf('  Q(5,5) = %.2f\n', Q5);
    fprintf('  R = %.2f\n', bestCandidate.R);
    fprintf('Integrated squared error: %.3f\n', bestCandidate.error_metric);
end

%% Optional: Plot the response for the best candidate
K_best = bestCandidate.K;
% Update controller matrices for the best candidate
A_c = A - B*K_best - L*C;
B_c = L;
C_c = -K_best;
D_c = zeros(size(C_c,1), size(B_c,2));

% Run simulation for best candidate
out_best = sim('question8.slx', T);
u8_best = out_best.u;
y8_best = out_best.y;
t8_best = out_best.t;

figure;
plot(t8_best, u8_best, 'LineWidth', 1.5);
xlabel('Time (s)', 'FontSize', 14);
ylabel('Control Input U', 'FontSize', 14);
title('Control Input Response for Best Candidate', 'FontSize', 14);
grid on;

figure;
plot(t8_best, y8_best(:,1), '-.', 'LineWidth', 1.5);
hold on;
plot(t8_best, y8_best(:,2), '-.', 'LineWidth', 1.5);
hold off;
xlabel('Time (s)', 'FontSize', 14);
ylabel('States', 'FontSize', 14);
title('Output Response (α and β) for Best Candidate', 'FontSize', 14);
legend('\alpha', '\beta', 'FontSize', 12, 'Location', 'best');
grid on;

%% Experimentation

save_system('question8','question8_2015a.slx','ExportToVersion','R2015a')

%% Functions
% Calculate damping ratio and natural frequency for complex poles
function [zeta, wn] = get_damping_natfreq(poles)
    zeta = [];
    wn = [];
    for i = 1:length(poles)
        if imag(poles(i)) ~= 0
            % Only process each complex conjugate pair once
            if i == 1 || poles(i) ~= conj(poles(i-1))
                wn_i = abs(poles(i));
                zeta_i = -real(poles(i))/wn_i;
                zeta = [zeta; zeta_i];
                wn = [wn; wn_i];
            end
        end
    end
end




