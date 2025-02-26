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

% Case 2: Both horizontal bar angle (x₁) and pendulum angle (x₃) are measured
C_case2 = [1 0 0 0 0; 0 0 1 0 0];  % Matrix C when measuring x₁ and x₃

% Compute the observability matrix
O_case2 = obsv(A, C_case2);

% Check the rank of the observability matrix
rank_O_case2 = rank(O_case2);

% Check if the system is observable
if rank_O_case2 == state_dim
    disp('Case 2: System is observable when measuring both angles (x₁ and x₃)')
else
    disp('Case 2: System is NOT observable when measuring both angles (x₁ and x₃)')
    disp(['Rank of observability matrix: ', num2str(rank_O_case2)])
    disp(['State dimension: ', num2str(state_dim)])
end

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
Qr2 = diag([10, 0, 10, 0, 0]); % Equal priority on both angles
Rr2 = 1;

% Case 3: Focus primarily on pendulum angle with some weight on velocities
Qr3 = diag([10, 0.1, 10, 0.1, 0]); % More comprehensive weighting
Rr3 = 0.5;                       % Lower penalty on control effort

% Case 4: High priority on stability with aggressive control
Qr4 = diag([5, 1, 20, 1, 0]);    % High weight on pendulum angle
Rr4 = 0.1;                      % Lower penalty allows more aggressive control

% Compute the controller gains for each case using LQR
K1 = lqr(A, B, Qr1, Rr1);
K2 = lqr(A, B, Qr2, Rr2);
K3 = lqr(A, B, Qr3, Rr3);
K4 = lqr(A, B, Qr4, Rr4);

% Display the computed gains
disp('Case 1: Focus on pendulum angle');
disp(['K1 = ', mat2str(K1, 5)]);
disp(' ');

disp('Case 2: Focus on both bar angle and pendulum angle');
disp(['K2 = ', mat2str(K2, 5)]);
disp(' ');

disp('Case 3: Comprehensive weighting with moderate control');
disp(['K3 = ', mat2str(K3, 5)]);
disp(' ');

disp('Case 4: High priority on stability with aggressive control');
disp(['K4 = ', mat2str(K4, 5)]);
disp(' ');

% Calculate and display closed-loop poles for each case
disp('Closed-loop poles for Case 1:');
eig1 = eig(A - B*K1);
disp(eig1);
disp(' ');

disp('Closed-loop poles for Case 2:');
eig2 = eig(A - B*K2);
disp(eig2);
disp(' ');

disp('Closed-loop poles for Case 3:');
eig3 = eig(A - B*K3);
disp(eig3);
disp(' ');

disp('Closed-loop poles for Case 4:');
eig4 = eig(A - B*K4);
disp(eig4);
disp(' ');

% Get and display the damping ratios and natural frequencies
[zeta1, wn1] = get_damping_natfreq(eig1);
[zeta2, wn2] = get_damping_natfreq(eig2);
[zeta3, wn3] = get_damping_natfreq(eig3);
[zeta4, wn4] = get_damping_natfreq(eig4);

disp('Case 1 complex poles - Damping ratios and natural frequencies:');
for i = 1:length(zeta1)
    disp(['Pole pair ', num2str(i), ': damping ratio = ', num2str(zeta1(i)), ', natural frequency = ', num2str(wn1(i)), ' rad/s']);
end
disp(' ');

disp('Case 2 complex poles - Damping ratios and natural frequencies:');
for i = 1:length(zeta2)
    disp(['Pole pair ', num2str(i), ': damping ratio = ', num2str(zeta2(i)), ', natural frequency = ', num2str(wn2(i)), ' rad/s']);
end
disp(' ');

disp('Case 3 complex poles - Damping ratios and natural frequencies:');
for i = 1:length(zeta3)
    disp(['Pole pair ', num2str(i), ': damping ratio = ', num2str(zeta3(i)), ', natural frequency = ', num2str(wn3(i)), ' rad/s']);
end
disp(' ');

disp('Case 4 complex poles - Damping ratios and natural frequencies:');
for i = 1:length(zeta4)
    disp(['Pole pair ', num2str(i), ': damping ratio = ', num2str(zeta4(i)), ', natural frequency = ', num2str(wn4(i)), ' rad/s']);
end

%% Question 6

% Set up controller parameters
Qr = diag([10, 1, 10, 1, 1]); % Weight matrix for states
Rr = 1;                       % Weight for input variabl

K = lqr(A, B, Qr, Rr);


% Initial conditions - small perturbation in the horizontal bar angle
x0 = [0 0 0.1 0 0]';
% Simulation parameters
D = [0 0 0 0 0]';
T = 2;  % Simulation time in seconds

% Run simulation
out = sim('question6.slx', T);
u = out.u;
x = out.x;
t= out.t;

% Plot results
figure;
plot(t, x, 'LineWidth', 1.5);
xlabel('Time (s)', 'FontSize', 14);
ylabel('States', 'FontSize', 14);
title('Inverted Pendulum State Feedback Control');
legend('α (rad)', 'α̇ (rad/s)', 'β (rad)', 'β̇ (rad/s)', 'i (A)');
grid on;

% Plot control input
figure;
plot(t, u, 'LineWidth', 1.5);
xlabel('Time (s)', 'FontSize', 14);
ylabel('Control input u (V)', 'FontSize', 14);
title('Control Input');
grid on;

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
disp('Computed observer (estimator) gain L:');
disp(L);

%% Question 8

Qr = diag([10, 1, 10, 1, 1]); % Weight matrix for states
Rr = 1;                       % Weight for input variabl
K = lqr(A, B, Qr, Rr);

A_c = A - B*K - L*C;  % Controller dynamics matrix
B_c = L;              % Controller input matrix
C_c = -K;             % Controller output matrix
D_c = zeros(size(C_c,1), size(B_c,2)); 

% Initial conditions - small perturbation in the horizontal bar angle
x0 = [1 0 1 0 0]';
D =[0 0]';
% Simulation parameters
T = 2;  % Simulation time in seconds

% Run simulation
out = sim('question8.slx', T);
u = out.u;
y = out.y;
t = out.t;
x1 = out.x1;

% Plot results
figure;
plot(t, y, 'LineWidth', 1.5);
xlabel('Time (s)', 'FontSize', 14);
ylabel('States', 'FontSize', 14);
title('Inverted Pendulum State Feedback Control');
legend('α (rad)', 'β (rad)', 'hey');
grid on;

x1_reshaped = reshape(x1, 5, 501);  % Convert (5 × 1 × 501) → (5 × 501)
figure;
plot(t, x1_reshaped', 'LineWidth', 1.5);  % Transpose for correct plot
xlabel('Time (s)', 'FontSize', 14);
ylabel('States', 'FontSize', 14);
title('Inverted Pendulum State Feedback Control');
legend('α (rad)', 'α̇ (rad/s)', 'β (rad)', 'β̇ (rad/s)', 'i (A)');
grid on;


% Plot control input
figure;
plot(t, u, 'LineWidth', 1.5);
xlabel('Time (s)', 'FontSize', 14);
ylabel('Control input u (V)', 'FontSize', 14);
title('Control Input');
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




