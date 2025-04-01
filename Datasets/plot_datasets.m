clear; 

% Open a file selection dialog
[file, path] = uigetfile('*.mat', 'Select a MAT-file');

% Check if the user selected a file or canceled
if isequal(file, 0)
    disp('No file selected. Exiting...');
    return;
else
    fullFilePath = fullfile(path, file);
    disp(['Loading file: ', fullFilePath]);
    load(fullFilePath);
end

dt = 0.01; % Time step in seconds
time = (0:length(u)-1) * dt; % Generate the time vector

y = y';

last_valid = find(~(y(1,:) == 0 & y(2,:) == 0), 1, 'last');

if ~isempty(last_valid)
    y = y(:, 1:last_valid);
    time_y = time(1:last_valid);
    u = u(1:last_valid + 1000);
    time_u = time(1:last_valid + 1000);
end

figure;

% Plot Input Control (u)
subplot(2,1,1);
plot(time_u, u, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angle (rads)', 'Interpreter', 'latex');
title('Input Control $u$', 'Interpreter', 'latex');
grid on;

% Plot System Response (y)
subplot(2,1,2);
plot(time_y, y, 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Angle (rads)', 'Interpreter', 'latex');
title('System Response $y$', 'Interpreter', 'latex');
legend('$\alpha$','$\beta$', 'Interpreter', 'latex');
grid on;