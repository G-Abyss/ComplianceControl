%% =========================================================================
% ADMITTANCE CONTROL SIMULATION
% =========================================================================
% This script simulates an admittance control system for force-controlled
% interaction with environment. The system regulates the relationship between
% position and force by controlling the mechanical admittance.
%
% Author: [GenAbyss]
% Date: [2025.9.2]
% Version: 1.0
%
% Theory: The admittance control law is given by:
%         ẍ_f = (τ_c - τ_d - Bd·ẋ_f - Kd·x_f) / Md
%         where Md, Bd, Kd are desired mass, damping, and stiffness
% =========================================================================

%% System Parameters Configuration
% Desired mechanical admittance parameters
Md = 0.001;    % Desired mass (kg)
Bd = 0.01;     % Desired damping (N·s/m) 
Kd = 0.1;      % Desired stiffness (N/m)

% Physical system parameters
k = 1;         % Spring stiffness (N/m)
d = 1;         % Maximum displacement (m)
C_originx = 1; % Origin position of contact point (m)

% Trajectory function: 5th order polynomial for smooth motion
y = @(t) d*(640*t^3 - 3840*t^4 + 6144*t^5); % Position trajectory (m)

% Force calculation functions
tau_c = @(delta_x) k * delta_x;  % Contact force based on spring deflection
tau_d = 0;                       % Desired force (N) - set to zero for free motion

%% Initial Conditions
x_f = 0;      % Initial position (m)
dx_f = 0;     % Initial velocity (m/s)
ddx_f = 0;    % Initial acceleration (m/s²)

%% Simulation Setup
dt = 0.005;   % Time step (s)
t = 0:dt:1;   % Time vector (s)
n = length(t); % Number of simulation steps

% Data recording arrays
x_f_record = zeros(1, n);      % Position history
dx_f_record = zeros(1, n);     % Velocity history  
ddx_f_record = zeros(1, n);    % Acceleration history
tau_c_record = zeros(1, n);    % Contact force history
y_record = zeros(1, n);        % Desired position history

%% Main Simulation Loop
fprintf('Starting admittance control simulation...\n');
for i = 1:n
    % Determine desired position based on time
    if t(i) <= 0.25
        yp = y(t(i));  % Follow trajectory for first 0.25s
    else
        yp = d;        % Hold final position
    end
    
    % Calculate contact force based on spring deflection
    tau_cp = tau_c(yp - x_f);
    
    % Admittance control law implementation
    ddx_fp = ((tau_cp - tau_d) - Bd*dx_f - Kd*x_f) / Md;
    
    % Euler integration for state update
    dx_fp = dx_f + ddx_fp * dt;
    x_fp = x_f + dx_fp * dt;
    
    % Record simulation data
    x_f_record(i) = x_fp;
    dx_f_record(i) = dx_fp;
    ddx_f_record(i) = ddx_fp;
    tau_c_record(i) = tau_cp;
    y_record(i) = yp;
    
    % Update states for next iteration
    x_f = x_fp;
    dx_f = dx_fp;
    ddx_f = ddx_fp;
end
fprintf('Simulation completed successfully!\n');

%% Results Visualization
fprintf('Generating plots...\n');
figure('Name', 'Admittance Control Simulation Results', 'Position', [100, 100, 800, 600]);

% Position plot
subplot(4, 1, 1);
plot(t, x_f_record, 'b-', 'LineWidth', 1.5);
hold on;
plot(t, y_record, 'r--', 'LineWidth', 1.5);
title('System Position Response', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Position (m)', 'FontSize', 12);
legend('Actual Position', 'Desired Position', 'Location', 'best');
grid on;

% Velocity plot
subplot(4, 1, 2);
plot(t, dx_f_record, 'b-', 'LineWidth', 1.5);
title('System Velocity Response', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Velocity (m/s)', 'FontSize', 12);
grid on;

% Acceleration plot
subplot(4, 1, 3);
plot(t, ddx_f_record, 'b-', 'LineWidth', 1.5);
title('System Acceleration Response', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Acceleration (m/s²)', 'FontSize', 12);
grid on;

% Force plot
subplot(4, 1, 4);
plot(t, tau_c_record, 'g-', 'LineWidth', 1.5);
title('Contact Force Response', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Force (N)', 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 12);
grid on;

% Adjust subplot spacing
sgtitle('Admittance Control System Simulation Results', 'FontSize', 16, 'FontWeight', 'bold');

%% Animation Generation
fprintf('Press ENTER to generate animated GIF...\n');
userInput = input('', 's');
close;

fprintf('Generating animated GIF...\n');
% Generate frames for animation
p = 1:3:n;  % Sample every 3rd frame for animation
q = 1;
nImages = length(p);
im = cell(1, nImages);

for i = p
    % Calculate position of controlled object (A)
    A = [x_f_record(i), 0]';
    A1 = [A(1), A(2) + 0.1]';
    A2 = [A(1) - 0.2, A(2) + 0.1]';
    A3 = [A(1) - 0.2, A(2) - 0.1]';
    A4 = [A(1), A(2) - 0.1]';
    A5 = [A(1) - 0.1, A(2) - 0.1]';
    A0 = [A(1) - 0.1, A(2)]';

    % Calculate position of reference point (C)
    C = [y_record(i) + C_originx, 0]';
    C1 = [C(1), C(2) + 0.1]';
    C2 = [C(1) + 0.2, C(2) + 0.1]';
    C3 = [C(1) + 0.2, C(2) - 0.1]';
    C4 = [C(1), C(2) - 0.1]';
    C0 = [C(1) + 0.1, C(2)]';
    
    % Calculate connection line and velocity vector
    AC = [A, C];
    dx = A5 + [dx_f_record(i)/10, 0]';  % Velocity vector (scaled 1:10)
    v = [A5, dx];

    % Create animation frame
    fig = figure('Visible', 'off');
    set(gcf, 'unit', 'normalized', 'position', [0.2, 0.0, 0.70, 0.28]);

    % Plot connection line
    plot(AC(1, :), AC(2, :), 'k-', 'LineWidth', 1);
    hold on;
    
    % Define object boundaries
    lines{1} = [A1, A2];  % Object A boundaries
    lines{2} = [A2, A3];
    lines{3} = [A3, A4];
    lines{4} = [A4, A1];
    lines{5} = [C1, C2];  % Object C boundaries
    lines{6} = [C2, C3];
    lines{7} = [C3, C4];
    lines{8} = [C4, C1];

    % Plot object boundaries
    for j = 1:length(lines)
        plot(lines{j}(1, :), lines{j}(2, :), 'b-', 'LineWidth', 2);
    end
    
    % Plot velocity vector with direction indication
    if dx(1) - A5(1) >= 0
        plot(v(1, :), v(2, :), 'r-', 'LineWidth', 2);
        plot(dx(1), dx(2), '>r', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    else
        plot(v(1, :), v(2, :), 'g-', 'LineWidth', 2);
        plot(dx(1), dx(2), '<g', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    end

    % Add labels
    text(A0(1), A0(2), 'A', 'FontSize', 16, 'FontWeight', 'bold');
    text(C0(1), C0(2), 'C', 'FontSize', 16, 'FontWeight', 'bold');

    % Configure plot appearance
    grid on;
    axis([-0.5 + x_f_record(1), C_originx + d + 0.5, -0.4, 0.4]);
    title(sprintf('Admittance Control Animation - Time: %.3f s', t(i)), ...
          'FontSize', 12, 'FontWeight', 'bold');

    % Capture frame
    frame = getframe(fig);
    im{q} = frame2im(frame);
    fprintf('Frame %d/%d completed\n', q, nImages);
    close(fig);
    q = q + 1;
end

%% GIF Generation
fprintf('Creating GIF file...\n');
filename = 'AdmittanceControl_Animation.gif';

for idx = 1:nImages
    [A, map] = rgb2ind(im{idx}, 256);
    if idx == 1
        imwrite(A, map, filename, 'gif', 'LoopCount', Inf, 'DelayTime', 0.1);
    else
        imwrite(A, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    end
end

fprintf('Animation saved as: %s\n', filename);
fprintf('Admittance control simulation completed successfully!\n');