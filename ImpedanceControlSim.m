%% =========================================================================
% IMPEDANCE CONTROL SIMULATION
% =========================================================================
% This script simulates an impedance control system for position-controlled
% interaction with environment. The system regulates the relationship between
% force and position by controlling the mechanical impedance.
%
% Author: [GenAbyss]
% Date: [2025.9.2]
% Version: 1.0
%
% Theory: The impedance control law is given by:
%         τ_f = m·ẍ_c - Md(ẍ_c - ẍ_d) - Bd(ẋ_c - ẋ_d) - Kd(x_c - x_d)
%         where Md, Bd, Kd are desired mass, damping, and stiffness
% =========================================================================

%% System Parameters Configuration
% Desired mechanical impedance parameters
Md = 1;        % Desired mass (kg)
Bd = 10;       % Desired damping (N·s/m)
Kd = 1000;     % Desired stiffness (N/m)

% Physical system parameters
m = 1;         % System mass (kg)
F = 0;         % External force (N) - set to zero for free motion

% Desired trajectory parameters
x_d = 0;       % Desired position (m)
dx_d = 0;      % Desired velocity (m/s)
ddx_d = 0;     % Desired acceleration (m/s²)

% Initial system state
x_c = 1;       % Initial position (m)
dx_c = 0;      % Initial velocity (m/s)
ddx_c = 0;     % Initial acceleration (m/s²)

% Initial control force calculation
tau_f = m*ddx_c - Md*(ddx_c - ddx_d) - Bd*(dx_c - dx_d) - Kd*(x_c - x_d);

%% Simulation Setup
dt = 0.005;    % Time step (s)
t = 0:dt:1;    % Time vector (s)
n = length(t); % Number of simulation steps

% Data recording arrays
x_c_record = zeros(1, n);      % Position history
dx_c_record = zeros(1, n);     % Velocity history
ddx_c_record = zeros(1, n);    % Acceleration history
tau_f_record = zeros(1, n);    % Control force history

%% Main Simulation Loop
fprintf('Starting impedance control simulation...\n');
for i = 1:n
    % System dynamics (Euler integration)
    ddx_cp = (tau_f + F) / m;
    dx_cp = dx_c + ddx_cp * dt;
    x_cp = x_c + dx_cp * dt;
    
    % Impedance control law implementation
    tau_fp = m*ddx_cp - Md*(ddx_cp - ddx_d) - Bd*(dx_cp - dx_d) - Kd*(x_cp - x_d);
    
    % Record simulation data
    x_c_record(i) = x_cp;
    dx_c_record(i) = dx_cp;
    ddx_c_record(i) = ddx_cp;
    tau_f_record(i) = tau_fp;
    
    % Update states for next iteration
    x_c = x_cp;
    dx_c = dx_cp;
    ddx_c = ddx_cp;
    tau_f = tau_fp;
end
fprintf('Simulation completed successfully!\n');

%% Results Visualization
fprintf('Generating plots...\n');
figure('Name', 'Impedance Control Simulation Results', 'Position', [100, 100, 800, 600]);

% Position plot
subplot(4, 1, 1);
plot(t, x_c_record, 'b-', 'LineWidth', 1.5);
hold on;
plot(t, x_d * ones(size(t)), 'r--', 'LineWidth', 1.5);
title('System Position Response', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Position (m)', 'FontSize', 12);
legend('Actual Position', 'Desired Position', 'Location', 'best');
grid on;

% Velocity plot
subplot(4, 1, 2);
plot(t, dx_c_record, 'b-', 'LineWidth', 1.5);
hold on;
plot(t, dx_d * ones(size(t)), 'r--', 'LineWidth', 1.5);
title('System Velocity Response', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Velocity (m/s)', 'FontSize', 12);
legend('Actual Velocity', 'Desired Velocity', 'Location', 'best');
grid on;

% Acceleration plot
subplot(4, 1, 3);
plot(t, ddx_c_record, 'b-', 'LineWidth', 1.5);
hold on;
plot(t, ddx_d * ones(size(t)), 'r--', 'LineWidth', 1.5);
title('System Acceleration Response', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Acceleration (m/s²)', 'FontSize', 12);
legend('Actual Acceleration', 'Desired Acceleration', 'Location', 'best');
grid on;

% Force plot
subplot(4, 1, 4);
plot(t, tau_f_record, 'g-', 'LineWidth', 1.5);
title('Control Force Response', 'FontSize', 14, 'FontWeight', 'bold');
ylabel('Force (N)', 'FontSize', 12);
xlabel('Time (s)', 'FontSize', 12);
grid on;

% Adjust subplot spacing
sgtitle('Impedance Control System Simulation Results', 'FontSize', 16, 'FontWeight', 'bold');

%% Animation Generation
fprintf('Press ENTER to generate animated GIF...\n');
userInput = input('', 's');
close;

fprintf('Generating animated GIF...\n');
% Fixed reference point (A)
A = [-1, 0]';
A1 = [A(1), A(2) + 0.1]';
A2 = [A(1) - 0.2, A(2) + 0.1]';
A3 = [A(1) - 0.2, A(2) - 0.1]';
A4 = [A(1), A(2) - 0.1]';
A0 = [A(1) - 0.1, A(2)]';

% Generate frames for animation
p = 1:3:n;  % Sample every 3rd frame for animation
k = 1;
nImages = length(p);
im = cell(1, nImages);

for i = p
    % Calculate position of controlled object (C)
    C = [x_c_record(i), 0]';
    C1 = [C(1), C(2) + 0.1]';
    C2 = [C(1) + 0.2, C(2) + 0.1]';
    C3 = [C(1) + 0.2, C(2) - 0.1]';
    C4 = [C(1), C(2) - 0.1]';
    C0 = [C(1) + 0.1, C(2)]';
    
    % Calculate connection line and force vector
    AC = [A, C];
    tau = C0 + [tau_f_record(i)/300, 0]';  % Force vector (scaled 1:300)
    F = [C0, tau];

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
    
    % Plot force vector with direction indication
    if tau(1) - C0(1) >= 0
        plot(F(1, :), F(2, :), 'r-', 'LineWidth', 2);
        plot(tau(1), tau(2), '>r', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    else
        plot(F(1, :), F(2, :), 'g-', 'LineWidth', 2);
        plot(tau(1), tau(2), '<g', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    end

    % Add labels
    text(A0(1), A0(2), 'A', 'FontSize', 16, 'FontWeight', 'bold');
    text(C0(1), C0(2), 'C', 'FontSize', 16, 'FontWeight', 'bold');

    % Configure plot appearance
    grid on;
    axis([-0.5 + A(1), x_c_record(1) + 0.5, -0.4, 0.4]);
    title(sprintf('Impedance Control Animation - Time: %.3f s', t(i)), ...
          'FontSize', 12, 'FontWeight', 'bold');

    % Capture frame
    frame = getframe(fig);
    im{k} = frame2im(frame);
    fprintf('Frame %d/%d completed\n', k, nImages);
    close(fig);
    k = k + 1;
end

%% GIF Generation
fprintf('Creating GIF file...\n');
filename = 'ImpedanceControl_Animation.gif';

for idx = 1:nImages
    [A, map] = rgb2ind(im{idx}, 256);
    if idx == 1
        imwrite(A, map, filename, 'gif', 'LoopCount', Inf, 'DelayTime', 0.1);
    else
        imwrite(A, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    end
end

fprintf('Animation saved as: %s\n', filename);
fprintf('Impedance control simulation completed successfully!\n');