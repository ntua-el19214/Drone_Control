clc;
close all;

Parameters

% Additional parameters to tune
Kp_z = 2;
Ki_z = 0.1;
Kd_z = 4.4;


Kp_dist = 1;
Ki_dist = 0;
Kd_dist = 1.05;

Kp_angle = 2.95;
Ki_angle = 0.5;
Kd_angle = 2.6;

Kp_angle_psi = 2.95;
Ki_angle_psi = 0.5;
Kd_angle_psi = 2.6;

set_param("Drone_Model_Initial",'SignalLogging','on') % Enable signal logging for the model
simOut = sim('Drone_Model_Initial'); 

% Retrieve and store simulation output for plotting
phi = simOut.logsout.get('Phi').Values; 
theta = simOut.logsout.get('Theta').Values;
z = simOut.logsout.get('Psi').Values;
psi_input = simOut.logsout.get('Psi_input').Values;

% Create a new figure for the plots
figure;

% Plot Phi
subplot(3, 1, 1);  % Creates a subplot in the first row of a 3-row plot
plot(phi.Time, rad2deg(phi.Data), 'LineWidth', 1.5);
title('Roll Angle (Phi)');
xlabel('Time (s)');
ylabel('\phi (deg)');
grid on;

% Plot Theta
subplot(3, 1, 2);  % Creates a subplot in the second row
plot(theta.Time, rad2deg(theta.Data), 'LineWidth', 1.5);
title('Pitch Angle (Theta)');
xlabel('Time (s)');
ylabel('\theta (deg)');
grid on;

subplot(3, 1, 3);  % Creates a subplot in the third row

% Plot the Yaw Angle (\psi)
plot(z.Time, rad2deg(z.Data), 'LineWidth', 1.5);
hold on;  % Hold the current plot to overlay the next plot

% Plot the Psi input on the same axis
plot(psi_input.Time, rad2deg(psi_input.Data), '--r', 'LineWidth', 1.5);  % Dashed red line for distinction

% Add titles and labels
title('Yaw Angle (\psi) and \psi_{input}');
xlabel('Time (s)');
ylabel('\psi (deg)');

% Add a legend to distinguish the lines
legend('\psi (Yaw Angle)', '\psi_{input} (Input)', 'Location', 'best');

% Enable grid
grid on;

% Release hold
hold off;

% Adjust layout
sgtitle('Simulation Results');  % Adds a title for the whole figure


%% Plot x y z for cascade control 
% Retrieve and store simulation output for plotting
x = simOut.logsout.get('x').Values; 
y = simOut.logsout.get('y').Values;
z = simOut.logsout.get('z').Values;
inputx = simOut.logsout.get('InputX').Values;
inputy = simOut.logsout.get('InputY').Values;

% Create a new figure for the plots
figure;

% Plot Phi
subplot(3, 1, 1);  % Creates a subplot in the first row of a 3-row plot
hold on
plot(x.Time, x.Data, 'LineWidth', 1.5);
plot(inputx.Time, inputx.Data, 'r--', 'LineWidth', 1.5);  % Dashed red line for InputX
title('Distance X (m)');
xlabel('Time (s)');
ylim([-0.1 1.5])
ylabel('x (m)');
legend('Actual', 'Target')
grid on;
hold off

% Plot Theta
subplot(3, 1, 2);  % Creates a subplot in the second row
hold on
plot(y.Time, y.Data, 'LineWidth', 1.5);
plot(inputy.Time, inputy.Data, '--', 'LineWidth', 1.5, 'Color','#3d8c40');  % Dashed green line for InputY
title('Distance Y (m)');
xlabel('Time (s)');
ylabel('y (m)');
ylim([-0.1 1.5])
legend('Actual', 'Target')
grid on;
hold off

% Plot z (Altitude)
subplot(3, 1, 3);  % Creates a subplot in the third row
plot(z.Time, z.Data, 'LineWidth', 1.5);
title('Altitude (z)');
xlabel('Time (s)');
ylabel('z (m)');
ylim([0 2.5])
grid on;





% Adjust layout
sgtitle('Simulation Results');  % Adds a title for the whole figure

% Retrieve and store the blade speed data from the simulation output
Omega1 = simOut.logsout.get('Omega1').Values; 
Omega2 = simOut.logsout.get('Omega2').Values;
Omega3 = simOut.logsout.get('Omega3').Values;
Omega4 = simOut.logsout.get('Omega4').Values;

% Create a new figure for the 2x2 subplot
figure;

% Plot Omega1
subplot(2, 2, 1);
plot(Omega1.Time, sqrt(Omega1.Data), 'LineWidth', 1.5);
title('Angular Velocity \Omega_1');
xlabel('Time (s)');
ylabel('\Omega_1 (rad/s)');
grid on;

% Plot Omega2
subplot(2, 2, 2);
plot(Omega2.Time, sqrt(Omega2.Data), 'LineWidth', 1.5);
title('Angular Velocity \Omega_2');
xlabel('Time (s)');
ylabel('\Omega_2 (rad/s)');
grid on;

% Plot Omega3
subplot(2, 2, 3);
plot(Omega3.Time, sqrt(Omega3.Data), 'LineWidth', 1.5);
title('Angular Velocity \Omega_3');
xlabel('Time (s)');
ylabel('\Omega_3 (rad/s)');
grid on;

% Plot Omega4
subplot(2, 2, 4);
plot(Omega4.Time, sqrt(Omega4.Data), 'LineWidth', 1.5);
title('Angular Velocity \Omega_4');
xlabel('Time (s)');
ylabel('\Omega_4 (rad/s)');
grid on;

% Add a super title for the entire figure
sgtitle('Blade Speeds (Angular Velocities)');

%%
% Extract the data from the simulation output
x_data = x.Data;
y_data = y.Data;
z_data = z.Data;
inputx_data = inputx.Data;
inputy_data = inputy.Data;

% Calculate the direction vectors for the arrows
% For simplicity, we'll use a step size to only plot arrows at intervals
step_size = 10; % Adjust this to control the density of arrows
x_dir = diff(x_data);
y_dir = diff(y_data);
z_dir = diff(z_data);

% Create a plot
figure;
plot3(x_data, y_data, z_data, 'b', 'LineWidth', 2);
hold on;

% Plot arrows for heading direction at specified intervals
for i = 1:step_size:length(x_dir)
    quiver3(x_data(i), y_data(i), z_data(i), x_dir(i), y_dir(i), z_dir(i), ...
        'k', 'LineWidth', 1, 'MaxHeadSize', 0.5);
end

% Enhancing the plot
grid on;
xlabel('X (meters)');
ylabel('Y (meters)');
zlabel('Z (meters)');
title('3D Trajectory of the Drone with Heading Arrows');
legend('Drone Path', 'Input Points', 'Heading Arrows');

% Adjust the view angle for better visualization
view(3); % 3D view
