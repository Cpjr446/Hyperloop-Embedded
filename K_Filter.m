% Hyperloop Pod Velocity Estimation using Kalman Filter

% Parameters
dt = 0.1;  % Time step (s)
T = 300;   % Total simulation time (s)
t = 0:dt:T;  % Time vector
N = length(t);  % Number of time steps

% Pod and environment parameters
mass = 15000;  % Pod mass (kg)
drag_coeff = 0.2;  % Drag coefficient
tube_radius = 5;  % Tube radius (m)
base_air_density = 0.1;  % Base low-pressure air density in tube (kg/m^3)
gravity = 9.81;  % Gravity (m/s^2)
tube_slope = 0.01;  % Slight upward slope

% GPS update frequency
gps_update_freq = 0.1;  % GPS update every 0.1 seconds
gps_noise = 2;  % GPS noise (m)

% New environmental parameters
base_pressure = 100;  % Base air pressure in Pa
base_temperature = 293.15;  % Base temperature in K (20°C)
friction_coeff = 0.01;  % Coefficient of friction with tube wall

% True state dynamics (enhanced model)
x_true = zeros(3, N);  % [position; velocity; acceleration]
x_true(:,1) = [0; 0; 0];  % Initial state

% Control input (thrust force in Newtons)
u = zeros(1, N);
u(1:100) = 200000;  % Acceleration phase
u(501:600) = -100000;  % Deceleration phase

% Generate environmental variations
pressure_variation = base_pressure + 10 * sin(0.01 * t);  % Pressure varies sinusoidally
temperature_variation = base_temperature + 5 * sin(0.005 * t);  % Temperature varies sinusoidally

% Generate true states
for k = 2:N
    % Current state
    pos = x_true(1, k-1);
    vel = x_true(2, k-1);
    
    % Calculate air density based on pressure and temperature
    air_density = base_air_density * (pressure_variation(k) / base_pressure) * (base_temperature / temperature_variation(k));
    
    % Calculate forces
    F_thrust = u(k-1);
    F_drag = 0.5 * drag_coeff * air_density * pi * tube_radius^2 * vel^2 * sign(vel);
    F_gravity = mass * gravity * sin(atan(tube_slope));
    F_magnetic_drag = 0.1 * vel;  % Simplified magnetic drag
    F_tube_irregularity = 500 * sin(0.1 * pos);  % Simulate tube irregularities
    F_friction = friction_coeff * mass * gravity * cos(atan(tube_slope)) * sign(vel);  % Friction with tube
    
    % Calculate acceleration
    acc = (F_thrust - F_drag - F_gravity - F_magnetic_drag - F_tube_irregularity - F_friction) / mass;
    
    % Update state
    x_true(:, k) = [
        pos + vel*dt + 0.5*acc*dt^2;
        vel + acc*dt;
        acc
    ];
end

% Simulating noisy sensor measurements
v_measured = x_true(2,:) + 2*randn(1,N);  % Noisy velocity (e.g., from Doppler sensor)
a_measured = x_true(3,:) + 0.5*randn(1,N);  % Noisy acceleration (e.g., from accelerometer)

% GPS measurements (less frequent, but gives absolute position)
gps_times = 1:gps_update_freq/dt:N;
gps_measurements = x_true(1, gps_times) + gps_noise * randn(size(gps_times));

% Kalman Filter Implementation
% State transition matrix
F = [1 dt 0.5*dt^2; 0 1 dt; 0 0 1];

% Control input matrix
B = [0.5*dt^2; dt; 1] / mass;

% Observation matrices
H_no_gps = [0 1 0; 0 0 1];  % Measuring velocity and acceleration
H_gps = [1 0 0; 0 1 0; 0 0 1];  % Measuring position, velocity, and acceleration

% Process noise covariance
Q = diag([0.01, 0.1, 1]);

% Measurement noise covariance
R_no_gps = diag([1, 0.1]);  % For velocity and acceleration
R_gps = diag([gps_noise^2, 1, 0.1]);  % For GPS, velocity, and acceleration

% Initial state estimate
x_est = zeros(3, N);
x_est(:,1) = [0; 0; 0];  % Initial state estimate

% Initial error covariance
P = diag([100, 10, 1]);  % Initial error covariance

% Kalman Filter loop
for k = 2:N
    % Predict
    x_pred = F * x_est(:,k-1) + B * u(k-1);
    P_pred = F * P * F' + Q;
    
    % Update
    if ismember(k, gps_times)
        % GPS update available
        gps_index = find(gps_times == k);
        y = [gps_measurements(gps_index); v_measured(k); a_measured(k)] - H_gps * x_pred;
        S = H_gps * P_pred * H_gps' + R_gps;
        K = P_pred * H_gps' / S;
        x_est(:,k) = x_pred + K * y;
        P = (eye(3) - K * H_gps) * P_pred;
    else
        % Only velocity and acceleration update
        y = [v_measured(k); a_measured(k)] - H_no_gps * x_pred;
        S = H_no_gps * P_pred * H_no_gps' + R_no_gps;
        K = P_pred * H_no_gps' / S;
        x_est(:,k) = x_pred + K * y;
        P = (eye(3) - K * H_no_gps) * P_pred;
    end
end

% Plotting results
figure;
subplot(3,1,1);
plot(t, x_true(1,:), 'b', t, x_est(1,:), 'r', gps_times*dt, gps_measurements, 'g.');
legend('True Position', 'Estimated Position', 'GPS Measurements');
xlabel('Time (s)');
ylabel('Position (m)');
title('Hyperloop Pod Position');

subplot(3,1,2);
plot(t, x_true(2,:), 'b', t, v_measured, 'g', t, x_est(2,:), 'r');
legend('True Velocity', 'Measured Velocity', 'Estimated Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Hyperloop Pod Velocity');

subplot(3,1,3);
plot(t, x_true(3,:), 'b', t, a_measured, 'g', t, x_est(3,:), 'r');
legend('True Acceleration', 'Measured Acceleration', 'Estimated Acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Hyperloop Pod Acceleration');

subplot(4,1,4);
bias_line = ones(size(t)) * bias_vel; % Create a line for the bias
plot(t, x_true(2,:) - x_est(2,:), 'b', t, bias_line, 'r--');
legend('Velocity Estimation Error', 'Bias');
xlabel('Time (s)');
ylabel('Velocity Error (m/s)');
title('Hyperloop Pod Velocity Estimation Bias');

% Calculate RMSE
rmse_pos = sqrt(mean((x_true(1,:) - x_est(1,:)).^2));
rmse_vel = sqrt(mean((x_true(2,:) - x_est(2,:)).^2));
rmse_acc = sqrt(mean((x_true(3,:) - x_est(3,:)).^2));
fprintf('RMSE of position estimation: %.2f m\n', rmse_pos);
fprintf('RMSE of velocity estimation: %.2f m/s\n', rmse_vel);
fprintf('RMSE of acceleration estimation: %.2f m/s^2\n', rmse_acc);

% Calculate bias
bias_vel = mean(x_true(2,:) - x_est(2,:));
fprintf('Bias of velocity estimation: %.2f m/s\n', bias_vel);


% Plot environmental variations
figure;
subplot(2,1,1);
plot(t, pressure_variation);
xlabel('Time (s)');
ylabel('Pressure (Pa)');
title('Air Pressure Variation in Tube');

subplot(2,1,2);
plot(t, temperature_variation);
xlabel('Time (s)');
ylabel('Temperature (K)');
title('Temperature Variation in Tube');
