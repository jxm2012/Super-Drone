% Simple vertical dynamics and PID example for MATLAB/Octave
% Save as quad_sim.m and run in MATLAB or Octave

clear; close all; clc;

m = 0.5; % kg
g = 9.80665;
dt = 0.005;
sim_time = 20.0;
t = 0:dt:sim_time;
n = length(t);

z = zeros(1,n);
vz = zeros(1,n);
thrust = zeros(1,n);

% sensor noise
accel_noise_std = 0.2;
baro_noise_std = 0.5;
baro_rate = 25;
baro_dt = 1/baro_rate;
next_baro = 0;
baro_z = zeros(1,n);
accel_z = zeros(1,n);
est_z = zeros(1,n);
est_vz = zeros(1,n);

% PID gains
Kp = 6.0; Ki = 1.0; Kd = 1.0;
integrator = 0; prev_err = 0; int_limit = 5;
setpoint = 2.0;

for i = 2:n
    az = (thrust(i-1) - m*g)/m;
    vz(i) = vz(i-1) + az*dt;
    z(i) = z(i-1) + vz(i)*dt;

    accel_meas = az + accel_noise_std*randn();
    accel_z(i) = accel_meas;

    if t(i) >= next_baro
        baro_z(i) = z(i) + baro_noise_std*randn();
        next_baro = next_baro + baro_dt;
    else
        baro_z(i) = baro_z(i-1);
    end

    est_vz(i) = est_vz(i-1) + accel_z(i)*dt;
    est_z(i) = 0.98*(est_z(i-1) + est_vz(i)*dt) + 0.02*baro_z(i);

    err = setpoint - est_z(i);
    integrator = integrator + err*dt;
    integrator = max(min(integrator, int_limit), -int_limit);
    deriv = (err - prev_err)/dt;
    prev_err = err;
    u_cmd = Kp*err + Ki*integrator + Kd*deriv;
    thrust(i) = m*g + max(min(u_cmd*m, 2.0*m*g), -0.8*m*g);
end

% plots
figure;
subplot(4,1,1);
plot(t,z,t,est_z,t,baro_z); hold on; yline(setpoint,'--k'); legend('true','est','baro');
ylabel('Altitude (m)');
subplot(4,1,2); plot(t,vz); ylabel('Velocity (m/s)');
subplot(4,1,3); plot(t,thrust/m); ylabel('Thrust acc (m/s^2)');
subplot(4,1,4); plot(t,accel_z); ylabel('Accel (m/s^2)'); xlabel('Time (s)');
