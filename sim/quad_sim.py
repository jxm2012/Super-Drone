#!/usr/bin/env python3
"""
Simple quadcopter vertical axis simulation with sensor models and PID altitude controller.

Produces plots: altitude, velocity, thrust, sensor readings, estimator.
"""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Simulation parameters
m = 0.5  # mass (kg)
g = 9.80665
dt = 0.005  # simulation timestep (s)
sim_time = 20.0
steps = int(sim_time / dt)

# Sensor noise models
accel_noise_std = 0.2  # m/s^2
baro_noise_std = 0.5  # meters
baro_rate = 25.0  # Hz
baro_dt = 1.0 / baro_rate

# Simple PID controller for altitude
class PID:
    def __init__(self, kp, ki, kd, dt, integrator_limit=1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integrator = 0.0
        self.prev_err = 0.0
        self.limit = integrator_limit

    def update(self, setpoint, measurement):
        err = setpoint - measurement
        self.integrator += err * self.dt
        # anti-windup
        self.integrator = np.clip(self.integrator, -self.limit, self.limit)
        deriv = (err - self.prev_err) / self.dt
        self.prev_err = err
        return self.kp * err + self.ki * self.integrator + self.kd * deriv


def run_sim():
    t = np.arange(0, sim_time, dt)
    z = np.zeros_like(t)
    vz = np.zeros_like(t)
    thrust = np.zeros_like(t)

    # sensor / estimator
    baro_z = np.zeros_like(t)
    accel_z = np.zeros_like(t)
    est_z = np.zeros_like(t)
    est_vz = np.zeros_like(t)

    pid = PID(kp=6.0, ki=1.0, kd=1.0, dt=dt, integrator_limit=5.0)

    setpoint = 2.0  # meters

    next_baro_t = 0.0
    for i in range(len(t)):
        # true dynamics
        # thrust is vertical force from motors (N)
        u = thrust[i - 1] if i > 0 else m * g
        az = (u - m * g) / m
        vz[i] = vz[i - 1] + az * dt if i > 0 else az * dt
        z[i] = z[i - 1] + vz[i] * dt if i > 0 else vz[i] * dt

        # sensors
        # accelerometer measures specific force (az + gravity) with noise
        accel_meas = az + np.random.randn() * accel_noise_std
        accel_z[i] = accel_meas

        # barometer samples at lower rate and returns noisy altitude
        if t[i] >= next_baro_t:
            baro_meas = z[i] + np.random.randn() * baro_noise_std
            baro_z[i] = baro_meas
            next_baro_t += baro_dt
        else:
            baro_z[i] = baro_z[i - 1] if i > 0 else 0.0

        # simple estimator: complementary between integrated accel and baro
        if i == 0:
            est_vz[i] = 0.0
            est_z[i] = 0.0
        else:
            est_vz[i] = est_vz[i - 1] + accel_z[i] * dt
            est_z[i] = 0.98 * (est_z[i - 1] + est_vz[i] * dt) + 0.02 * baro_z[i]

        # control (PID on estimated altitude)
        u_cmd = pid.update(setpoint, est_z[i])
        # convert PID output to thrust (simple mapping), clamp
        thrust_cmd = m * g + np.clip(u_cmd * m, -0.8 * m * g, 2.0 * m * g)
        thrust[i] = thrust_cmd

        # write next step thrust for dynamics
        if i + 1 < len(t):
            thrust[i + 1] = thrust_cmd

    # plotting
    plt.figure(figsize=(10, 8))
    plt.subplot(4, 1, 1)
    plt.plot(t, z, label="true z")
    plt.plot(t, est_z, label="est z")
    plt.plot(t, baro_z, label="baro z", alpha=0.6)
    plt.axhline(setpoint, color="k", linestyle="--", label="setpoint")
    plt.legend()
    plt.ylabel("Altitude (m)")

    plt.subplot(4, 1, 2)
    plt.plot(t, vz)
    plt.ylabel("Velocity (m/s)")

    plt.subplot(4, 1, 3)
    plt.plot(t, thrust / m)
    plt.ylabel("Thrust accel (m/s^2)")

    plt.subplot(4, 1, 4)
    plt.plot(t, accel_z, label="accel meas")
    plt.ylabel("Accel (m/s^2)")
    plt.xlabel("Time (s)")

    plt.tight_layout()
    plt.savefig("sim_altitude.png")

    print("Simulation complete. Output: sim_altitude.png")


if __name__ == "__main__":
    run_sim()
