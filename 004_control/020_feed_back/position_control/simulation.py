import gymnasium as gym
import time
import numpy as np
import time
from pole_model import PendulumEnvCustom
import pandas as pd
import matplotlib.pyplot as plt
import os

class Pole_simulation():
    def __init__(self, T=12.0, dt=0.02):
        # default value... simlation length: 12s, simulation step 0.02s

        # prepare environment
        self.env = PendulumEnvCustom(render_mode="human", g=0.0)
        self.obs = self.env.reset()

        # torque input
        self.torque_value = 0.0
        self.ref_value = 0.0

        # simulation info
        self.T = T  # max time
        self.dt = dt  # time step

        # create file for data
        fname = "result.csv"
        if os.path.exists(fname):
            os.remove(fname)
        self.df = pd.DataFrame({"time[s]", "angle[rad]", "ref[rad]"})
        self.df.to_csv(fname, index=False)

        self.angle_list = []
        self.ref_list = []

        self.simtime = 0.0

        self.is_running = True

    # update simulation
    def update(self):
        self.action = [self.torque_value]
        self.obs, self.reward, self.terminated, self.truncated, self.info = self.env.step(self.action)

        self.angle_list.append(self.angle())
        self.ref_list.append(self.ref_value)

        # update simulation time
        self.simtime += self.dt
        if self.simtime >= self.T:
            self.stop()
            self.close()
            self.plot()

    # get torque input for simulation
    def torque_input(self, torque):
        self.torque_value = torque

    def ref_input(self, ref):
        self.ref_value = ref

    def angle(self):
        sin = self.obs[0]
        cos = self.obs[1]
        return np.arctan2(sin, cos)

    def angular_vel(self):
        return self.obs[2]

    # reset simulation
    def stop(self):
        self.obs = self.env.reset()

    # close simulation env
    def close(self):
        self.env.close()
        self.is_running = False
    
    def plot(self):
        t = np.arange(0, len(self.angle_list)*self.dt, self.dt)
        plt.plot(t, self.angle_list, label="Angle")
        plt.plot(t, self.ref_list, label="Reference")
        plt.xlabel("Time [s]")
        plt.ylabel("Angle [rad]")
        plt.legend()
        plt.show()

if __name__ == "__main__":
    T = 12
    dt = 0.02
    pole = Pole_simulation(T=T, dt=dt)

    k_p = 1
    k_i = 0
    k_d = 60

    def normalize_angle(angle):
        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi
        return angle

    # ref = input("input reference angle: ")
    ref = 0
    pole.ref_input(ref)

    error_ex = 0.0
    error_i = 0.0
    while True:
        pole.update()

        if pole.is_running:
           # PD control
            angle = pole.angle()

            error = normalize_angle( float(ref) - pole.angle() )
            error_d = error - error_ex
            error_i += error
            torque = -1 * ( k_p * error + k_i * error_i + k_d * error_d )
            pole.torque_input(torque)

            error_ex = error
            time.sleep(dt)
        
        else:
            break
