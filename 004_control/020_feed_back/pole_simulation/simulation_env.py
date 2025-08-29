import time
import numpy as np
from pole_model import PendulumEnvCustom
import matplotlib.pyplot as plt

class Pole_simulation():
    def __init__(self, g=4, T=12.0, dt=0.02):
        # default value... simlation length: 12s, simulation step 0.02s

        # prepare environment
        self.env = PendulumEnvCustom(render_mode="human", g=g)
        self.obs = self.env.reset()

        # torque and reference value
        self.torque_value = 0.0
        self.ref_value = 0.0

        # simulation info
        self.T = T  # max time
        self.dt = dt  # time step

        # list for data
        self.angle_list = []
        self.ref_list = []
        self.torque_list = []

        # simulation status
        self.simtime = 0.0
        self.is_running = True

    # update simulation
    def update(self):
        self.action = [self.torque_value]
        self.obs, self.reward, self.terminated, self.truncated, self.info = self.env.step(self.action)

        self.angle_list.append(self.angle())
        self.ref_list.append(self.ref_value)
        self.torque_list.append(self.torque_value)

        # update simulation time
        self.simtime += self.dt
        if self.simtime >= self.T:
            self.stop()
            self.close()
            self.plot()

    # get torque input for simulation
    def torque_input(self, torque):
        self.torque_value = torque

    # set reference value for simulation
    def ref_input(self, ref):
        self.ref_value = ref

    # return current angle of the pendulum
    def angle(self):
        sin = self.obs[0]
        cos = self.obs[1]
        angle = np.arctan2(sin, cos)
        # unnormalize angle
        if self.angle_list != []:
            idx_last = len(self.angle_list) -1
            if angle - self.angle_list[idx_last] > np.pi:
                angle -= 2*np.pi
            elif angle - self.angle_list[idx_last] < -np.pi:
                angle += 2*np.pi
        return angle
    
    # return current angular velocity
    def angular_vel(self):
        return self.obs[2]

    # reset simulation
    def stop(self):
        self.obs = self.env.reset()

    # close simulation env
    def close(self):
        self.env.close()
        self.is_running = False
    
    # plot angles and references
    def plot(self):
        t = np.arange(0, len(self.angle_list)*self.dt, self.dt)
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

        ax1.plot(t, self.angle_list, label="Angle")
        ax1.plot(t, self.ref_list, label="Reference")
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("Angle [rad]")
        ax1.set_title("Angle and Reference")
        ax1.legend()
        ax1.grid(True)

        ax2.plot(t, self.torque_list, label="Torque", color="orange")
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Torque [Nm]")
        ax2.set_title("Torque")
        ax2.legend()
        ax2.grid(True)

        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    # initiate simulation environment
    T = 12.0
    dt = 0.02
    pole = Pole_simulation(T=T, dt=dt)

    # PID controller gains
    k_p = 10
    k_i = 18 * dt
    k_d = 1.5 / dt

    # angle normalization
    def normalize_angle(angle):
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        return angle

    ref_id = 0
    refs = [-np.pi/3, -np.pi/3, 0, np.pi/3]
    refs_t = [0, 4, 8, 12]  # time to change reference

    error_ex = 0.0
    error_i = 0.0
    while True:
        pole.update()

        if pole.is_running:
            # update reference
            if ref_id < len(refs) and pole.simtime >= refs_t[ref_id]:
                ref_id += 1
                pole.ref_input(refs[ref_id])
            
            # PD control
            angle = pole.angle()
            reference = refs[ref_id]

            error = normalize_angle( float(reference) - pole.angle() )
            error_d = error - error_ex
            error_i += error
            torque = -1 * ( k_p * error + k_i * error_i + k_d * error_d )
            pole.torque_input(torque)

            error_ex = error
            time.sleep(dt)
        
        else:
            break
