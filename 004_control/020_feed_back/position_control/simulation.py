import gymnasium as gym
import time
import numpy as np
import time
from pole_model import PendulumEnvCustom

class Pole_simulation():
    def __init__(self):
        # prepare environment
        self.env = PendulumEnvCustom(render_mode="human", g=0.0)
        self.obs = self.env.reset()

        # torque input
        self.torque_value = 0.0
    
    # update simulation
    def update(self):
        self.action = [self.torque_value]
        self.obs, self.reward, self.terminated, self.truncated, self.info = self.env.step(self.action)

    # get torque input for simulation
    def torque_input(self, torque):
        self.torque_value = torque
    
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

if __name__ == "__main__":
    pole = Pole_simulation()
   
    k_p = 0.001
    k_d = 0.1

    def normalize_angle(angle):
        if angle > np.pi:
            angle -= 2 * np.pi
        elif angle < -np.pi:
            angle += 2 * np.pi
        return angle

    # ref = input("input reference angle: ")
    ref = 0

    error_ex = 0.0
    while True:
        pole.update()

        # PD control
        angle = pole.angle()

        error = normalize_angle( float(ref) - pole.angle() )
        error_d = error - error_ex
        torque = -1 * ( k_p * error + k_d * error_d )
        # pole.torque_input(torque)
        pole.torque_input(1.0)

        print(f"deg: {angle}, error: {error}, torque: {torque}")

        time.sleep(0.02)
        error_ex = error
