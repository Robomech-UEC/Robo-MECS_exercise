from simulation_env import Pole_simulation
import numpy as np
import time

# initiate simulation environment
T = 12
dt = 0.02
pole = Pole_simulation(T=T, dt=dt)

# PID controller gains
# ここに制御ゲインを書く。
##################################
k_p = 0
k_i = 0
k_d = 0
##################################

# angle normalization
def normalize_angle(angle):
    # 角度の正規化プログラム
    ##################################

    ##################################
    return angle

ref_id = 0
refs = [-np.pi/3, -np.pi/3, 0, np.pi/3]
refs_t = [0, 4, 8, 12]  # time to change reference

# 必要に応じて中で使う変数を定義してもよい
##################################

##################################
while True:
    pole.update()

    # if pole simulation is running
    if pole.is_running:
        # update reference
        if ref_id < len(refs) and pole.simtime >= refs_t[ref_id]:
            ref_id += 1
            pole.ref_input(refs[ref_id])
        
        # PID control
        # ここに制御則を書く。
        ##################################
        # 現在のpoleの角度と、目標角度
        angle = pole.angle()
        reference = refs[ref_id]

        # エラーを計算し、エラーの差分と、積分も計算する
        # いったん、エラー0を代入
        error = 0
        # エラーを正規化する
        error = normalize_angle(error)

        # トルクを計算し、poleに入力する(pole.torque_input())
        # いったん、トルク0.1を入力
        pole.torque_input(1.8)
        ##################################
        time.sleep(dt)
    
    else:
        break
