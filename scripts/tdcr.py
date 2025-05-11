import numpy as np
import time
from dynamixel_sdk import *  # Uses Dynamixel SDK library
from xl330 import XL330
import copy

class TDCR_model:
    def __init__(self, calibrated_step, wire_sign, wirepos_arr, coupled_arr, r_pulley=8, r_wirepos=8, kappa_limit=np.pi/6):
        self.calibrated_step = calibrated_step
        self.wire_sign = wire_sign
        self.wirepos_arr = wirepos_arr
        self.coupled_arr = coupled_arr
        self.r_pulley = r_pulley # [mm]
        self.r_wirepos = r_wirepos # [mm]
        self.kappa_limit = kappa_limit
        self.ID = range(-1,10)

    def kappa2step(self, phai_kappa_rad, theta_kappa_rad, section, id):
        # id：モータid．1番スタートで配列と兼ね合わせてる
        id = self.ID[id]
        # section：0番スタート．どのセクションなのか．曲率が骨あたりだからワイヤが何個の骨を通ってるかみるため
        if(self.kappa_limit < theta_kappa_rad):
            print(f"曲率の最大値 {self.kappa_limit} [rad]を超えないように与えてください")
            raise Exception('Error!')
        # print(f"cos:{self.wirepos_arr[id], phai_kappa_rad}")
        l_diff = self.wire_sign[id] * np.sum(self.coupled_arr[section]) * theta_kappa_rad * self.r_wirepos * np.cos(self.wirepos_arr[id]-phai_kappa_rad)
        step = l_diff/self.r_pulley * 180/np.pi * 4096/360
        return int(np.round(self.calibrated_step[id]+step, decimals=0))


if __name__ == "__main__":
    Xl330 = XL330(range(1,10))
    
    if(0):
        Xl330.setmode(Xl330.CURRENT_MODE)
        Xl330.rxposition()
        Xl330.setcurrent([30]*9)
        Xl330.tx(1.5)
        
        time.sleep(1.5)

    Xl330.setmode(Xl330.EXTENDEDPOSITION_MODE)
    Xl330.rxposition()
    print(f"start position is : {Xl330.position_pre}")

    # motor param
    # calibrated_deg = [53.96, 162.33, 192.57, 264.99, 111.53,
    #                   47.29, 153.02, 3.60, 111.97]
    # calibrated_stp = [614, 1847, 2191, 3015, 1269,
    #                   538, 1741, 41, 1274]
    calibrated_stp = copy.copy(Xl330.position_pre)
    ## 締め付け向きの符号
    wire_sign = [1,-1,1, 1,1,1, 1,1,1]
    
    # manipulator param
    ## モータ側から見たとき（根本ｰ>先端の視点で）右をx，上をy
    wirepos_arr = [1/6*np.pi, 5/6*np.pi, 3/2*np.pi,
                   1/6*np.pi, 3/2*np.pi, 11/6*np.pi,
                   1/6*np.pi, 3/2*np.pi, 1/2*np.pi]
    ## セクションは根本からナンバリング
    coupled_arr = [[1,1,1,1,1,1,1,1,1],
                   [0,0,0,0,0,0,0,0,0],
                   [0,0,0,0,0,0,0,0,0]]

    # test
    Model = TDCR_model(calibrated_stp, wire_sign, wirepos_arr, coupled_arr)
    # 所望曲率
    input_arr = [[1/9*np.pi,1/6*np.pi,1/6*np.pi]]
    # phai = np.repeat(1/2*np.pi, 9)
    phai = np.repeat(1/2*np.pi, 9)
    # print(f"phai: \n{phai}")
    theta = np.repeat(input_arr, 3).reshape((-1,9))
    # phai, theta = -1/2*np.pi, np.pi/6
    section = np.repeat([0,1,2], 3) # 0スタート
    goal_step = []
    for t in range(len(theta)):
        # print(f"t: {t}")
        g = []
        # print(f"g: {g}")
        for id in range(1,10):
            stp = Model.kappa2step(phai[id-1], theta[t][id-1], section[id-1], id)
            g.append(stp)
            print(f"id:{id} is {calibrated_stp[id-1]} to {stp}")
        goal_step.append(g)
        now_step = calibrated_stp
    print(f"now_step: \n{now_step}")
    print(f"goal_step: \n{goal_step}")

    if(1):
        try:
            for no in range(200):
                step = no%len(goal_step)
                time_length = 2
                increment = 10
                for t in np.linspace(0, time_length, increment):
                    goal = np.round(np.array(now_step) + t/time_length * (np.array(goal_step[step])-np.array(now_step)), decimals=0)
                    # input = np.concatenate([np.array(sublist) for sublist in [calibrated_stp[:6], [int(g) for g in goal]]])
                    input = [int(g) for g in goal]
                    print(input)
                    Xl330.setposition(input)
                    Xl330.tx(time_length/increment)
                now_step = goal_step[step]
        except KeyboardInterrupt:
            pass
        
    if(1):
        time.sleep(1)
        
        Xl330.setmode(Xl330.CURRENT_MODE)
        Xl330.rxposition()
        Xl330.setcurrent(np.tile([30,-30,30],3))
        Xl330.tx(1.5)

    
    if(0):
        Xl330.setmode(Xl330.EXTENDEDPOSITION_MODE)
        Xl330.rxposition()
        now_step = Xl330.position_pre
        goal_step = calibrated_stp
        print(now_step)
        print(goal_step)
        for t in np.linspace(0, time_length, increment):
            goal = np.round(np.array(now_step) + t/time_length * (np.array(goal_step)-np.array(now_step)), decimals=0)
            # input = np.concatenate([np.array(sublist) for sublist in [calibrated_stp[:6], [int(g) for g in goal]]])
            input = [int(g) for g in goal]
            print(input)
            Xl330.setposition(input)
            Xl330.tx(time_length/increment)
