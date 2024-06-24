import numpy as np
import time
from dynamixel_sdk import *  # Uses Dynamixel SDK library
from xl330 import XL330
import copy

class TDCR_model:
    def __init__(self, motor_id, wire_sign, wirepos_arr, bone_segment, segment_motor, r_pulley=8, r_wirepos=8, kappa_limit=np.pi/6):
        # XL330 constructor
        self.Xl330 = XL330(motor_id)
        self.Xl330.rxposition()
        # initialization
        self.calibrated_step = copy.copy(self.Xl330.position_pre)
        self.wire_sign = wire_sign
        self.wirepos_arr = wirepos_arr
        # bone_segmentとsegment_motorからbone_motorつくる？
        self.bone_segment = bone_segment
        self.segment_motor = segment_motor
        self.r_pulley = r_pulley # [mm]
        self.r_wirepos = r_wirepos # [mm]
        self.kappa_limit = kappa_limit
        self.ID = range(-1,10)

    def kappa2step(self, phai_kappa_rad, theta_kappa_rad, section, id):
        # id：モータid．1番スタートで配列と兼ね合わせてる
        id = self.ID[id]
        # 指定したIDのモータがどのセグメントに属するか
        segment = self.segment_motor[self.ID[id]]
        if(self.kappa_limit < theta_kappa_rad):
            print(f"曲率の最大値 {self.kappa_limit} [rad]を超えないように与えてください")
            raise Exception('Error!')
        # print(f"cos:{self.wirepos_arr[id], phai_kappa_rad}")
        l_diff = self.wire_sign[self.ID[id]] * np.sum(self.bone_segment[section]) * theta_kappa_rad * self.r_wirepos * np.cos(self.wirepos_arr[self.ID[id]]-phai_kappa_rad)
        step = l_diff/self.r_pulley * 180/np.pi * 4096/360
        return int(np.round(self.calibrated_step[self.ID[id]]+step, decimals=0))
    
    def kappa2step(self, phai_kappa_rad, theta_kappa_rad, section, id):
        # id：モータid．1番スタートで配列と兼ね合わせてる
        id = self.ID[id]
        # section：0番スタート．どのセクションなのか．曲率が骨あたりだからワイヤが何個の骨を通ってるかみるため
        if(self.kappa_limit < theta_kappa_rad):
            print(f"曲率の最大値 {self.kappa_limit} [rad]を超えないように与えてください")
            raise Exception('Error!')
        # print(f"cos:{self.wirepos_arr[id], phai_kappa_rad}")
        l_diff = self.wire_sign[id] * np.sum(self.bone_segment[section]) * theta_kappa_rad * self.r_wirepos * np.cos(self.wirepos_arr[id]-phai_kappa_rad)
        step = l_diff/self.r_pulley * 180/np.pi * 4096/360
        return int(np.round(self.calibrated_step[id]+step, decimals=0))
    
    def current(self, current, time_sleep=0):
        self.Xl330.setmode(self.Xl330.CURRENT_MODE)
        self.Xl330.rxposition()
        self.Xl330.setcurrent(current)
        self.Xl330.tx(time_sleep)

    def current_bulk(self, current=0, time_sleep=0):
        self.current(current*len(self.Xl330.ID_list), time_sleep)

    def get_current_pos(self):
        self.Xl330.setmode(self.Xl330.EXTENDEDPOSITION_MODE)
        self.Xl330.rxposition()
        self.position = copy.copy(self.Xl330.position_pre)
        print(f"'self.position' is : {self.position}")
    
    def exposition(self, position, time_length=0, increment=10):
        self.Xl330.setmode(self.Xl330.EXTENDEDPOSITION_MODE)
        self.Xl330.rxposition()
        now_step = copy.copy(self.Xl330.position_pre)
        goal_step = position
        print(f"exposition diff mode")
        for id in range(len(self.Xl330.ID_list)):
            print(f"ID.{self.ID[id]}: {now_step[id]} to {goal_step[id]}")
        if(time_length>0):
            for t in np.linspace(0, time_length, increment):
                goal = np.round(np.array(now_step) + t/time_length * (np.array(goal_step)-np.array(now_step)), decimals=0)
                # input = np.concatenate([np.array(sublist) for sublist in [calibrated_stp[:6], [int(g) for g in goal]]])
                input = [int(g) for g in goal]
                self.Xl330.setposition(input)
                self.Xl330.tx(time_length/increment)
        else:
            self.Xl330.setposition(goal_step)
            self.Xl330.tx(time_length)

    def exposition_diff(self, position_diff, time_length=0, increment=10):
        now_step = copy.copy(self.Xl330.position_pre)
        self.exposition(now_step+position_diff, time_length, increment)
    

if __name__ == "__main__":
    # test
    # manipulator param
    ## モータ側から見たとき（根本ｰ>先端の視点で）右をx，上をy
    wirepos_arr = [3/2*np.pi, 1/6*np.pi, 5/6*np.pi,
                   7/6*np.pi, 1/2*np.pi, 11/6*np.pi,
                   7/6*np.pi, 11/6*np.pi, 1/2*np.pi]
    ## 骨とセグメントの対応関係（プログラムのwire_array）
    bone_segment = [[1,1,0,0,1,1,0,1,1],
                    [0,0,1,1,0,0,1,1,1],
                    [0,0,0,0,1,1,0,1,1]]
    ## 締め付け向きの符号
    wire_sign = [1,1,1, 1,1,1, 1,1,1]
    # 各モータのセグメント割当て
    segment_motor = [2,2,2, 1,1,1, 3,3,3]
    Model = TDCR_model(range(1,10), wire_sign, wirepos_arr, bone_segment, segment_motor)
    
    # 電流
    if(0): Model.current_bulk(30, 2)

    # 位置読み取り
    if(0): Model.get_current_pos()

    # 所望曲率
    phai = np.repeat([np.pi, 0, 0], 3)
    theta = np.repeat([np.pi/6], 9)
    # phai, theta = -1/2*np.pi, np.pi/6
    section = np.repeat([0,1,2], 3) # 0スタート
    goal_step = []
    for id in range(1,10):
        stp = Model.kappa2step(phai[id-1], theta[id-1], section[id-1], id)
        goal_step.append(stp)
        print(f"id:{id} is {Model.calibrated_step[id-1]} to {stp}")

    # Ex位置
    if(0): Model.exposition(goal_step, 3)
        
    # 電流
    if(0):
        time.sleep(1)
        Model.current_bulk(30, 1.5)
    
    # 初期位置に戻る
    if(0): Model.exposition(Model.calibrated_step, 3)
