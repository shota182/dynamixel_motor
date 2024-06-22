import numpy as np
import time

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
        l_diff = -self.wire_sign[id] * np.sum(self.coupled_arr[section]) * theta_kappa_rad * self.r_wirepos * np.cos(self.wirepos_arr[id]-phai_kappa_rad)
        step = l_diff/self.r_pulley * 180/np.pi * 4096/360
        return int(np.round(self.calibrated_step[id]+step, decimals=0))


if __name__ == "__main__":
    # motor param
    calibrated_deg = [53.96, 162.33, 192.57, 264.99, 111.53,
                      47.29, 153.02, 3.60, 111.97]
    calibrated_stp = [614, 1847, 2191, 3015, 1269,
                      538, 1741, 41, 1274]
    ## 締め付け時の符号
    wire_sign = [1,1,1, 1,1,1, 1,1,1]
    
    # manipulator param
    ## モータ側から見たとき（根本ｰ>先端の視点で）右をx，上をy
    wirepos_arr = [1/2*np.pi, 7/6*np.pi, 11/6*np.pi,
                   1/6*np.pi, 5/6*np.pi, 3/2*np.pi,
                   1/2*np.pi, 7/6*np.pi, 11/6*np.pi]
    ## セクションは根本からナンバリング
    coupled_arr = [[1,1,1,0,0,0,0,0,0],
                   [0,0,0,1,1,1,0,0,0],
                   [0,0,0,0,0,0,1,1,1]]

    # test
    Model = TDCR_model(calibrated_stp, wire_sign, wirepos_arr, coupled_arr)
    # 所望曲率
    phai, theta = 0, np.pi/6
    wire = [np.pi/2, 7/6*np.pi, 11/6*np.pi]
    section = 0 # 0スタート
    for id in range(1,10):
        print(f"id:{id} is {calibrated_stp[id-1]} to {Model.kappa2step(phai, theta, section, id)}")