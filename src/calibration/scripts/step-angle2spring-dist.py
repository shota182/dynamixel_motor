# モータのステップ角の変化からばね変位の計算をするスクリプト
# このスクリプトは、モータのステップ角の変化からばねの変位を計算し、結果をCSVファイルに保存します。
# 変位ベースの計算であり，ばねの厳密化をする場合はこのスクリプトではなくmag_calibration.cppを使用してください。
import csv
import sys
from datetime import datetime
from tqdm import tqdm
import numpy as np
import matplotlib.pyplot as plt

class StepAngle2SpringDist:
    def __init__(self, step_1lap, length_fixedend2springpulley_initial, length_springpulley2motor_initial, theta_wire_fixedend2springpulley_initial, theta_wire_springpulley2motor_initial, distance_fixedend2springpulley_holizontal, distance_springpulley2motor_holizontal, radius_springpulley, radius_fixedend, radius_motor, spring_displacement_limit, plot_increment):
        # 初期化
        self.step_1lap = step_1lap                                                                             # [step]
        self.length_fixedend2springpulley_initial = length_fixedend2springpulley_initial * 10**(-3)            # [m]
        self.length_springpulley2motor_initial = length_springpulley2motor_initial * 10**(-3)                  # [m]
        self.theta_wire_fixedend2springpulley_initial = theta_wire_fixedend2springpulley_initial * np.pi / 180.0   # [rad]
        self.theta_wire_springpulley2motor_initial = theta_wire_springpulley2motor_initial * np.pi / 180.0     # [rad]
        # self.distance_fixedend2springpulley_holizontal = distance_fixedend2springpulley_holizontal * 10**(-3)  # [m]
        # self.distance_springpulley2motor_holizontal = distance_springpulley2motor_holizontal * 10**(-3)        # [m]
        self.radius_springpulley = radius_springpulley * 10**(-3)                                              # [m]
        self.radius_fixedend = radius_fixedend * 10**(-3)                                                      # [m]
        self.radius_motor = radius_motor * 10**(-3)                                                            # [m]
        self.spring_displacement_limit = np.array(spring_displacement_limit) * 10**(-3)                        # [m]
        self.plot_increment = plot_increment * 10**(-3)                                                        # [m]
        self.distance_fixedend2springpulley_holizontal = (np.sqrt(self.length_fixedend2springpulley_initial**2-self.radius_springpulley**2)+self.radius_springpulley*np.tan(self.theta_wire_fixedend2springpulley_initial))*np.cos(self.theta_wire_fixedend2springpulley_initial)           # [m]
        self.distance_springpulley2motor_holizontal = np.sqrt(self.length_springpulley2motor_initial**2 - (self.radius_motor-self.radius_springpulley)**2)*np.cos(self.theta_wire_springpulley2motor_initial) - self.radius_motor*np.sin(self.theta_wire_springpulley2motor_initial)        # [m]
        print(f"d1: {self.distance_fixedend2springpulley_holizontal*1000}")
        print(f"d2: {self.distance_springpulley2motor_holizontal*1000}")
    
    def calculate_stepmotorangle(self):
        # ばねの変位の配列を生成して，モータのステップ角変化を計算してcsvに保存+プロットする
        # info
        print(f"ばねの範囲：{self.spring_displacement_limit[0]} m から {self.spring_displacement_limit[1]} m")
        print(f"プロットの間隔：{self.plot_increment} m")
        print(f"データ数：{int((self.spring_displacement_limit[1] - self.spring_displacement_limit[0]) / self.plot_increment) + 1}")
        # モータステップのデータ生成
        self.spring_displacement_data = np.arange(self.spring_displacement_limit[0], self.spring_displacement_limit[1]+self.plot_increment/2, self.plot_increment)
        # forで回す
        self.motor_step_angle_data = []
        for s in tqdm(self.spring_displacement_data):
            print(f"s: {s}")
            self.calc_springdist2stepangle(s)
            self.motor_step_angle_data.append(self.step_motorangle)
        # save csv
        self.save_to_csv(self.motor_step_angle_data, self.spring_displacement_data, "motor_step_angle", "spring_displacement")
        # plot spring_displacement - motor_step_angle
        plt.figure()
        plt.grid()
        plt.plot(self.spring_displacement_data, self.motor_step_angle_data, "black")
        plt.xlabel("Spring Displacement (m)")
        plt.ylabel("Motor Step Angle (step)")
        plt.show()
        # motor_step_angle - plot spring_displacement
        plt.figure()
        plt.grid()
        plt.plot(self.motor_step_angle_data, self.spring_displacement_data, "black")
        plt.xlabel("Motor Step Angle (step)")
        plt.ylabel("Spring Displacement (m)")
        plt.show()

    def save_to_csv(self, a, b, name_a, name_b):
        if(len(a) != len(b)):
            print("Error: a and b must have the same length.")
            sys.exit(1)
        filename = f"data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([name_a, name_b])  # ヘッダー行
            for i,j in zip(a,b):
                writer.writerow([i, j])

    def calc_springdist2stepangle(self, spring_displacement): # delta_x
        # バネの変位からモータのステップ角を計算する
        self.spring_displacement = spring_displacement
        self.calc_heightfixedend2springpulley()
        self.calc_lengthfixedend2springpulley()
        self.calc_thetafixedend2springpulley()
        self.calc_thetawirefixedend2springpulley()
        self.calc_heightspringpulley2motorinitial()
        self.calc_lengthspringpulley2motor()
        self.calc_thetaspringpulley2motor()
        self.calc_thetawirespringpulley2motor()
        self.calc_lengthwirespringpulley()
        self.calc_lengthwirefixedend2springpulley()
        self.calc_lengthwirespringpulley2motor()
        self.calc_lengthwirespringpulleyinitial()
        self.calc_lengthwirefixedend2springpulleyinitial()
        self.calc_lengthwirespringpulley2motorinitial()
        self.calc_lengthwholewire()
        self.calc_lengthwholewireinitial()
        self.calc_lengthwirechange()
        self.calc_lengthwirechange2stepmotorangle()
    
    def calc_lengthwirechange2stepmotorangle(self): # delta_x
        # ワイヤの長さ変化からモータのステップ角を計算する
        self.step_motorangle = (self.step_1lap / (2 * np.pi)) * (self.length_wire_change / self.radius_motor + self.theta_wire_springpulley2motor - self.theta_wire_springpulley2motor_initial)
        print(f"delta_x: {self.step_motorangle}\n")
    
    def calc_lengthwirechange(self): # delta_l
        # ワイヤの長さ変化を計算する
        self.length_wire_change = self.length_wire_whole_initial - self.length_wire_whole
        print(f"delta_l: {self.length_wire_change*1000}")
    
    def calc_lengthwholewireinitial(self): # L'
        # ワイヤの初期長さを計算する
        self.length_wire_whole_initial = self.length_wire_springpulley2motor_initial + self.length_wire_fixedend2springpulley_initial + self.length_wire_springpulley_initial
        print(f"L': {self.length_wire_whole_initial*1000}")
    
    def calc_lengthwholewire(self): # L
        # ワイヤの長さを計算する
        self.length_wire_whole = self.length_wire_springpulley2motor + self.length_wire_fixedend2springpulley + self.length_wire_springpulley
        print(f"L: {self.length_wire_whole*1000}")
    
    def calc_lengthwirespringpulley2motorinitial(self): # l_i'
        # ばねプーリとモータ間のワイヤの初期長さを計算する
        self.length_wire_springpulley2motor_initial = np.sqrt(self.length_springpulley2motor_initial**2 - (self.radius_motor - self.radius_springpulley)**2)
        print(f"l_i': {self.length_wire_springpulley2motor_initial*1000}")
    
    def calc_lengthwirefixedend2springpulleyinitial(self): # l_o'
        # 固定端とばねプーリ間のワイヤの初期長さを計算する
        self.length_wire_fixedend2springpulley_initial = - self.radius_fixedend + np.sqrt(self.length_fixedend2springpulley_initial**2 - self.radius_springpulley**2)
        print(f"l_o': {self.length_wire_fixedend2springpulley_initial*1000}")
    
    def calc_lengthwirespringpulleyinitial(self): # l_s'
        # ばねプーリに巻き付くワイヤの初期長さを計算する
        self.length_wire_springpulley_initial = self.radius_springpulley * (self.theta_wire_springpulley2motor_initial + self.theta_wire_fixedend2springpulley_initial)
        print(f"l_s': {self.length_wire_springpulley_initial*1000}")
    
    def calc_lengthwirespringpulley2motor(self): # l_i
        # ばねプーリとモータ間のワイヤの長さを計算する
        self.length_wire_springpulley2motor = np.sqrt(self.length_springpulley2motor**2 - (self.radius_motor - self.radius_springpulley)**2)
        print(f"l_i: {self.length_wire_springpulley2motor*1000}")
    
    def calc_lengthwirefixedend2springpulley(self): # l_o
        # 固定端とばねプーリ間のワイヤの長さを計算する
        self.length_wire_fixedend2springpulley = - self.radius_fixedend + np.sqrt(self.length_fixedend2springpulley**2 - self.radius_springpulley**2)
        print(f"l_o: {self.length_wire_fixedend2springpulley*1000}")
    
    def calc_lengthwirespringpulley(self): # l_s
        # ばねプーリに巻き付くワイヤの長さを計算する
        self.length_wire_springpulley = self.radius_springpulley * (self.theta_wire_springpulley2motor + self.theta_wire_fixedend2springpulley)
        print(f"l_s: {self.length_wire_springpulley*1000}")
    
    def calc_thetawirespringpulley2motor(self): # theta_i
        # ばねプーリとモータ間のワイヤの角度を計算する
        # self.theta_wire_springpulley2motor = self.theta_springpulley2motor - np.arcsin((self.radius_motor - self.radius_springpulley) / self.length_springpulley2motor)
        self.theta_wire_springpulley2motor = -np.arctan2(self.radius_motor, np.sqrt(self.length_springpulley2motor**2-(self.radius_motor-self.radius_springpulley)**2)) + np.arccos((self.length_springpulley2motor/np.sqrt(self.length_springpulley2motor**2+self.radius_motor**2-(self.radius_motor-self.radius_springpulley)**2))*np.cos(self.theta_springpulley2motor))
        print(f"theta_i: {self.theta_wire_springpulley2motor*180/np.pi}")
    
    def calc_thetaspringpulley2motor(self): # theta_2
        # ばねプーリとモータの角度を計算する
        self.theta_springpulley2motor = np.arccos(self.distance_springpulley2motor_holizontal / self.length_springpulley2motor)
        print(f"theta_2: {self.theta_springpulley2motor*180/np.pi}")
    
    def calc_lengthspringpulley2motor(self): # l_2
        # ばねプーリとモータの長さを計算する
        self.length_springpulley2motor = np.sqrt((self.height_springpulley2motor_initial - self.spring_displacement)**2 + self.distance_springpulley2motor_holizontal**2)
        print(f"l_2: {self.length_springpulley2motor*1000}")
    
    def calc_heightspringpulley2motorinitial(self): # h_2
        # ばねプーリとモータの高さを計算する
        # self.height_springpulley2motor_initial = self.length_springpulley2motor_initial * np.sin(self.theta_wire_springpulley2motor_initial + np.arcsin((self.radius_motor - self.radius_springpulley)/self.length_springpulley2motor_initial))
        self.height_springpulley2motor_initial = np.sqrt(self.length_springpulley2motor_initial**2 - self.distance_springpulley2motor_holizontal**2)
        print(f"h_2: {self.height_springpulley2motor_initial*1000}")
    
    def calc_thetawirefixedend2springpulley(self): # theta_o
        # 固定端とばねプーリ間のワイヤ角度を計算する
        self.theta_wire_fixedend2springpulley = self.theta_fixedend2springpulley + np.arcsin(self.radius_springpulley / self.length_fixedend2springpulley)
        print(f"theta_o: {self.theta_wire_fixedend2springpulley*180/np.pi}")
    
    def calc_thetafixedend2springpulley(self): # theta_1
        # 固定端とばねプーリ間の角度を計算する
        self.theta_fixedend2springpulley = np.arccos(self.distance_fixedend2springpulley_holizontal / self.length_fixedend2springpulley)
        print(f"theta_1: {self.theta_fixedend2springpulley*180/np.pi}")

    def calc_lengthfixedend2springpulley(self): # l_1
        # 固定端とばねプーリの長さを計算する
        self.length_fixedend2springpulley = np.sqrt((self.height_fixedend2springpulley - self.spring_displacement)**2 + self.distance_fixedend2springpulley_holizontal**2)
        print(f"l_1: {self.length_fixedend2springpulley*1000}")
    
    def calc_heightfixedend2springpulley(self): # h_1
        # 固定端とばねプーリの高さを計算する
        # self.height_fixedend2springpulley = self.length_fixedend2springpulley_initial * np.sin(self.theta_wire_fixedend2springpulley_initial - np.arcsin(self.radius_springpulley/self.length_fixedend2springpulley_initial))
        self.height_fixedend2springpulley = np.sqrt(self.length_fixedend2springpulley_initial**2 - self.distance_fixedend2springpulley_holizontal**2)
        print(f"h_1: {self.height_fixedend2springpulley*1000}")

if __name__ == "__main__":
    # param motor
    step_1lap = 4096  # モータの1回転あたりのステップ数 (step)
    # param dynamics
    length_fixedend2springpulley_initial = 80              # l_1' (mm)
    length_springpulley2motor_initial = 56                # l_2' (mm)
    theta_wire_springpulley2motor_initial = 5             # theta_i' (deg)
    theta_wire_fixedend2springpulley_initial = 15          # theta_o' (deg)
    distance_fixedend2springpulley_holizontal = 77         # d_1 (mm)
    distance_springpulley2motor_holizontal = 54           # d_2 (mm)
    radius_springpulley = 3                              # r_s (mm)
    radius_fixedend = 2                                  # r_f (mm)
    radius_motor = 7.25                                     # r_m (mm)
    # param plot
    spring_displacement_limit = [0.0, 3.0]  # ばね変位のLimit (mm)
    plot_increment = 0.05  # プロットの間隔 (mm)
    StepAngle2SpringDist = StepAngle2SpringDist(
        step_1lap,
        length_fixedend2springpulley_initial,
        length_springpulley2motor_initial,
        theta_wire_fixedend2springpulley_initial,
        theta_wire_springpulley2motor_initial,
        distance_fixedend2springpulley_holizontal,
        distance_springpulley2motor_holizontal,
        radius_springpulley,
        radius_fixedend,
        radius_motor,
        spring_displacement_limit,
        plot_increment
    )
    StepAngle2SpringDist.calculate_stepmotorangle()