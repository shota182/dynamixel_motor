# 変位からばね定数

import numpy as np
import sympy as sp
from tqdm import tqdm
import matplotlib.pyplot as plt

# 検査項目
d2theta = False
bool_print = True
bool_placevisualize = True
bool_z2T = True
#

# 条件
z_range = [1, 10] # 距離[mm]
I_range = [0, 1470] # 電流の最小と最大[mA]
d_pulley = 15 # プーリ直径[mm]
#

# dとthetaの関係
if(d2theta):
    ## d -> theta
    d = 50 # センサからワイヤ端点[mm]
    z_ring = 0 # z軸高さ[mm]
    theta_range_0 = np.rad2deg(np.arctan2(-z_ring+z_range[0], d)) # 下端[deg]
    theta_range_1 = np.rad2deg(np.arctan2(-z_ring+z_range[1], d)) # 上端[deg]
    theta_range = [theta_range_0, theta_range_1]
else:
    ## theta -> d
    theta_range = [10, 30] # [deg]
    d = (z_range[0]-z_range[1]) / (np.tan(np.deg2rad(theta_range[0]))-np.tan(np.deg2rad(theta_range[1])))
    z_ring = z_range[1] - (z_range[1]-z_range[0]) / (1-(np.tan(np.deg2rad(theta_range[0]))/np.tan(np.deg2rad(theta_range[1]))))
#

# 張力最大値
## 電流とトルクが原点を通る比例とみなす
Torque_max = 0.52 * (I_range[1]/1470) # トルク[Nm]
## プーリ径を加えて張力の最大値を計算
T_max = Torque_max / (d_pulley/2 * 10**-3) # 張力[N]
#
# 実際の張力レンジ
T_range = [2*T_max*np.sin(np.deg2rad(theta_range[0])), 2*T_max*np.sin(np.deg2rad(theta_range[1]))]
# ばね定数の計算
k = T_range[0] / (z_range[1]-z_range[0])
#

# print
if(bool_print):
    print(f"z[mm]            : {z_range[0]} ~ {z_range[1]}")
    print(f"d[mm]            : {d}")
    print(f"z_ring[mm]       : {z_ring}")
    print(f"theta[deg]       : {theta_range[0]} ~ {theta_range[1]}")
    print(f"Tension_max[N]   : {T_max}")
    print(f"Tension range[N] : {T_range[1]} ~ {T_range[0]}")
    print(f"k[N/mm]          : {k}")
#

# 配置
if(bool_placevisualize):
    plt.figure()
    plt.axes().set_aspect("equal")
    ## axis
    plt.plot([0,0], [-1, z_range[1]+2], "-.", color="gray", lw=0.5)
    plt.plot([-d-1, 5+2], [0,0], "-.", color="gray", lw=0.5)
    ## sensor
    plt.plot([-5,-5,5,5, -5], [-2,-1,-1,-2, -2], color="limegreen")
    plt.plot([-1,-1,1,1, -1], [-1,0,0,-1, -1], color="black")
    ## ring
    plt.plot([0, -d, 0], [z_range[1], z_ring, z_range[0]], "o-", color="red")
    ## moving range
    plt.arrow(0+1,z_range[0], 0,z_range[1]-z_range[0]-0.3, lw=2, head_width=0.3, head_length=0.3, fc='red', ec='red')
    plt.arrow(0+1,z_range[1], 0,z_range[0]-z_range[1]+0.3, lw=2, head_width=0.3, head_length=0.3, fc='red', ec='red')
    plt.show()
#

# 距離 to 張力
if(bool_z2T):
    ## calc
    z_list = np.linspace(z_range[0], z_range[1], 100)
    theta_list = np.arctan2(-z_ring+z_list, d) # [rad]
    T_list = np.array([k*(z_range[1]-z_list[i]) / (2*np.sin(theta_list[i])) for i in range(100)])
    I_list = T_list*(d_pulley/2 * 10**-3) * 1470/0.52
    ## plot
    fig, ax1 = plt.subplots()
    plt.title(f"T[N](-z - axis) and z[mm]")
    ax1.set_xlabel("T [N]")
    ax1.set_ylabel("z [mm]")
    ax2 = ax1.twiny()
    ax2.plot(I_list, z_list, ".-", color="blue", alpha=0)
    ax2.set_xlabel("I [mA]")
    ax2.xaxis.set_label_position('bottom')
    ax2.spines['bottom'].set_position(('outward', 50))
    ax2.xaxis.tick_bottom()
    # plt.xlim([0, 30])
    plt.ylim([0,z_range[1]+1])
    ax1.plot(T_list, z_list, ".-", color="red")
    plt.grid()
    plt.tight_layout()
    plt.show()
#


# # theta制約範囲
# if(bool_thetarange):
#     theta_0_list = [30, 45, 60]
#     colorlist = ["red", "blue", "green", "yellow", "orange", "magenta", "skyblue", "lightgreen", "purple"]
#     z = np.linspace(z[0], z[-1], 100)
#     plt.figure()
#     plt.title(f"z[mm] and theta[deg]")
#     plt.xlabel("z [mm]")
#     plt.ylabel("theta [deg]")
#     plt.grid()
#     for i in range(len(theta_0_list)):
#         theta = np.arctan2((z-(z[-1]-x_w*np.tan(np.radians(theta_0_list[i])))), x_w)
#         plt.plot(z, np.degrees(theta), label=f"theta0 = {theta_0_list[i]}[deg]", color=colorlist[i])
#     plt.legend()
#     plt.tight_layout()
#     plt.show()
# #

# # 変位が変わった時の張力
# if(bool_z2T):
#     z = np.linspace(z[0], z[-1], 100)
#     T_look = T[-1]*np.sin( np.arctan2((z-(z[-1]-x_w*np.tan(np.radians(theta_0)))), x_w) )
#     print(f"張力の ばねを引っ張る方向の成分: {T_look[0]}[N] ~ {T_look[-1]}[N]")
#     colorlist = ["red", "blue", "green", "yellow", "orange", "magenta", "skyblue", "lightgreen", "purple", "black"]
#     plt.figure()
#     plt.title(f"T[N](-z - axis) and z[mm]")
#     plt.xlabel("T [N]")
#     plt.ylabel("z [mm]")
#     plt.xlim([0, 30])
#     plt.ylim([0,12])
#     plt.grid()
#     plt.plot(T_look, z, ".-", color="red")
#     plt.tight_layout()
#     plt.show()
# #

# # ばね定数導出
# T_list = np.linspace(T[0], T[-1], 10)
# if(bool_kspring):
#     colorlist = ["red", "blue", "green", "yellow", "orange", "magenta", "skyblue", "lightgreen", "purple", "black"]
#     plt.figure()
#     plt.title(f"T[N] and k[N/mm]")
#     plt.xlabel("Tmax [N]")
#     plt.ylabel("k [N/mm]")
#     plt.grid()
#     ## 最大張力が作用したときに ばねが一番下に下がった状態　を仮定したときのばね定数
#     ## 張力とzが比例じゃないが短調なのでok?
#     k = (T_list)/(z[-1]-z[0]) * np.sin( np.arctan2((z[0]-(z[-1]-x_w*np.tan(np.radians(theta_0)))), x_w) ) # N/mm
#     plt.plot(T_list, k, ".-", color="red")
#     plt.tight_layout()
#     plt.show()
# else:
#     k = (T[-1])/(z[-1]-z[0]) * np.sin( np.arctan2((z[0]-(z[-1]-x_w*np.tan(np.radians(theta_0)))), x_w) )
#     print(f"k: {k}")
# #

# # Iからk
# if(bool_current2spring):
#     I_list = np.linspace(I[0], I[1], 10)
#     Torque_c2s = [[0.52 * (I[0]/1470), 0.52 * (current/1470)] for current in I_list] # トルク[Nm]
#     T_c2s = np.array([[t[0] / (d_pulley/2 * 10**-3), t[-1] / (d_pulley/2 * 10**-3)] for t in Torque_c2s]) # 張力[N]
#     k = (T_c2s[:,1])/(z[-1]-z[0]) * np.sin( np.arctan2((z[0]-(z[-1]-x_w*np.tan(np.radians(theta_0)))), x_w) )
#     plt.figure()
#     plt.title(f"I[mA] and k[N/m] #theta0={theta_0}[deg]")
#     plt.xlabel("I [mA]")
#     plt.ylabel("k [N/m]")
#     plt.grid()
#     plt.plot(I_list, k, ".-", color="red")
#     plt.tight_layout()
#     plt.show()
# #

# # Tからz
# if(bool_z):
#     z_list = []
#     k = (T[-1])/(z[-1]-z[0]) * np.sin( np.arctan2((z[0]-(z[-1]-x_w*np.tan(np.radians(theta_0)))), x_w) )
#     for i in tqdm(range(len(T_list))):
#         z_re = sp.symbols("z")
#         equation = sp.Eq((T_list[i])/(z[-1]-z_re) * sp.sin( sp.atan2((z_re-(z[-1]-x_w*sp.tan(theta_0*sp.pi/180))), x_w) ), k)
#         solutions = sp.solve(equation, z_re)
#         try:
#             z_list.append(float(solutions[0]))
#         except:
#             z_list.append(z[-1])
#     z_list = np.ravel(z_list)
#     plt.figure()
#     plt.title(f"T[N] and z[mm] #theta0={theta_0}[deg]")
#     plt.xlabel("T [N]")
#     plt.ylabel("z [mm]")
#     plt.grid()
#     k = (2*T_list)/(z[-1]-z[0]) * np.sin( np.arctan2((z[0]-(z[-1]-x_w*np.tan(np.radians(theta_0)))), x_w) )
#     plt.plot(T_list, z_list, ".-", color="red")
#     plt.tight_layout()
#     plt.show()
# #