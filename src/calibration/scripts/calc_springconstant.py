# calculate plate spring constant
import numpy as np
import matplotlib.pyplot as plt

class Spring:
    def __init__(self, t, h, A, L, X, ingredient):
        self.n = 1                      # ばねの枚数
        self.t = t                      # 厚さ[mm]
        self.h = h                      # 幅[mm]
        self.A = A                      # 穴径[mm2]
        self.L = L                      # 長さ[mm]
        self.X = X                      # 穴位置[mm]
        self.ingredient = ingredient    # 材料
        self.calc_param()               # 材料の物性値を計算
        self.spring_constant()          # ばね定数を計算

    def calc_param(self):                           # 材料の物性値を計算
        if(self.ingredient == "SUS304-CSP3/4H"):
            self.E = 186000                         # 縦弾性係数 [N/mm2]
            self.nyu = 0.3                          # ポアソン比
            self.sigma_max = 651                    # 使用最大応力 [N/mm2]
            self.f = 1.05                           # ばね定数の補正係数
        elif(self.ingredient == "SUS304-CSPH"):
            self.E = 186000                         # 縦弾性係数 [N/mm2]
            self.nyu = 0.3                          # ポアソン比
            self.sigma_max = 791                    # 使用最大応力 [N/mm2]
            self.f = 1.05                           # ばね定数の補正係数

    def spring_constant(self):                      # ばね定数を計算
        self.k = self.n * (self.E*self.h*self.t**3) / (4*self.L**3) * self.f
        print(f"ばね定数: {self.k:.2f} [N/mm]")
        

if __name__ == "__main__":
    t = 0.5
    h = 6
    A = 3
    L = 38
    X = 3
    ingredient = "SUS304-CSP3/4H"
    spring = Spring(t, h, A, L, X, ingredient)
    