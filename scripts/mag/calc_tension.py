import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import japanize_matplotlib

japanize_matplotlib.japanize()

plt.rcParams['font.size'] = 60
plt.rcParams['axes.unicode_minus'] = False  # マイナス符号対応

# CSVファイルの読み込み（ファイル名を適宜変更してください）
df = pd.read_csv('/home/sskr3/acm_ws/scripts/image_distance_processed.csv')  # 例：'yourfile.csv'

# dist_mmは最大値からの差分を計算する
displacement = df['dist_mm'].max() - df['dist_mm']

k=0.67
L_AB = 65.4
L_BC = 56.0
H_AB = 6.9
H_BC = 4.9
# Tの計算
df['T'] = k * displacement / (((H_AB-displacement) / L_AB) + ((H_BC-displacement) / L_BC))

x = df['mag'].to_numpy() * 3.3 / 1024
y = df['T'].to_numpy()
print(max(y))

# 線形近似（y = ax + b）
a, b = np.polyfit(x, y, 1)
y_fit = a * x + b
x_plot = np.array([0, 3.3])
y_plot = a * x_plot + b

# プロット
plt.figure(figsize=(8, 6))
plt.scatter(df['mag'].to_numpy()*3.3/1024, df['T'].to_numpy(), label="測定点", s=500, c="k", marker="o")
plt.plot(x_plot, y_plot, "--", label=f'Fit: T = {a:.2f}$V_H$ -{abs(b):.2f}', color='red', linewidth=5, zorder=0)
plt.xlim([0, 3.3])  # x軸の範囲を設定
plt.ylim([0, 20])  # x軸の範囲を設定
plt.xlabel('$V_H$ [V]', fontsize=60)
plt.ylabel('T [N]', fontsize=60)
plt.tick_params(axis='y', pad=20)   # デフォルトは 4 pt 程度
plt.tick_params(axis='x', pad=20)   # 必要なら x 側も微調整
plt.legend(fontsize=60)
# plt.title('T vs mag_fit')
plt.grid(True)
# plt.tight_layout()
plt.show()
