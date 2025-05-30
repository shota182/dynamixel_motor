import pandas as pd
import matplotlib.pyplot as plt

# CSVファイル読み込み（ファイル名を適宜変更）
df = pd.read_csv('/home/sskr3/acm_ws/src/calibration/csv/spring_vs_mag_20250530_133100.csv')

# NaNのある行を除外し、必要な列の型変換を明示
df = df.dropna(subset=['interpolated_tension', 'mag2', 'spring_displacement'])
df['interpolated_tension'] = pd.to_numeric(df['interpolated_tension'], errors='coerce')
df['mag2'] = pd.to_numeric(df['mag2'], errors='coerce')
df['spring_displacement'] = pd.to_numeric(df['spring_displacement'], errors='coerce')

# プロット設定
fig, ax1 = plt.subplots()

# 左のY軸：mag2
color = 'tab:blue'
ax1.set_xlabel('Interpolated Tension [N]')
ax1.set_ylabel('Magnetic Field (mag2)', color=color)
ax1.plot(df['interpolated_tension'].values, df['mag2'].values, color=color, label='mag2')
ax1.tick_params(axis='y', labelcolor=color)

# 右のY軸：spring_displacement
ax2 = ax1.twinx()
color = 'tab:green'
ax2.set_ylabel('Spring Displacement [mm]', color=color)
ax2.plot(df['interpolated_tension'].values, (df['spring_displacement']).values, color=color, linestyle='--', label='spring_displacement')
ax2.tick_params(axis='y', labelcolor=color)

# タイトル・グリッド
plt.title('mag2 and Spring Displacement vs Tension')
plt.grid(True)
plt.tight_layout()
plt.show()
