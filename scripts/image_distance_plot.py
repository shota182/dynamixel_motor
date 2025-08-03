#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_exp_fit.py

使い方:
    python plot_exp_fit.py data.csv

説明:
    - CSV読み込み、3mm換算距離計算、データ保存
    - 指数近似フィット、フィットデータ保存
    - japanize_matplotlib で日本語化、フォントサイズカスタマイズ対応
"""


import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import sys
from pathlib import Path
import japanize_matplotlib

# === フォントサイズ設定 ===
BASE_FONT_SIZE  = 60  # 全体の基本フォントサイズ
TITLE_FONT_SIZE = 24  # グラフタイトル用フォントサイズ
LABEL_FONT_SIZE = 80  # 軸ラベル用フォントサイズ
LEGEND_FONT_SIZE= 80  # 凡例用フォントサイズ

# japanize_matplotlib により日本語フォント設定を自動で行う
japanize_matplotlib.japanize()
# フォントサイズを設定
plt.rcParams['font.size'] = BASE_FONT_SIZE
plt.rcParams['axes.unicode_minus'] = False  # マイナス符号対応
from scipy.optimize import curve_fit

# 真空の透磁率 [T·m/A]
MU_0 = 4 * np.pi * 1e-7


def magnetic_flux_density_fixed(z, M, L, R):
    """
    棒状磁石の軸方向磁束密度モデル (LとRは固定)
    z: 軸方向の距離 [mm]
    M: 磁化強度 [A/m]
    L: 磁石の長さ [mm] (固定)
    R: 磁石の半径 [mm] (固定)
    """
    z = z / 1000  # mm -> m
    L = L / 1000  # mm -> m
    R = R / 1000  # mm -> m
    term1 = (z + L / 2) / ((z + L / 2)**2 + R**2)
    term2 = (z - L / 2) / ((z - L / 2)**2 + R**2)
    return (MU_0 * M / 2) * (term1 - term2)

def dipole_approximation(z, a, b):
    """
    遠方場での双極子近似モデル
    z: 軸方向の距離 [mm]
    a: フィッティング係数
    b: オフセット距離 [mm]
    """
    return a / (z + b)**3

def main(csv_path):
    # 1) CSV読み込み
    df = pd.read_csv(csv_path)
    z = df["dist_mm"].to_numpy()  # 軸方向の距離 [mm]
    B_measured = df["mag"].to_numpy() * 3.3 / 1024  # 測定された磁束密度 [T]

    # 2) フィッティング (a, bをフィット)
    initial_guess = [1e6, 1]  # 初期値: [a, b]
    popt, pcov = curve_fit(dipole_approximation, z, B_measured, p0=initial_guess)
    a_fit, b_fit = popt

    # 3) フィット曲線生成
    z_fit = np.linspace(z.min(), z.max(), 500)
    B_fit = dipole_approximation(z_fit, a_fit, b_fit)

    # 4) プロット
    plt.figure(figsize=(10, 6))
    plt.scatter(z, B_measured, label="測定値", color="black", s=200)
    # plt.plot(z_fit, B_fit, "--", label=f"近似曲線: $a={a_fit:.0f}$, $b={b_fit:.2f}$ mm", color="red", linewidth=2, zorder=0)
    plt.xlabel("距離 d [mm]", fontsize=LEGEND_FONT_SIZE)
    plt.ylabel("ホール出力電圧 $V_H$ [V]", fontsize=LEGEND_FONT_SIZE)
    # plt.legend(fontsize=LEGEND_FONT_SIZE)
    plt.xlim([0, 3])
    plt.ylim([0, 3.3])
    plt.tick_params(axis='both', labelsize=60, pad=20)
    plt.grid(True)
    plt.show()

    # 5) フィット結果表示
    print(f"フィット結果:")
    print(f"  フィッティング係数 a = {a_fit:.2e}")
    print(f"  オフセット距離 b = {b_fit:.2f} [mm]")

if __name__ == "__main__":
    import sys
    from pathlib import Path

    if len(sys.argv) != 2:
        print("Usage: python magnetic_fit.py data.csv")
        sys.exit(1)

    csv_file = Path(sys.argv[1])
    if not csv_file.is_file():
        print(f"Error: '{csv_file}' が見つかりません。")
        sys.exit(1)

    main(csv_file)
