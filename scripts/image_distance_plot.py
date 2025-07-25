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

import sys
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
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

def main(csv_path: Path):
    # 1) CSV読み込み
    df = pd.read_csv(csv_path, skipinitialspace=True)

    # 2) 3mm換算距離
    df["dist_mm"] = df["dist"] * 3.0 / df["dist_standard"]

    # 3) 元データ保存
    processed_path = csv_path.with_name(csv_path.stem + "_processed.csv")
    df.to_csv(processed_path, index=False)
    print(f"Processed data saved to: {processed_path}")

    # 4) 指数近似フィット
    x = df["dist_mm"].to_numpy()
    y = df["mag"].to_numpy()
    y = y * 3.3 / 1024
    B, lnA = np.polyfit(x, np.log(y), 1)
    A = np.exp(lnA)

    # 5) フィット曲線生成／保存
    x_fit = np.linspace(0, x.max()*1.05, 200)
    y_fit = A * np.exp(B * x_fit)
    fit_curve_path = csv_path.with_name(csv_path.stem + "_fit_curve.csv")
    pd.DataFrame({"dist_mm": x_fit, "mag_fit": y_fit}).to_csv(fit_curve_path, index=False)
    print(f"Fit curve data saved to: {fit_curve_path}")

    # 6) プロット
    fig, ax = plt.subplots()
    ax.scatter(x, y, label="測定点", s=500, c="k", marker="o")
    ax.plot(x_fit, y_fit, "--",
            label=f"$ V_H = {A:.3}exp({B:.2}x)$",
            linewidth=5, zorder=0)

    # タイトル・ラベル・凡例のフォントサイズ調整
    # ax.set_title("磁気センサと磁石の距離に対する磁気センサ値", fontsize=TITLE_FONT_SIZE)
    ax.set_xlabel("d [mm]", fontsize=LABEL_FONT_SIZE)
    ax.set_ylabel("$V_H$ [V]", fontsize=LABEL_FONT_SIZE)
    ax.legend(fontsize=LEGEND_FONT_SIZE)

    ax.set_xlim([0, 3])
    ax.set_ylim([0, 3.3])

    ax.tick_params(axis='y', pad=20)   # デフォルトは 4 pt 程度
    ax.tick_params(axis='x', pad=20)   # 必要なら x 側も微調整

    ax.grid(True)
    # fig.tight_layout()
    plt.show()

    # 7) パラメータ表示
    print(f"フィットパラメータ: A={A:.6f}, B={B:.6f}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python plot_exp_fit.py data.csv")
        sys.exit(1)
    csv_file = Path(sys.argv[1])
    if not csv_file.is_file():
        print(f"Error: '{csv_file}' が見つかりません。")
        sys.exit(1)
    main(csv_file)
