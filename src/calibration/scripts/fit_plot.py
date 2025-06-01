import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import sys

def main(csv_path):
    # CSVファイルを読み込む
    data = pd.read_csv(csv_path)

    # データを取得
    x = data.iloc[:, 0].values  # mag4
    y = data.iloc[:, 1].values  # spring_displacement

    # 単調増加を保証する最小二乗法の目的関数
    def monotonic_fit(params, x, y):
        # paramsは単調増加を保証するyの推定値
        y_fit = params
        # 単調増加を保証する制約
        penalty = np.sum(np.maximum(0, -(np.diff(y_fit))))  # 単調増加でない部分にペナルティを課す
        # 最小二乗法の誤差 + ペナルティ
        return np.sum((y - y_fit)**2) + penalty * 1e6

    # 初期値として元のyを使用
    initial_params = y.copy()

    # 最適化
    result = minimize(monotonic_fit, initial_params, args=(x, y), method="L-BFGS-B")

    # 最適化された単調増加のy値
    y_fit = result.x

    # グラフを描画
    plt.figure(figsize=(10, 6))
    plt.scatter(x, y, label="Original Data", color="blue", alpha=0.6)
    plt.plot(x, y_fit, label="Monotonic Fit", color="red", linewidth=2)
    plt.xlabel("mag4")
    plt.ylabel("spring_displacement")
    plt.title("Monotonic Fit using Least Squares")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # フィッティング結果を保存
    output_path = csv_path.replace(".csv", "_monotonic_fit.csv")
    output_data = pd.DataFrame({"mag4": x, "spring_displacement_fit": y_fit})
    output_data.to_csv(output_path, index=False)
    print(f"Fitted data saved to {output_path}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python fit_plot.py <csv_path>")
        sys.exit(1)

    csv_path = sys.argv[1]
    main(csv_path)