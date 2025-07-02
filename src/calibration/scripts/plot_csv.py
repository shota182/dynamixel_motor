import pandas as pd
import matplotlib.pyplot as plt
import sys
import numpy as np

# 固定列番号
X_COLUMN_INDEX = 0  # X軸に使用する列番号
Y_COLUMN_INDEX = 1  # Y軸に使用する列番号

def adjust_and_plot(csv1):
    # CSVファイルを読み込む
    df1 = pd.read_csv(csv1, header=None)  # ヘッダーなしで読み込む

    # 列番号でデータを取得し、数値型に変換
    x1 = pd.to_numeric(df1.iloc[:, X_COLUMN_INDEX], errors='coerce').to_numpy()
    y1 = pd.to_numeric(df1.iloc[:, Y_COLUMN_INDEX], errors='coerce').to_numpy()

    # NaNを除外
    x1 = x1[~np.isnan(x1)]
    y1 = y1[~np.isnan(y1)]

    # 列番号0の値を比較し、小さい方を先にプロット
    first_x, first_y = x1, y1
    
    # プロット
    plt.figure(figsize=(10, 6))

    # 小さい方をプロット
    plt.plot(first_x, first_y, label="Smaller First", color="red")

    # ラベルやタイトルを設定
    plt.xlabel(f"Column {X_COLUMN_INDEX}")
    plt.ylabel(f"Column {Y_COLUMN_INDEX}")
    plt.title(f"Comparison of Column {Y_COLUMN_INDEX} vs Column {X_COLUMN_INDEX}")
    plt.legend()

    plt.grid(True)
    plt.tight_layout()

    # プロットを表示
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 plot_double.py <csv1>")
        sys.exit(1)

    csv1 = sys.argv[1]

    adjust_and_plot(csv1)