import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def process_csv(csv_path):
    # CSVファイルを読み込む
    data = pd.read_csv(csv_path)

    # spring_displacement_fitの最大値と最小値を取得
    max_value = data.loc[data['mag4'] == 615, 'spring_displacement_fit'].values[0]
    min_value = data.loc[data['mag4'] == 260, 'spring_displacement_fit'].values[0]

    # 差分が2になるように正規化
    scale_factor = 2 / (max_value - min_value)
    data['spring_displacement_fit'] *= scale_factor
    max_disp = data['spring_displacement_fit'].max() + 0.5
    data['spring_displacement_fit'] = max_disp - data['spring_displacement_fit']

    # ndarrayに最大値と最小値を格納
    result = np.array([data['spring_displacement_fit'].min(), data['spring_displacement_fit'].max()])
    return data, result

def save_to_csv(data, output_path):
    # 正規化後のデータをCSVに保存
    data.to_csv(output_path, index=False)
    print(f"Normalized data saved to {output_path}")

def plot_data(data):
    # プロット
    plt.figure(figsize=(10, 6))
    plt.plot(data['mag4'].values, data['spring_displacement_fit'].values, label='Spring Displacement Fit', color='blue')
    plt.xlabel('mag4')
    plt.ylabel('spring_displacement_fit')
    plt.title('Spring Displacement Fit Plot (Normalized)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # CSVファイルのパスを指定
    input_csv_path = "/home/sskr3/acm_ws/src/calibration/csv/2_merged_mean_mag2_monotonic_fit.csv"
    output_csv_path = "/home/sskr3/acm_ws/src/calibration/csv/normalized_data.csv"

    # 処理を実行
    modified_data, result = process_csv(input_csv_path)
    print("Min and Max values after normalization:", result)

    # 正規化後のデータを保存
    if(1):
        save_to_csv(modified_data, output_csv_path)

    # プロット
    plot_data(modified_data)