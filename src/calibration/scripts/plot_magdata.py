import os
import glob
import pandas as pd
import sys
import matplotlib.pyplot as plt
from bisect import bisect_left

MAG2_COLUMN = "mag4"  # mag2列名を変数化
SPRING_DISPLACEMENT_COLUMN = "spring_displacement"  # spring_displacement列名を変数化

def interpolate_spring_for_mag2(ref_df, target_mag2):
    df_sorted = ref_df.sort_values(MAG2_COLUMN).reset_index(drop=True)
    mag2_list = df_sorted[MAG2_COLUMN].to_numpy()
    spring_list = df_sorted[SPRING_DISPLACEMENT_COLUMN].to_numpy()

    pos = bisect_left(mag2_list, target_mag2)

    if pos == 0:
        return spring_list[0]
    if pos == len(mag2_list):
        return spring_list[-1]

    x0, x1 = mag2_list[pos - 1], mag2_list[pos]
    y0, y1 = spring_list[pos - 1], spring_list[pos]

    # 線形内挿
    ratio = (target_mag2 - x0) / (x1 - x0)
    return y0 + ratio * (y1 - y0)


def load_csv_with_head_mag2(csv_file):
    try:
        df = pd.read_csv(csv_file)
        print(f" → {csv_file} 読み込み成功。列名: {df.columns.tolist()}")
        if all(col in df.columns for col in [SPRING_DISPLACEMENT_COLUMN, MAG2_COLUMN]):
            head_mag2 = df.iloc[0][MAG2_COLUMN]
            return head_mag2, df
        else:
            print(f" → 必要な列が存在しません: {csv_file}")
    except Exception as e:
        print(f" → Error reading {csv_file}: {e}")
    return None, None


def align_csvs_by_mag2(csv_dir):
    csv_files = sorted(glob.glob(os.path.join(csv_dir, "*.csv")))

    file_info = []
    for f in csv_files:
        mag2, df = load_csv_with_head_mag2(f)
        if mag2 is not None:
            file_info.append((mag2, f, df))

    # 初期mag2で昇順ソート
    file_info.sort(key=lambda x: x[0])

    aligned_data = []
    merged_reference = pd.DataFrame()

    for i, (head_mag2, path, df) in enumerate(file_info):
        df = df.copy()
        print(f"[{i}] Processing: {path}, head_mag2 = {head_mag2}")

        if i == 0:
            df[SPRING_DISPLACEMENT_COLUMN] -= df.iloc[0][SPRING_DISPLACEMENT_COLUMN]
            merged_reference = df.copy()
        else:
            match = merged_reference[merged_reference[MAG2_COLUMN] == head_mag2]

            if not match.empty:
                # 完全一致が見つかった場合 → そのspringを使う
                target_spring = match.iloc[0][SPRING_DISPLACEMENT_COLUMN]
                method = "exact"
            else:
                # 一致なし → 内挿を試みる
                try:
                    target_spring = interpolate_spring_for_mag2(merged_reference, head_mag2)
                    method = "interpolated"
                except Exception as e:
                    print(f" → ERROR: Could not interpolate mag2={head_mag2}: {e}")
                    method = None

            if method:
                shift = target_spring - df.iloc[0][SPRING_DISPLACEMENT_COLUMN]
                df[SPRING_DISPLACEMENT_COLUMN] += shift
                print(f" → {method.capitalize()} alignment for mag2={head_mag2:.2f} → shift={shift:.4f}")
            else:
                print(f" → WARNING: Failed to align mag2={head_mag2}. Using unshifted.")

            merged_reference = pd.concat([merged_reference, df], ignore_index=True)
            aligned_data.append((os.path.basename(path), df))

    return aligned_data

def save_merged_mean_mag2(aligned_data, output_csv="merged_mean_mag2.csv"):
    # aligned_data: List of (label, df)
    dfs = []
    for label, df in aligned_data:
        # spring_displacementとmag2だけ使う
        dfs.append(df[[SPRING_DISPLACEMENT_COLUMN, MAG2_COLUMN]])
    merged = pd.concat(dfs, ignore_index=True)
    
    # mag2ごとにspring_displacementの平均を計算
    grouped = merged.groupby(MAG2_COLUMN, as_index=False).mean()
    grouped = grouped.sort_values(MAG2_COLUMN)  # mag2でソート
    grouped.to_csv(output_csv, index=False)
    print(f"保存しました: {output_csv}")

    # プロット
    plt.figure()

    # 元のデータをプロット
    for label, df in aligned_data:
        plt.plot(df[SPRING_DISPLACEMENT_COLUMN].to_numpy(), df[MAG2_COLUMN].to_numpy(), alpha=0.5, label=f"Original: {label}")

    # 平均値をプロット
    plt.plot(grouped[SPRING_DISPLACEMENT_COLUMN].to_numpy(), grouped[MAG2_COLUMN].to_numpy(), marker='o', color='red', label="Mean Mag2")

    # ラベルやタイトルを設定
    plt.xlabel("Spring Displacement")
    plt.ylabel("Mag2")
    plt.title("Mean Mag2 vs Spring Displacement (Grouped by Mag2)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 plot_csvs_align_by_mag2.py <csv_directory>")
        sys.exit(1)

    aligned_list = align_csvs_by_mag2(sys.argv[1])

    # ここで平均化＆保存
    save_merged_mean_mag2(aligned_list)
