#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
process_columns.py

使い方:
    python process_columns.py input.csv
"""

import sys
from pathlib import Path

import pandas as pd

def main(csv_path: Path) -> None:
    # 1) CSV 読み込み
    df = pd.read_csv(csv_path, skipinitialspace=True)

    # 2) １列目を (最大値 – 各値) に変換
    col0 = df.columns[0]
    max0 = df[col0].max()
    df[col0] = max0 - df[col0]

    # 3) ２列目を (各値 – 最小値) に変換
    col1 = df.columns[1]
    min1 = df[col1].min()
    df[col1] = df[col1] - min1

    # 4) 保存
    output_path = csv_path.with_name(csv_path.stem + "_processed.csv")
    df.to_csv(output_path, index=False)
    print(f"Processed CSV saved to: {output_path}")

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python process_columns.py input.csv")
        sys.exit(1)

    csv_file = Path(sys.argv[1])
    if not csv_file.is_file():
        print(f"Error: {csv_file} not found.")
        sys.exit(1)

    main(csv_file)
