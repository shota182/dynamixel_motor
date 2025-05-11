import yaml
import numpy as np

# YAMLファイルを開く
with open('/home/sskr3/ros_ws/acm_ws/b9se5k30wa5_spe.yaml', 'r') as file:
    # YAMLを読み込む
    data = yaml.safe_load(file)

# 読み込んだデータを表示
label10_value = data.get('label10', None)
print(label10_value)
input_arr_spe = np.dot(np.deg2rad(label10_value["kappa"]),label10_value["input_arr_spe"])

print(input_arr_spe)