# 2025/5/8

## ```mag_publisher```
### 概要

mag_publisherに存在。

アナログ入ピンのA0~A11を```std_msgs::Int32multiArray```としてpublish

### 実行

#### Arduino側

プログラムを書き込みする

#### PC側

* acm_ws

```
roscore
```

```
rosrun rosserial_python serial_node.py _port:=/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Due_Prog._Port_2423831343535160F0C1-if00 _baud:=115200
```
これでArduinoからtopicがpublishされる

---

## ```bilateral_test```
### 概要

追従制御，バイラテラルなど

### ```position_follow_1d_node.cpp```

モータ1つの連続体をマスタ・スレーブで動かす．
位置の追従制御．

モータの回転向きに注意すること．
設定ではどちらも値が減ると張力値が増える．

たるみ除去，大きい張力の除去機能を持つ．

### ```position_follow_2d_node.cpp```

モータ2つの連続体をマスタ・スレーブで動かす．
位置の追従制御．

↑のモータ4つバージョンだが，こちらもモータの回転向きに注意すること．
設定ではモータID=1, 3は値が減ると張力値が増える．
モータID=2，4は値が増えると張力値が増えるようにコーディングしている．

### ```bilateral_2d_node.cpp```

力逆走型P制御のバイラテラル制御．
↑と同じモータ設定．

#### 理論式

マスタ(リーダー)
$$
\tau_m = - K_f T_s + K_d (\theta_s - \theta_m)
$$

スレーブ(フォロワー)
$$
\tau_s = K_p (\theta_m - \theta_s)
$$

### ```mag_tension_converter.cpp```

磁気センサデータを張力に変換するノード．

#### パラメータ

パラメータはcsv_params.yamlで読み込む．
磁気センサの対象の要素番号にcsvファイルを合わせる．

### 実行

基本的に各launchで実行できる

バイラテラル一括⇓
```
roslaunch bilateral_test bilateral_control_2d_launcher.launch
```

---

## ```dynamixel_control```
### 概要

position制御でXL330-M288-Tを動かすノード。

### 構成

#### multi_pos_ctrl.launch

複数のモータを拡張位置制御で動かすプログラム

* device
ttyUSB0から絶対パスにした

* motor_count
使用するモータ数。
IDは1から連番で指定される。

* freq
位置，電流データの読み込み周波数

* goal_topic
入力位置。

* pos_topic
読み込んだ位置

* cur_topic
読み込んだ電流値

### 実行

```
roslaunch dynamixel_control multi_pos_ctrl.launch 
```

```
rostopic pub -1 /sensor/motor/input/position std_msgs/Int32MultiArray \
  "data: [0, 2048]"
```

* dataの配列にステップ入力値を入れて流す

---

## ```joy_to_dynamixel```

### 概要

joyでモータを動かすためのノード。

### ```config/params.yaml```

```axis_indices: [1,3]```：操作するモータID

```scale: 400.0```：速度(感度)

```motor_count: 2```：動かすモータ数

```control_freq: 50.0```：制御周波数

### 実行

```
roslaunch joy_to_dynamixel joy_to_extended_position.launch
```

---

## ```calibration```

キャリブレーション用パッケージ

### 実行順序

* ```python3 /home/sskr3/acm_ws/src/calibration/scripts/calc_springconstant.py```
ばね定数を計算してくれる．
2025/05/30時点のばねは0.67[N/m]．

* 固定端環境を用意して，計測した値を```step-angle2spring-dist.py```に書き込む

* ```python3 /home/sskr3/acm_ws/src/calibration/scripts/step-angle2spring-dist.py```
csvに```stepangle_vs_springdist_```から始まるcsvファイルが保存される．
これでモータのステップ角とばねの変位&張力の関係が求まる．

* ```roslaunch calibration zero_point_calibration.launch```
指定したモータの張力が0になるように0点キャリブレーションしてくれる．

* ```roslaunch calibration mag_calibration.launch```
ステップ角を走査して，磁気センサ値と張力の関係を```spring_vs_mag_```から始まるcsvに保存する．
これで磁気センサ値と張力の関係が求まる．

---

### ```zero_point_calibration_node.cpp```

張力の0点合わせをするためのノード．

#### 実行

```roslaunch calibration zero_point_calibration.launch```

#### パラメータ

```zero_point_calibration.yaml```に記載

### ```mag_calibration_node.cpp```

磁気センサのキャリブレーションをするためのノード．
```csv/stepangle_vs_springdist_20250530_134950.csv```形式のcsvファイルを読み込む．
出力は```csv/spring_vs_mag_20250530_133100.csv```形式で行われ，ばね定数，磁気センサ値，張力値が保存される．

#### 実行

```roslaunch calibration mag_calibration.launch```

#### パラメータ
```mag_calibration.yaml```に記載

---

### ```scripts/calc_springconstant.py```

板バネのパラメータから板バネのばね定数を計算する．

#### 実行

```python3 scripts/calc_springconstant.py```

### ```scripts/plot_tension-mag.py```

張力とばね変位，磁気センサ出力のプロットをする．
csvは```csv/spring_vs_mag_20250530_133100.csv```形式を選択する．

### ```step-angle2spring-dist.py```

モータステップ角の変化とばね変位の関係を計算するpythonファイル．
オフラインで計算して，csvに関係を格納する．

#### 実行

```python3 step-angle2spring-dist.py```
