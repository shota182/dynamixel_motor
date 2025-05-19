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

## ```mag_calibration```

### 概要

キャリブレーション用パッケージ
磁気センサ値から張力値を計算するためのノード．
パラメータを