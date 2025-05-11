# 2025/5/8

## mag_publisher
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

## dynamixel_control
### 概要

position制御でXL330-M288-Tを動かすノード。

### 構成

#### multi_pos_ctrl.launch

複数のモータを位置制御で動かすプログラム

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