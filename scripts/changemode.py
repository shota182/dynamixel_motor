import os
from dynamixel_sdk import *                    # Dynamixel SDKのインポート


# ポート名とボーレートの設定
DEVICENAME          = "/dev/ttyUSB0"           # デバイス名 (Linuxの場合)
BAUDRATE            = 57600                    # ボーレート
PROTOCOL_VERSION    = 2.0                      # Dynamixelプロトコルバージョン

# Dynamixelのレジスタアドレス
ADDR_TORQUE_ENABLE  = 64                       # トルク有効のアドレス
ADDR_OPERATING_MODE = 11                       # 動作モードのアドレス
ADDR_REBOOT         = 6                        # リブートアドレス

# 動作モード
TORQUE_DISABLE      = 0                        # トルク無効
[CURRENT_MODE, VELOCITY_MODE, _, POSITION_MODE, EXTENDEDPOSITION_MODE, CURRENTBASEDPOSITION_MODE] = range(6)

# ポート初期化
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# ポートを開く
if portHandler.openPort():
    print("Succeeded to open the port!")
else:
    print("Failed to open the port!")
    quit()

# ボーレートを設定
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate!")
else:
    print("Failed to change the baudrate!")
    quit()

def main(id, operate_mode):
    for DXL_ID in id:
        # トルクを無効にする
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("1")
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("1")
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        # Operating Modeを位置制御モードに変更
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, operate_mode)
        if dxl_comm_result != COMM_SUCCESS:
            print("2")
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("2")
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        # モータを再起動
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_REBOOT, 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("3")
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("3")
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    # ポートを閉じる
    portHandler.closePort()

if __name__ == "__main__":
    id = range(10)
    main(id, POSITION_MODE)