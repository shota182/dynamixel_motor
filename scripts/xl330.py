import time
import numpy as np
from dynamixel_sdk import *  # Uses Dynamixel SDK library

class XL330:
    def __init__(self, ID_list):
        self.PROTOCOL_VERSION = 2.0
        self.ID_list = ID_list
        self.BAUDRATE = 57600  # Dynamixel default baudrate : 57600
        self.DEVICENAME = '/dev/ttyUSB0'  # Check which port is being used on your controller
        self.OPERATING_MODE = 2
        [self.CURRENT_MODE, self.VELOCITY_MODE, _, self.POSITION_MODE, self.EXTENDEDPOSITION_MODE, self.CURRENTBASEDPOSITION_MODE] = range(6)
        self.mode = ["current", "velocity", "", "position", "extended position", "current_based position"]

        self.ADDR_OPERATING_MODE = 11
        self.ADDR_GOAL_CURRENT = 102
        self.ADDR_GOAL_VELOCITY = 104
        self.ADDR_GOAL_POSITION = 116
        self.ADDR_TORQUE_ENABLE = 64
        self.TORQUE_ENABLE = 1
        self.TORQUE_DISABLE = 0

        self.ADDR_PRESENT_POSITION = 132     # 現在位置のアドレス

        # Initialize
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()

        # Set port self.baudrate
        if self.portHandler.setBaudRate(self.BAUDRATE):
            print("Succeeded to change the self.baudrate")
        else:
            print("Failed to change the self.baudrate")
            quit()
    
    def rxposition(self):# 現在位置を読み取る
        self.position_pre = list(range(len(self.ID_list)))
        for i in range(len(self.ID_list)):
            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, self.ID_list[i], self.ADDR_PRESENT_POSITION)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Current Position : %d" % dxl_present_position)
            self.position_pre[i] = dxl_present_position
    
    def setmode(self, MODE):
        if(self.OPERATING_MODE in [self.CURRENT_MODE, self.CURRENTBASEDPOSITION_MODE]):
            self.groupSyncWrite_cur.clearParam()
        if(self.OPERATING_MODE==self.VELOCITY_MODE):
            self.groupSyncWrite_vel.clearParam()
        if(self.OPERATING_MODE in [self.POSITION_MODE, self.EXTENDEDPOSITION_MODE, self.CURRENTBASEDPOSITION_MODE]):
            self.groupSyncWrite_pos.clearParam()
        self.OPERATING_MODE = MODE
        # Enable Dynamixel Torque # Set Operating Mode to Current Control mode
        for id in self.ID_list:
            # トルクを無効にする
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("1")
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("1")
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            # Operating Modeを電流制御モードに変更
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_OPERATING_MODE, self.OPERATING_MODE)
            if dxl_comm_result != COMM_SUCCESS:
                print("2")
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("2")
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print(f"ID {id} Dynamixel has been successfully connected")
            #
            self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_OPERATING_MODE, self.OPERATING_MODE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print(f"ID {id} Operating mode set to {self.mode[self.OPERATING_MODE]} control.")

            # Initialize self.GroupSyncWrite instance
            if(self.OPERATING_MODE in [self.CURRENT_MODE, self.CURRENTBASEDPOSITION_MODE]):
                self.groupSyncWrite_cur = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_CURRENT, 2)
            if(self.OPERATING_MODE==self.VELOCITY_MODE):
                self.groupSyncWrite_vel = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_VELOCITY, 4)
            if(self.OPERATING_MODE in [self.POSITION_MODE, self.EXTENDEDPOSITION_MODE, self.CURRENTBASEDPOSITION_MODE]):
                self.groupSyncWrite_pos = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, 4)

    def setcurrent(self, i):
        # Set Goal Current
        self.groupSyncWrite_cur.clearParam()
        for num in range(len(self.ID_list)):
            id = self.ID_list[num]
            ii = i[num]
            param_goal_current = [DXL_LOBYTE(DXL_LOWORD(ii)), DXL_HIBYTE(DXL_LOWORD(ii))]
            self.groupSyncWrite_cur.addParam(id, param_goal_current)
    
    def setposition(self, p):
        # Set Goal Position
        self.groupSyncWrite_pos.clearParam()
        for num in range(len(self.ID_list)):
            id = self.ID_list[num]
            pos = p[num]
            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(pos)), DXL_HIBYTE(DXL_LOWORD(pos)),
                                   DXL_LOBYTE(DXL_HIWORD(pos)), DXL_HIBYTE(DXL_HIWORD(pos))]
            self.groupSyncWrite_pos.addParam(id, param_goal_position)
            # print(f"set {pos} to ID:{id}")
    
    def setvelocity(self, v):
        # Set Goal Velocity
        self.groupSyncWrite_vel.clearParam()
        for num in range(len(self.ID_list)):
            id = self.ID_list[num]
            vel = v[num]
            param_goal_velocity = [DXL_LOBYTE(DXL_LOWORD(vel)), DXL_HIBYTE(DXL_LOWORD(vel)),
                                   DXL_LOBYTE(DXL_HIWORD(vel)), DXL_HIBYTE(DXL_HIWORD(vel))]
            self.groupSyncWrite_vel.addParam(id, param_goal_velocity)
    
    def tx(self, time_sleep):
        # Syncwrite goal current
        if(self.OPERATING_MODE in [self.CURRENT_MODE, self.CURRENTBASEDPOSITION_MODE]):
            print("current control")
            dxl_comm_result = self.groupSyncWrite_cur.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print(self.packetHandler.getTxRxResult(dxl_comm_result))
        if(self.OPERATING_MODE==self.VELOCITY_MODE):
            print("velocity control")
            dxl_comm_result = self.groupSyncWrite_vel.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print(self.packetHandler.getTxRxResult(dxl_comm_result))
        if(self.OPERATING_MODE in [self.POSITION_MODE, self.EXTENDEDPOSITION_MODE, self.CURRENTBASEDPOSITION_MODE]):
            print("position control")
            dxl_comm_result = self.groupSyncWrite_pos.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print(self.packetHandler.getTxRxResult(dxl_comm_result))
        if(time_sleep > 0): time.sleep(time_sleep)

    def __del__(self):
        # Clear syncwrite parameter storage
        if(self.OPERATING_MODE in [self.CURRENT_MODE, self.CURRENTBASEDPOSITION_MODE]):
            self.groupSyncWrite_cur.clearParam()
        if(self.OPERATING_MODE==self.VELOCITY_MODE):
            self.groupSyncWrite_vel.clearParam()
        if(self.OPERATING_MODE in [self.POSITION_MODE, self.EXTENDEDPOSITION_MODE, self.CURRENTBASEDPOSITION_MODE]):
            self.groupSyncWrite_pos.clearParam()
        
        # Disable Torque for all Dynamixels
        for id in self.ID_list:
            self.packetHandler.write1ByteTxRx(self.portHandler, id, self.ADDR_TORQUE_ENABLE, self.TORQUE_DISABLE)

        # Close port
        self.portHandler.closePort()
        print("Finished correctly.")

def main():
    Xl330 = XL330(range(1,10))
    Xl330.setmode(Xl330.CURRENT_MODE)
    Xl330.rxposition()
    Xl330.setcurrent([30]*9)
    # Xl330.setcurrent(np.tile([-25, -50, 50], 3))
    Xl330.tx(1.5)
if __name__ == "__main__":
    main()