import time
from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
DXL1_ID = 1       # Dynamixel#1 ID
DXL2_ID = 2       # Dynamixel#2 ID
BAUDRATE = 57600  # Dynamixel default baudrate : 57600
DEVICENAME = '/dev/ttyUSB0'  # Check which port is being used on your controller

ADDR_PRO_OPERATING_MODE = 11
CURRENT_CONTROL_MODE = 0  # Value for current control mode
ADDR_PRO_GOAL_CURRENT = 102
ADDR_PRO_TORQUE_ENABLE = 64
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel has been successfully connected")



# Set Operating Mode to Current Control mode
packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_OPERATING_MODE, CURRENT_CONTROL_MODE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Operating mode set to current control.")
packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_OPERATING_MODE, CURRENT_CONTROL_MODE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Operating mode set to current control.")



# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_CURRENT, 2)

# Set Goal Current
dxl_goal_current = [50, 60]  # Goal current for DXL1 and DXL2
param_goal_current = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_current[0])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_current[0]))]
groupSyncWrite.addParam(DXL1_ID, param_goal_current)
param_goal_current = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_current[1])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_current[1]))]
groupSyncWrite.addParam(DXL2_ID, param_goal_current)

# Syncwrite goal current
dxl_comm_result = groupSyncWrite.txPacket()
time.sleep(3)
if dxl_comm_result != COMM_SUCCESS:
    print(packetHandler.getTxRxResult(dxl_comm_result))

# Clear syncwrite parameter storage
groupSyncWrite.clearParam()

# Disable Torque for all Dynamixels
packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)

# Close port
portHandler.closePort()
