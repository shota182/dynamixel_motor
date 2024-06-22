import os
import time
from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
DXL_IDs = range(1, 10)  # Dynamixel ID: 1 to 9
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'

# Dynamixel Control Table Addresses
ADDR_OPERATING_MODE = 11  # address for the operating mode
ADDR_GOAL_CURRENT = 102   # address for goal current
ADDR_TORQUE_ENABLE = 64   # address for torque enable

# Operating Modes
CURRENT_CONTROL_MODE = 0  # value for current control mode

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

# Set operating mode to current control mode and enable torque
for dxl_id in DXL_IDs:
    # Write operating mode
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_OPERATING_MODE, CURRENT_CONTROL_MODE)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to change operating mode for Dynamixel ID {dxl_id}")

    # Enable torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to enable torque for Dynamixel ID {dxl_id}")

    # Set goal current
    goal_current = 100  # Set appropriate value
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, dxl_id, ADDR_GOAL_CURRENT, goal_current)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to set goal current for Dynamixel ID {dxl_id}")

# Some operation, e.g., rotating for a while
print("Operating servos...")
time.sleep(2)  # Running time

# Disable torque
for dxl_id in DXL_IDs:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0)
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to disable torque for Dynamixel ID {dxl_id}")

# Close port
portHandler.closePort()
print("Finished!")

