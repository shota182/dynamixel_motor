from dynamixel_sdk import *  # Uses Dynamixel SDK library
import time

def convert_to_signed(value, bits=16):
    if value >= 2**(bits-1):
        value -= 2**bits
    return value

# Protocol version
PROTOCOL_VERSION = 2.0

# Default setting
DXL_ID = 7                   # Dynamixel ID
BAUDRATE = 57600             # Dynamixel default baudrate
DEVICENAME = '/dev/ttyUSB0'  # Check which port is being used on your controller

# Control Table Addresses
ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_GOAL_POSITION = 116
ADDR_GOAL_CURRENT = 102
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_CURRENT = 126

# Operating Modes
CURRENT_BASED_POSITION_MODE = 5  # Check the correct value from the e-Manual

# Dynamixel Values
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 4095
DXL_MOVING_STATUS_THRESHOLD = 20

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

# Disable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("Failed to disable torque: %s" % packetHandler.getTxRxResult(dxl_comm_result))

# Set operating mode to current-based position control
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, CURRENT_BASED_POSITION_MODE)
if dxl_comm_result != COMM_SUCCESS:
    print("Failed to change operating mode: %s" % packetHandler.getTxRxResult(dxl_comm_result))

# Enable Dynamixel Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("Failed to enable torque: %s" % packetHandler.getTxRxResult(dxl_comm_result))

# Set goal position and current
goal_position = 800  # Middle position
goal_current = 50    # Some appropriate value for current
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, goal_position)
if dxl_comm_result != COMM_SUCCESS:
    print("Failed to set goal position: %s" % packetHandler.getTxRxResult(dxl_comm_result))

dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_CURRENT, goal_current)
if dxl_comm_result != COMM_SUCCESS:
    print("Failed to set goal current: %s" % packetHandler.getTxRxResult(dxl_comm_result))

try:
    while 1:
        # # Read present position
        # dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        # if dxl_comm_result != COMM_SUCCESS:
        #     print("Failed to read present position")
        #     break
        # signed_position = convert_to_signed(dxl_present_position, 32)

        # # Read present current
        # dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_CURRENT)
        # if dxl_comm_result != COMM_SUCCESS:
        #     print("Failed to read present current")
        #     break
        # signed_current = convert_to_signed(dxl_present_current, 16)
        # print(f"{'Position':>10} {signed_position:10} {'Current':>10} {signed_current:10}")
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    # Close port
    portHandler.closePort()
