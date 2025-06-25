from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Control table address for your Dynamixel model (X-series shown here)
ADDR_PRESENT_POSITION = 132  # X-Series uses 4 bytes at address 132

PROTOCOL_VERSION = 2.0       # X-Series = 2.0, older MX/RX might be 1.0
DXL_ID = int(input("Give the servo's ID number: "))               # Change to your servo's ID
BAUDRATE = 1000000             # Adjust if your servo uses a different baud
DEVICENAME = "/dev/ttyUSB0"  # Change as needed (e.g., COM3 for Windows)

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if portHandler.openPort():
    print("Port opened successfully")
else:
    print("Failed to open port")
    quit()

# Set baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Baudrate set")
else:
    print("Failed to set baudrate")
    quit()

# Read Present Position (4 bytes)
position, comm_result, error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)

if comm_result != COMM_SUCCESS:
    print(f"Communication error: {packetHandler.getTxRxResult(comm_result)}")
elif error != 0:
    print(f"Servo error: {packetHandler.getRxPacketError(error)}")
else:
    print(f"Present Position of ID {DXL_ID}: {position}")

# Close port
portHandler.closePort()
