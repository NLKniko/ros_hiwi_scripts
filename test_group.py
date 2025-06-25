from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Control Table address (X-series: Present Position = 132, 4 bytes)
ADDR_PRESENT_POSITION = 132
LENGTH_PRESENT_POSITION = 4

# Protocol version and connection info
PROTOCOL_VERSION = 2.0
DXL_IDS = list(range(1, 12))          # IDs 1 through 11
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'           # Adjust as needed

# Initialize handlers
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LENGTH_PRESENT_POSITION)

# Open port
if not portHandler.openPort():
    print("[ERROR] Failed to open the port")
    quit()
print("Port opened successfully")

# Set baudrate
if not portHandler.setBaudRate(BAUDRATE):
    print("[ERROR] Failed to set baudrate")
    quit()
print("Baudrate set successfully")

# Add each Dynamixel ID to the group
for dxl_id in DXL_IDS:
    dxl_addparam_result = groupSyncRead.addParam(dxl_id)
    if not dxl_addparam_result:
        print(f"[ERROR] Failed to add ID {dxl_id} to groupSyncRead")
        quit()

# Send group read request
dxl_comm_result = groupSyncRead.txRxPacket()
if dxl_comm_result != COMM_SUCCESS:
    print(f"[ERROR] txRxPacket failed: {packetHandler.getTxRxResult(dxl_comm_result)}")
else:
    print("GroupSyncRead successful. Results:")

# Retrieve and print each servo's present position
for dxl_id in DXL_IDS:
    dxl_getdata_result = groupSyncRead.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LENGTH_PRESENT_POSITION)
    if not dxl_getdata_result:
        print(f"[ERROR] Data not available for ID {dxl_id}")
        continue

    position = groupSyncRead.getData(dxl_id, ADDR_PRESENT_POSITION, LENGTH_PRESENT_POSITION)
    print(f"ID {dxl_id}: Present Position = {position}")

# Clear parameters and close
groupSyncRead.clearParam()
portHandler.closePort()
