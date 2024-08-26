import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String, Int32
from dynamixel_sdk import *
import tty
import termios

# 시리얼 포트 설정
SERIAL_PORT = "/dev/ttyACM0"
SERIAL_SPEED = 115200

# Define constants
MY_DXL = 'X_SERIES'
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    LEN_GOAL_POSITION = 4  # Data Byte Length
    ADDR_PRESENT_POSITION = 132
    LEN_PRESENT_POSITION = 4  # Data Byte Length
    ADDR_PROFILE_VELOCITY = 112
    BAUDRATE = 57600
PROTOCOL_VERSION = 2.0
DXL1_ID = 7
DXL2_ID = 8
DXL3_ID = 9
DEVICENAME = '/dev/ttyACM0'
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MOVING_STATUS_THRESHOLD = 20

# Initialize port handler and packet handler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
# Initialize GroupSyncRead instance for Present Position
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

def initialize_dynamixels():
    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # Enable and initialize Dynamixel motors
    for dxl_id in [DXL1_ID, DXL2_ID, DXL3_ID]:
        enable_dynamixel_torque(dxl_id)
        set_dynamixel_profile_velocity(dxl_id, 100)
        move_dynamixel_to_position(dxl_id, 2048)

def enable_dynamixel_torque(dxl_id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % dxl_id)

    # Add parameter storage for Dynamixel present position value
    dxl_addparam_result = groupSyncRead.addParam(dxl_id)
    if not dxl_addparam_result:
        print("[ID:%03d] groupSyncRead addparam failed" % dxl_id)
        quit()

def disable_dynamixel_torque(dxl_id):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully disabled" % dxl_id)

def set_dynamixel_profile_velocity(dxl_id, velocity):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, dxl_id, ADDR_PROFILE_VELOCITY, velocity)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d profile velocity set to %d" % (dxl_id, velocity))

def move_dynamixel_to_position(dxl_id, position):
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(position)),
                           DXL_HIBYTE(DXL_LOWORD(position)),
                           DXL_LOBYTE(DXL_HIWORD(position)),
                           DXL_HIBYTE(DXL_HIWORD(position))]

    dxl_addparam_result = groupSyncWrite.addParam(dxl_id, param_goal_position)
    if not dxl_addparam_result:
        print("[ID:%03d] groupSyncWrite addparam failed" % dxl_id)
        quit()
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    groupSyncWrite.clearParam()

def read_dynamixel_position(dxl_id):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, dxl_id, ADDR_PRESENT_POSITION)
    return dxl_present_position

def oper(id, index):
    dxl_present_position = read_dynamixel_position(id)
    DXL_MINIMUM_POSITION_VALUE = dxl_present_position - 20
    DXL_MAXIMUM_POSITION_VALUE = dxl_present_position + 20
    dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]
    move_dynamixel_to_position(id, dxl_goal_position[index])

    while True:
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        dxl_getdata_result = groupSyncRead.isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if not dxl_getdata_result:
            print("[ID:%03d] groupSyncRead getdata failed" % id)
            quit()

        dxl_present_position = read_dynamixel_position(id)
        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (id, dxl_goal_position[index], dxl_present_position))
        if not (abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD):
            break

class DynamixelNode(Node):
    def __init__(self):
        super().__init__('dynamixel_node')
        
        # Subscribers
        self.subscription = self.create_subscription(Int32MultiArray, 'to_dynamixel', self.listener_callback, 10)
        self.manual_tracking_sub = self.create_subscription(String, 'camera_tracking_mode', self.camera_tracking_mode_callback, 10)
        self.manual_imu_sub = self.create_subscription(Int32MultiArray, 'manual_imu_data', self.manual_imu_callback, 10)
        self.tracking_mode_control_sub = self.create_subscription(String, 'robot_tracking_mode', self.robot_tracking_mode_callback, 10)

        self.yaw_position_pub = self.create_publisher(Int32, 'dynamixel_yaw', 10)

        # State Variables
        self.manual_mode = False

    def listener_callback(self, msg):
        if len(msg.data) >= 2:
            id = msg.data[0]
            index = msg.data[1]
            print("Received data - x:", id, "y:", index)

            if id == 101:
                id = DXL1_ID
            elif id == 102:
                id = DXL2_ID
            elif id == 103:
                id = DXL3_ID

            oper(id, index)

    def manual_imu_callback(self, msg):
        if self.manual_mode and len(msg.data) == 3:
            ang_pitch, ang_roll, ang_yaw= msg.data
            self.get_logger().info(f"Received manual IMU data: AngX={ang_pitch}, AngY={ang_roll}, AngZ={ang_yaw}")

            # yaw   left(ccw) : +, right(cw) : -
            if 0 < ang_yaw < 180:
                move_to_position = int(2048 + 11.3777*ang_yaw)
                move_dynamixel_to_position(DXL1_ID, move_to_position)  # yaw +
            else :
                move_to_position = int(2048 - 11.3777*ang_yaw)
                move_dynamixel_to_position(DXL1_ID, move_to_position)  # yaw -

            # pitch  up(ccw) : +, down(cw) : -
            if 0 < ang_pitch < 180:
                move_to_position = int(2048 + 11.3777*ang_pitch)
                move_dynamixel_to_position(DXL2_ID, move_to_position)  # pitch +
            else :
                move_to_position = int(2048 - 11.3777*ang_pitch)
                move_dynamixel_to_position(DXL2_ID, move_to_position)  # pitch -

            # roll   ccw : +, cw : -
            if 0 < ang_roll < 180:
                move_to_position = int(2048 + 11.3777*ang_roll)
                move_dynamixel_to_position(DXL3_ID, move_to_position)  # roll +
            else :
                move_to_position = int(2048 - 11.3777*ang_roll)
                move_dynamixel_to_position(DXL3_ID, move_to_position)  # roll -


    def camera_tracking_mode_callback(self, msg):
        if msg.data == 'manual':
            self.manual_mode = True
            self.get_logger().info("Switched to manual tracking mode")
            # Now use listener_callback for manual mode
            self.subscription.callback = self.listener_callback

        elif msg.data == 'automatic':
            self.manual_mode = False
            self.get_logger().info("Switched to automatic tracking mode")
            # Now use manual_imu_callback for automatic mode
            self.subscription.callback = self.manual_imu_callback

    def robot_tracking_mode_callback(self, msg):
        if msg.data == 'robot tracking start':
            self.get_logger().info("Starting yaw position publishing")
            self.timer = self.create_timer(0.1, self.publish_yaw_position)
        elif msg.data == 'robot tracking stop':
            self.get_logger().info("Stopping yaw position publishing")
            if hasattr(self, 'timer'):
                self.timer.cancel()

    def publish_yaw_position(self):
        yaw_position = read_dynamixel_position(DXL1_ID)
        self.yaw_position_pub.publish(Int32(data=yaw_position))
        self.get_logger().info(f"Published yaw position: {yaw_position}")

def main(args=None):
    rclpy.init(args=args)
    dynamixel_node = DynamixelNode()
    rclpy.spin(dynamixel_node)
    dynamixel_node.destroy_node()
    rclpy.shutdown()

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def dynamixel_thread():
    # Initialize Dynamixel motors
    # initialize_dynamixels()

    # Run ROS 2 node
    main()

    # Clear syncread parameter storage
    groupSyncRead.clearParam()

    # Disable all Dynamixel torques
    for dxl_id in [DXL1_ID, DXL2_ID, DXL3_ID]:
        disable_dynamixel_torque(dxl_id)

    # Close port
    portHandler.closePort()

initialize_dynamixels()
dynamixel_thread()
