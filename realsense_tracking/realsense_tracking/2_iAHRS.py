#imu_realsense_tracking_pyqt (2)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String
import serial

# 시리얼 포트 설정
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_SPEED = 115200

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('iAHRS_node')
        
        # Publisher
        self.publisher = self.create_publisher(Int32MultiArray, 'to_dynamixel', 10)
        self.azimuth_publisher = self.create_publisher(Float32MultiArray, 'azimuth', 10)
        
        # Subscriber for start signal
        self.subscription = self.create_subscription(String, 'start_signal', self.start_signal_callback, 10)
        self.reset_subscription = self.create_subscription(String, 'reset_signal', self.reset_callback, 10)

        
        self.serial = self.serial_open()
        self.max_data = 10
        self.start_signal_received = False  # '시작' 신호 수신 상태를 추적하는 변수

    def start_signal_callback(self, msg):
        """Callback for receiving start signal."""
        self.get_logger().info(f"'Start' signal received: {msg.data}")
        self.start_signal_received = True
    
    def reset_callback(self, msg):
        """리셋 신호 수신 시 처리"""
        self.get_logger().info(f"Reset signal received: {msg.data}")

        """3축 데이터 출력 플래그 초기화"""
        self.start_signal_received = False
        self.get_logger().info("Start signal reset. Publishing all axes.")

    def serial_open(self):
        """Open the serial port for communication."""
        try:
            ser = serial.Serial(SERIAL_PORT, SERIAL_SPEED, timeout=0.1)
            self.get_logger().info(f"{SERIAL_PORT} open success")
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f"Error unable to open {SERIAL_PORT}: {e}")
            return None

    def send_recv(self, command, data_length):
        """Send command and receive data from the serial port."""
        if not self.serial:
            return 0, []

        # Send command to the serial port
        self.serial.write(command.encode())

        # Receive and process data
        recv_buff = self.serial.readline().decode().strip()

        # Process received data if available
        if recv_buff:
            if recv_buff[0] == '!':
                return -1
            if command[:-1] == recv_buff[:len(command)-1] and recv_buff[len(command)-1] == '=':
                data_str = recv_buff[len(command):]
                data_list = data_str.split(',')
                data_count = min(len(data_list), data_length)
                try:
                    returned_data = [int(data, 16) if data.startswith('0x') else float(data) for data in data_list[:data_count]]
                    return data_count, returned_data
                except ValueError:
                    pass
        return 0, []

    def publish_data(self):
        """Publish IMU data based on start signal status."""
        euler_msg = Int32MultiArray()

        # Read Euler angles
        data_count, data = self.send_recv("e\n", self.max_data)
        if data_count >= 3:
            roll, pitch, yaw = data[:3]

            self.publish_azimuth(yaw)

            # Publish all data before receiving the start signal
            if not self.start_signal_received:
                self.publish_all(roll, pitch, yaw)
                self.get_logger().info(f"Published Euler angles: {yaw}")
            else:
                # After receiving the start signal, publish only roll data
                self.publish_roll(roll)
                self.get_logger().info(f"Published Euler angles: {roll}")

        else:
            self.get_logger().warn("Received incomplete Euler angles data")

    def publish_all(self, roll, pitch, yaw):
        """Publish roll, pitch, and yaw data to Dynamixel."""
        # Roll
        if 10 < roll < 180:
            self.publish_message(103, 0)  # roll ccw
        elif -180 < roll < -10:
            self.publish_message(103, 1)  # roll cw

        # Pitch
        if 10 < pitch < 180:
            self.publish_message(102, 1)  # pitch cc
        elif -180 < pitch < -10:
            self.publish_message(102, 0)  # pitch ccw

        # Yaw
        if 10 < yaw < 180:
            self.publish_message(101, 0)  # yaw ccw
        elif -180 < yaw < -10:
            self.publish_message(101, 1)  # yaw cw

    def publish_roll(self, roll):
        """Publish only roll data to Dynamixel."""
        if 10 < roll < 180:
            self.publish_message(103, 0)  # roll ccw
        elif -180 < roll < -10:
            self.publish_message(103, 1)  # roll cw

    def publish_message(self, dxl_id, direction):
        """Publish data to Dynamixel."""
        data_ = (dxl_id, direction)
        msg = Int32MultiArray(data=data_)
        self.publisher.publish(msg)

    def publish_azimuth(self, euler_z) :
        msg = Float32MultiArray(data=[euler_z])
        self.azimuth_publisher.publish(msg)


def main():
    rclpy.init()
    serial_publisher = SerialPublisher()
    timer_period = 0.001  # seconds
    serial_publisher.create_timer(timer_period, serial_publisher.publish_data)

    rclpy.spin(serial_publisher)

    serial_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
