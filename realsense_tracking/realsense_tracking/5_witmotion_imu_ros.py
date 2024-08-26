import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import device_model
import time
import math

class IMUPublisher(Node):
    def __init__(self):
        super().__init__('witmotion_imu_node')

        # Publisher to publish IMU data
        self.publisher = self.create_publisher(Int32MultiArray, 'manual_imu_data', 10)

        # Initialize the device model
        self.device = device_model.DeviceModel("Test Device", "/dev/ttyUSB0", 9600, 0x50, self.update_data)

        # Open the device
        self.device.openDevice()

        # Start reading data in a loop
        self.device.startLoopRead()

        self.ang_y_last = None
        self.convert_ang_x = None
        self.convert_ang_y = None
        self.convert_ang_y_last = 0
        self.convert_ang_z = None
        self.init_ang_z = None
        self.msg_ang_z = None

    def update_data(self, DeviceModel):
        # Get angle and angular velocity data
        ang_x = DeviceModel.get("AngX")
        ang_y = DeviceModel.get("AngY")
        ang_z = DeviceModel.get("AngZ")
        ang_vel_x = DeviceModel.get("AsX")
        ang_vel_y = DeviceModel.get("AsY")
        ang_vel_z = DeviceModel.get("AsZ")
        ang_hx = DeviceModel.get("HX")
        ang_hy = DeviceModel.get("HY")
        ang_hz = DeviceModel.get("HZ")

        angle = math.atan2(ang_hy, ang_hx)
        angle = math.degrees(angle)
        # print(f"ang_hx : {ang_hx}, ang_hy : {ang_hy}, ang_hz : {ang_hz}")
        # print(f"방위각 : {angle}")  # 222도가 0도, 좌 - / 우 +
        # print(ang_z)

        # # Only print and publish if data is not None
        # if ang_x is not None and ang_y is not None and ang_z is not None:
        #     self.get_logger().info(f"AngX: {ang_x}, AngY: {ang_y}, AngZ: {ang_z}")

        # if ang_vel_x is not None and ang_vel_y is not None and ang_vel_z is not None:
        #     self.get_logger().info(f"Ang_vel_X: {ang_vel_x}, Ang_vel_Y: {ang_vel_y}, Ang_vel_Z: {ang_vel_z}")


        #yaw
        if -180 < ang_z <= 0 :
            self.convert_ang_z = 360 + ang_z
        else :
            self.convert_ang_z = ang_z

        # Set the initial yaw value as the offset on the first run
        # if self.init_ang_z is None:
        #     self.init_ang_z = self.convert_ang_z 
        
        # Apply the offset to make the initial value 0
        # self.msg_ang_z = (self.convert_ang_z - self.init_ang_z) % 360   #left : +, right : -
        self.msg_ang_z = self.convert_ang_z % 360   #left : +, right : -

        #pitch  up : +, down : -
        if -180 < ang_x <= 0 :
            self.convert_ang_x = 360 + ang_x
        else :
            self.convert_ang_x = ang_x

        # Check if ang_y_last is initialized
        if self.ang_y_last is None:
            self.ang_y_last = ang_y

        #roll   cw : +, ccw : -
        if 0 < ang_y and 0 < ang_vel_y :            
            if ang_y < self.ang_y_last :            #2사분면
                self.convert_ang_y = 180 - ang_y
            else :                                  #1사분면
                self.convert_ang_y = ang_y
                
        elif 0 < ang_y and ang_vel_y < 0 :
            if self.ang_y_last < ang_y :            #2사분면
                self.convert_ang_y = 180 - ang_y
            else:                                   #1사분면
                self.convert_ang_y = ang_y

        
        elif ang_y < 0 and 0 < ang_vel_y :
            if ang_y < self.ang_y_last :            #3사분면
                self.convert_ang_y = 180 - ang_y
            else :                                  #4사분면
                self.convert_ang_y = 360 + ang_y

        elif ang_y < 0 and ang_vel_y < 0 :
            if self.ang_y_last < ang_y :            #3사분면
                self.convert_ang_y = 180 - ang_y                 
            else :                                  #4사분면
                self.convert_ang_y = 360 + ang_y

        if self.ang_y_last == ang_y :
            self.convert_ang_y = self.convert_ang_y_last

        if 100 < abs(self.convert_ang_y_last - self.convert_ang_y) < 200 :
            self.convert_ang_y = self.convert_ang_y_last

        # Update the previous ang_y value
        self.ang_y_last = ang_y
        self.convert_ang_y_last = self.convert_ang_y
        
        print("ang_yaw :", ang_z, "ang_pitch :", self.convert_ang_x, ", ang_roll :", self.convert_ang_y)

        self.publish_message(self.convert_ang_x, self.convert_ang_y, self.msg_ang_z)


    def publish_message(self, ang_x, ang_y, ang_z):
        """Publish the IMU data."""
        # Convert float data to integer representation if necessary
        data_ = [int(ang_x), int(ang_y), int(ang_z)]
        msg = Int32MultiArray(data=data_)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()

    try:
        while rclpy.ok():
            rclpy.spin_once(imu_publisher)
            time.sleep(1)  # Publish at 1Hz
    except KeyboardInterrupt:
        pass
    finally:
        imu_publisher.device.stopLoopRead()
        imu_publisher.device.closeDevice()
        imu_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
