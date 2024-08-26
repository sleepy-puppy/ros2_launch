#for gps_navigation_and_direction(1)

from serial import Serial
from pyubx2 import UBXReader
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('current_gps_node')
        self.publisher_ = self.create_publisher(String, 'from_zedf9p_gps', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        with Serial("/dev/ttyGPS", 9600, timeout=3) as stream:
            ubr = UBXReader(stream)
            for raw, parsed in ubr:
                if hasattr(parsed, "lat") and hasattr(parsed, "lon"):
                    latitude = parsed.lat
                    longitude = parsed.lon
                    gps_data = f"Latitude: {latitude}, Longitude: {longitude}"
                    msg = String()
                    msg.data = gps_data
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published GPS Data: {gps_data}")
                    break  # break after publishing the first valid message

def main(args=None):
    rclpy.init(args=args)
    gps_publisher = GPSPublisher()

    try:
        rclpy.spin(gps_publisher)
    except KeyboardInterrupt:
        gps_publisher.get_logger().info('Shutting down GPS Publisher...')

    # Destroy the node explicitly
    gps_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
