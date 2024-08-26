#for gps_navigation_and_direction(3)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, Vector3
import math
import os
import time

class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.subscription_gps = self.create_subscription(Float32MultiArray, 'from_zedf9p_gps', self.callback_gps, 10)
        self.subscription_azimuth = self.create_subscription(Float32MultiArray, 'azimuth', self.callback_azimuth, 10)

        self.current_lat = 0.0
        self.current_lon = 0.0
        self.current_azimuth = 0.0
        self.target_lat = 0.0
        self.target_lon = 0.0

        self.robot_v = 0.2
        self.robot_w = 0.0 

        self.waypoints = self.load_waypoints()
        self.current_waypoint_index = 0
        self.max_omega = 9.0

    def load_waypoints(self):
        waypoints = []
        waypoints_file = os.path.join(os.path.dirname(__file__), 'waypoints', 'waypoints.txt')
        try:
            with open(waypoints_file, 'r') as f:
                for line in f:
                    lat, lon = map(float, line.strip().split(','))
                    waypoints.append((lat, lon))
        except FileNotFoundError:
            self.get_logger().error('Waypoints file not found')

        # print(waypoints)
        return waypoints

    def callback_gps(self, msg):
        self.current_lat = msg.data[0]
        self.current_lon = msg.data[1]
        print("current coords :", self.current_lat, ",", self.current_lon)

        self.update_target()
        self.compute_steering_angle()

    def callback_azimuth(self, msg) :
        if len(msg.data) > 0:
            self.current_azimuth = int(msg.data[0])
        else:
            self.get_logger().error('Azimuth data is empty')
            return
        if self.current_azimuth < 0.0 :
            self.current_azimuth = 360 + self.current_azimuth
        # print(self.current_azimuth)

    def update_target(self):
        if self.current_waypoint_index < len(self.waypoints):
            self.target_lat, self.target_lon = self.waypoints[self.current_waypoint_index]
            if self.reached_waypoint():
                self.current_waypoint_index += 1
                if self.current_waypoint_index >= len(self.waypoints):
                    self.get_logger().info("목적지에 도착했습니다.")
        print("Target coords :", self.target_lat, ",", self.target_lon)

    def reached_waypoint(self):
        distance = math.sqrt((self.target_lat - self.current_lat)**2 + (self.target_lon - self.current_lon)**2)
        return distance < 0.0001  # Adjust the threshold as needed

    def compute_steering_angle(self):
        if self.current_waypoint_index >= len(self.waypoints):
            # 최종 목적지에 도착한 경우
            self.robot_v = 0.0
            self.robot_w = 0.0
        else:
            delta_lon = self.target_lon - self.current_lon
            delta_lat = self.target_lat - self.current_lat
            Ld = math.sqrt(delta_lon**2 + delta_lat**2)

            if Ld == 0:
                return  # 0으로 나누는 상황 피하기

            radian = math.atan2(delta_lat, delta_lon)
            degree = math.degrees(radian)
            if degree < 0:
                degree += 360

            print(f"degree : {degree}")

            alpha = self.current_azimuth - degree
            print(f"alpha before : {alpha}")
            if alpha < -180:
                alpha = alpha + 360
            elif alpha > 180:
                alpha = alpha - 360

            # alpha_rad = math.radians(alpha)

            if abs(alpha) > 90:
                # 제자리 회전 메시지 발행
                self.get_logger().info(f'alpha: {alpha}, 제자리 회전 필요')
                self.robot_w = 0.5 * math.copysign(1, alpha)  # 회전 속도를 0.5 rad/s로 설정 (필요에 따라 조정 가능)
                self.robot_v = 0.0  # 직진 속도 0으로 설정
            else:
                self.robot_v = 0.2

                if abs(alpha) < 0.01:
                    self.robot_w = 0.0
                else:
                    # 선형적으로 오메가 값을 계산
                    self.robot_w = (1.5 / 90.0) * alpha   #copysing(a,b)는 a에 b의 부호 붙여서 반환

        self.get_logger().info(f'alpha: {alpha}, robot_v: {self.robot_v}, robot_w: {self.robot_w}')
        self.publish_cmd_vel()

    def publish_cmd_vel(self):
        purepursuit_msg = Twist()
        purepursuit_msg.linear = Vector3(x=self.robot_v, y=0.0, z=0.0)
        purepursuit_msg.angular = Vector3(x=0.0, y=0.0, z=self.robot_w)
        self.publisher_.publish(purepursuit_msg)
        time.sleep(0.5)  # 0.5초마다 발행


def main(args=None):
    rclpy.init(args=args)
    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)
    pure_pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
