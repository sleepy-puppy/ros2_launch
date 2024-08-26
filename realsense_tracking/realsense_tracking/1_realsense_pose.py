import sys
import cv2
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QPushButton, QWidget, QHBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
import pyrealsense2 as rs
from ultralytics import YOLO
from deep_sort.deep_sort.tracker import Tracker as DeepSortTracker
from deep_sort.tools import generate_detections as gdet
from deep_sort.deep_sort import nn_matching
from deep_sort.deep_sort.detection import Detection
import threading
import timeit
import rclpy
from std_msgs.msg import Int32MultiArray, Float32MultiArray, String
import subprocess

class RealSenseObjectTracker(QMainWindow):
    def __init__(self):
        super().__init__()

        # UI 및 변수 초기화
        self.init_ui()
        self.init_vars()
        
        # Realsense 및 YOLO 모델 초기화
        self.pipeline, self.align = self.initialize_realsense()
        self.pose_model = YOLO('yolov8n-pose.pt')  # 포즈 모델
        self.detection_model = YOLO('knife.pt')  # Knife 감지 모델

        # Deep SORT Tracker 초기화
        self.tracker, self.encoder = self.initialize_tracker()

        # ROS2 초기화
        rclpy.init()
        self.init_ros2_publishers()

        # 타이머 및 쓰레드 초기화
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.main_loop)
        self.timer.start(1)
        
        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.capture_thread.start()

    def init_ui(self):
        self.setGeometry(100, 100, 1280, 720)
        self.setWindowTitle("Police")
        self.main_layout = QVBoxLayout()
        self.label = QLabel(self)
        self.main_layout.addWidget(self.label)

        # 트래킹 모드 버튼
        self.manual_tracking_button = self.create_button("카메라 수동 트래킹", self.switch_to_manual_tracking)
        self.automatic_tracking_button = self.create_button("카메라 자동 트래킹", self.switch_to_automatic_tracking)
        self.audio_comm_button = self.create_button("현장 통신", self.send_audio_comm)
        self.stop_audio_comm_button = self.create_button("통신 중지", self.send_stop_audio_comm)
        self.start_tracking_button = self.create_button("로봇 트래킹 시작", self.start_tracking)
        self.stop_tracking_button = self.create_button("로봇 트래킹 중지", self.stop_tracking)

        self.autonomous_driving_button = self.create_button("로봇 자동 주행", self.start_autonomous_driving)

        self.button_layout = QHBoxLayout()
        self.main_layout.addLayout(self.button_layout)
        central_widget = QWidget(self)
        central_widget.setLayout(self.main_layout)
        self.setCentralWidget(central_widget)

    def create_button(self, text, func):
        button = QPushButton(text, self)
        button.clicked.connect(func)
        self.main_layout.addWidget(button)
        return button

    def init_vars(self):
        self.tracking = False
        self.target_id = None
        self.global_center_x = 0
        self.global_center_y = 0
        self.last_detection_time = {}
        self.buttons = {}
        self.label_history = {}
        self.DISAPPEAR_THRESHOLD = 3
        self.frame = None
        self.stop_thread = False

    def initialize_realsense(self):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        pipeline.start(config)
        align = rs.align(rs.stream.color)
        return pipeline, align

    def initialize_tracker(self):
        MAX_COSINE_DISTANCE = 0.3
        NN_BUDGET = None
        ENCODER_MODEL_FILENAME = './mars-small128.pb'
        metric = nn_matching.NearestNeighborDistanceMetric("cosine", MAX_COSINE_DISTANCE, NN_BUDGET)
        tracker = DeepSortTracker(metric)
        encoder = gdet.create_box_encoder(ENCODER_MODEL_FILENAME, batch_size=1)
        return tracker, encoder

    def init_ros2_publishers(self):
        self.node = rclpy.create_node('realsense_tracker_node')
        self.publisher = self.node.create_publisher(Int32MultiArray, 'to_dynamixel', 10)
        self.start_signal_publisher = self.node.create_publisher(String, 'start_signal', 10)
        self.tracking_mode_publisher = self.node.create_publisher(String, 'camera_tracking_mode', 10)
        self.reset_signal_publisher = self.node.create_publisher(String, 'reset_signal', 10)
        self.audio_comm_publisher = self.node.create_publisher(String, 'audio_comm', 10)
        self.emergency_publisher = self.node.create_publisher(String, 'emergency', 10) 
        self.target_person_depth_publisher = self.node.create_publisher(Float32MultiArray, 'target_person_depth', 10)
        self.robot_tracking_mode_publisher = self.node.create_publisher(String, 'robot_tracking_mode', 10)

    def capture_frames(self):
        while not self.stop_thread:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if color_frame and depth_frame:
                self.frame = np.asanyarray(color_frame.get_data())
                self.depth_frame = depth_frame

    def main_loop(self):
        if self.frame is None:
            return

        start_t = timeit.default_timer()
        current_frame = self.frame.copy()

        # 포즈 모델을 사용하여 사람과 손 인식
        pose_results = self.pose_model(current_frame)
        hands, persons = self.get_hands_and_persons_from_pose(pose_results)

        if self.tracking and self.target_id is not None:
            self.update_tracking(current_frame, persons)

        knife_detections = self.process_detections(current_frame)
        detections = self.extract_features(current_frame, knife_detections)
        self.tracker.predict()
        self.tracker.update(detections)

        knife_centers = [np.array([(box.xyxy[0][0] + box.xyxy[0][2]) / 2, (box.xyxy[0][1] + box.xyxy[0][3]) / 2]) for box in knife_detections]
        dangerous_person_ids, knife_to_person_mapping = self.detect_dangerous_person(hands, knife_centers)

        self.update_buttons_and_labels(persons, dangerous_person_ids, knife_to_person_mapping)
        self.update_ui_frame(current_frame, persons, knife_detections, dangerous_person_ids, knife_to_person_mapping)

        terminate_t = timeit.default_timer()
        FPS = int(1. / (terminate_t - start_t))

    def process_detections(self, frame):
        results = self.detection_model(frame)
        return [box for box in results[0].boxes if box.conf.item() >= 0.6]

    def extract_features(self, frame, detections):
        if not detections:
            return []
        bboxes = np.array([box.xyxy[0].cpu().numpy() for box in detections])
        scores = np.array([box.conf.item() for box in detections])
        bboxes[:, 2:] -= bboxes[:, :2]  # [x, y, w, h] 형태로 변환
        features = self.encoder(frame, bboxes)
        return [Detection(bbox, score, feature) for bbox, score, feature in zip(bboxes, scores, features)]

    def get_hands_and_persons_from_pose(self, pose_results):
        hands, persons = [], []
        for result in pose_results:
            if result.keypoints is None:
                continue
            keypoints = result.keypoints.xy.tolist()
            for person_id, kp in enumerate(keypoints):
                valid_kp = [point for point in kp if point[0] > 0 and point[1] > 0]
                if not valid_kp:
                    continue
                x_min, y_min = np.min(valid_kp, axis=0)
                x_max, y_max = np.max(valid_kp, axis=0)
                persons.append({'id': person_id, 'bbox': [x_min, y_min, x_max, y_max]})
                left_hand, right_hand = kp[9], kp[10]
                if left_hand[0] > 0 and left_hand[1] > 0:
                    hands.append({'id': person_id, 'hand': 'left', 'coords': left_hand})
                if right_hand[0] > 0 and right_hand[1] > 0:
                    hands.append({'id': person_id, 'hand': 'right', 'coords': right_hand})
        return hands, persons

    def detect_dangerous_person(self, hands, knife_centers):
        dangerous_person_ids, knife_to_person_mapping = [], {}
        for knife_center in knife_centers:
            min_distance, closest_person_id = float('inf'), None
            for hand in hands:
                hand_distance = np.linalg.norm(knife_center - hand['coords'])
                if hand_distance < min_distance:
                    min_distance = hand_distance
                    closest_person_id = hand['id']
            if closest_person_id is not None:
                dangerous_person_ids.append(closest_person_id)
                knife_to_person_mapping[tuple(knife_center)] = closest_person_id
        if dangerous_person_ids:
            self.publish_emergency_signal(dangerous_person_ids)
        return dangerous_person_ids, knife_to_person_mapping

    def update_tracking(self, current_frame, persons):
        for person in persons:
            if person['id'] == self.target_id:
                self.update_tracking_center(person['bbox'])
                depth_value = self.depth_frame.get_distance(self.global_center_x, self.global_center_y)
                
                # center_x = current_frame.shape[1] // 2
                # center_y = current_frame.shape[0] // 2
                # center_depth_value = self.depth_frame.get_distance(center_x, center_y)

                
                depth_msg = Float32MultiArray()
                depth_msg.data = [float(self.target_id), depth_value] #float(self.global_center_x), center_depth_value]
                self.target_person_depth_publisher.publish(depth_msg)
                print(f"Target ID {self.target_id} depth: {depth_value:.3f} meters")

    def update_tracking_center(self, bbox):
        center_x = int((bbox[0] + bbox[2]) / 2)
        center_y = int((bbox[1] + bbox[3]) / 2)
        self.global_center_x = center_x
        self.global_center_y = center_y
        print(self.global_center_x, self.global_center_y)
        dx1 = 280 - self.global_center_x if self.global_center_x < 280 else 0
        dx2 = self.global_center_x - 360 if self.global_center_x > 360 else 0
        dy1 = 200 - self.global_center_y if self.global_center_y < 200 else 0
        dy2 = self.global_center_y - 280 if self.global_center_y > 280 else 0
        if (dx1 < dy1 and dx2 <= dy1) or (dx1 < dy2 and dx2 <= dy2):
            self.publish_data(102, 0 if self.global_center_y > 280 else 1)  # y ccw/cw
        elif (dy1 < dx1 and dy2 <= dx1) or (dy1 < dx2 and dy2 <= dx2):
            self.publish_data(101, 0 if self.global_center_x > 360 else 1)  # x ccw/cw

    def update_buttons_and_labels(self, persons, dangerous_person_ids, knife_to_person_mapping):
        current_ids = set([person['id'] for person in persons])
        current_time = timeit.default_timer()

        for person_id in current_ids:
            if person_id not in self.buttons:
                self.create_button_id(person_id)
                self.button_layout.addWidget(self.buttons[person_id])
            self.label_history[person_id] = current_time

        for person_id in list(self.buttons.keys()):
            if current_time - self.label_history.get(person_id, 0) > self.DISAPPEAR_THRESHOLD:
                self.buttons[person_id].setParent(None)
                del self.buttons[person_id]
                del self.label_history[person_id]
                self.publish_reset_signal()

    def update_ui_frame(self, frame, persons, knife_detections, dangerous_people, knife_to_person_mapping):
        for person in persons:
            bbox = person['bbox']
            person_id = person['id']
            color = (255, 255, 255) if self.tracking and self.target_id == person_id else (0, 255, 255) if person_id in dangerous_people else (0, 0, 0)
            cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), color, 2)
            cv2.putText(frame, f'ID: {person_id}', (int(bbox[0]), int(bbox[1]) + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        for knife_index, knife_box in enumerate(knife_detections, 1):
            bbox = knife_box.xyxy[0].cpu().numpy()
            cv2.rectangle(frame, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (0, 0, 255), 2)
            knife_center = tuple(np.array([(bbox[0] + bbox[2]) / 2, (bbox[1] + bbox[3]) / 2]))
            if knife_center in knife_to_person_mapping:
                person_id = knife_to_person_mapping[knife_center]
                cv2.putText(frame, f'Closest ID: {person_id}', (int(bbox[0]), int(bbox[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            cv2.putText(frame, f'Knife ID: {knife_index}', (int(bbox[0]), int(bbox[1]) + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        color_image = cv2.resize(frame, (640, 360))
        h, w, ch = color_image.shape
        bytes_per_line = ch * w
        q_image = QImage(color_image.data, w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(q_image)
        self.label.setPixmap(pixmap)
        self.label.setFixedSize(w, h)

    def start_autonomous_driving(self):
        subprocess.Popen(['ros2', 'launch', 'gps_navigation', 'buddybot_launch.py'])
        print("GPS_navigation pkg start")

    def publish_data(self, dxl_id, direction):
        data = Int32MultiArray(data=(dxl_id, direction))
        self.publisher.publish(data)

    def create_button_id(self, track_id):
        button = QPushButton(str(track_id), self)
        button.clicked.connect(self.on_button_click)
        self.buttons[track_id] = button

    def send_start_signal(self):
        msg = String()
        msg.data = 'Start'
        self.start_signal_publisher.publish(msg)
        print("Start signal sent")

    def publish_reset_signal(self):
        msg = String()
        msg.data = 'Reset'
        self.reset_signal_publisher.publish(msg)
        print("Reset signal sent")

    def on_button_click(self):
        sender = self.sender()
        if sender:
            self.target_id = int(sender.text())
            self.send_start_signal()
            self.tracking = True

    def start_tracking(self):
        msg = String()
        msg.data = 'robot tracking start'
        self.robot_tracking_mode_publisher.publish(msg)
        print("Tracking started")

    def stop_tracking(self):
        msg = String()
        msg.data = 'robot tracking stop'
        self.robot_tracking_mode_publisher.publish(msg)
        print("Tracking stopped")

    def switch_to_manual_tracking(self):
        msg = String()
        msg.data = 'manual'
        self.tracking_mode_publisher.publish(msg)

    def switch_to_automatic_tracking(self):
        msg = String()
        msg.data = 'automatic'
        self.tracking_mode_publisher.publish(msg)

    def send_audio_comm(self):
        msg = String()
        msg.data = 'Audio communication triggered'
        self.audio_comm_publisher.publish(msg)
        print("Audio communication message sent")

    def send_stop_audio_comm(self):
        msg = String()
        msg.data = 'Audio communication stopped'
        self.audio_comm_publisher.publish(msg)
        print("Audio communication stop message sent")

    def publish_emergency_signal(self, dangerous_person_ids):
        msg = String()
        msg.data = f"Emergency: Dangerous person detected with IDs {dangerous_person_ids}"
        self.emergency_publisher.publish(msg)
        print("Emergency signal sent")

    def closeEvent(self, event):
        self.stop_thread = True
        self.pipeline.stop()
        event.accept()

def main():
    app = QApplication(sys.argv)
    main_win = RealSenseObjectTracker()
    main_win.show()
    try:
        sys.exit(app.exec_())
    except SystemExit:
        print('Closing Window...')

if __name__ == '__main__':
    main()
