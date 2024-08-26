import rclpy
import pyaudio
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray, String  # String 메시지 타입 추가
import sys, os

# os.environ["ROS_DOMAIN_ID"] = "10"

class RealtimeAudio(Node):
    def __init__(self):
        super().__init__('realtime_audio_pc_node')
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 2
        self.RATE = 22050 
        self.p = pyaudio.PyAudio()

        self.publisher = self.create_publisher(UInt8MultiArray, '/voice_PC', 10)
        self.subscription = self.create_subscription(UInt8MultiArray, '/voice_Orin', self.sub_audio, 10)
        self.comm_subscription = self.create_subscription(String, '/audio_comm', self.handle_comm, 10)  # 추가된 구독
        self.timer = self.create_timer(0.01, self.pub_audio)

        # 스트림 변수 초기화
        self.input_stream = None
        self.output_stream = None
        
        print('Ready for audio communication commands.')

    def pub_audio(self):
        if self.input_stream:  # 통신이 활성화된 경우에만 실행
            data = self.input_stream.read(self.CHUNK)
            print(type(data), data)
            pub_msg = UInt8MultiArray()
            pub_msg.data = list(data)
            
            # 퍼블리시
            self.publisher.publish(pub_msg)
        
    def sub_audio(self, sub_msg):
        if self.output_stream:  # 통신이 활성화된 경우에만 실행
            sub_msg_ = bytes(sub_msg.data)
            self.output_stream.write(sub_msg_)

    def handle_comm(self, msg):
        command = msg.data
        if command == 'Audio communication triggered':
            # 스트림 시작
            if self.input_stream is None:
                self.input_stream = self.p.open(format=self.FORMAT,
                                                channels=self.CHANNELS,
                                                rate=self.RATE,
                                                input=True,
                                                frames_per_buffer=self.CHUNK)
                
            if self.output_stream is None:
                self.output_stream = self.p.open(format=self.FORMAT,
                                                 channels=self.CHANNELS,
                                                 rate=self.RATE,
                                                 output=True)
            
            print('Audio communication triggered, starting audio streams.')

        elif command == 'Audio communication stopped':
            # 스트림 종료
            if self.input_stream is not None:
                self.input_stream.stop_stream()
                self.input_stream.close()
                self.input_stream = None
            
            if self.output_stream is not None:
                self.output_stream.stop_stream()
                self.output_stream.close()
                self.output_stream = None
            
            print('Audio communication stopped received, stopping audio streams.')

def main(args=None):
    rclpy.init(args=args)
    audio_node = RealtimeAudio()
    rclpy.spin(audio_node)    

if __name__ == '__main__':
    main()
