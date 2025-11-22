#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_robot.msg import PersonInfo
from gtts import gTTS
import pygame
import tempfile
import time

import sys
import os

# Add package directory to path
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))

class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech_node')
        self.subscription = self.create_subscription(
            PersonInfo,
            'person_info',
            self.info_callback,
            10)
        pygame.mixer.init()
        self.last_spoken = 0
        self.get_logger().info("Speech Node Started")

    def info_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_spoken > 10:  # 10-second cooldown
            greeting = f"Namaste! {msg.name} with ID {msg.id}"
            self.get_logger().info(f"Speaking: {greeting}")
            
            # Generate speech
            tts = gTTS(text=greeting, lang='hi')  # Hindi language
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as fp:
                tts.save(fp.name)
                pygame.mixer.music.load(fp.name)
                pygame.mixer.music.play()
                while pygame.mixer.music.get_busy():
                    pygame.time.Clock().tick(10)
            
            self.last_spoken = current_time

def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
