#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_robot.msg import PersonInfo
from std_msgs.msg import String
from gtts import gTTS
import pygame
import tempfile
import time
import os

class SpeechNode(Node):
    def __init__(self):
        super().__init__('speech_node')
        
        # Subscribers
        self.subscription = self.create_subscription(
            PersonInfo, 'person_info', self.info_callback, 10)
        self.command_sub = self.create_subscription(
            String, 'voice_command', self.command_callback, 10)
        self.llm_response_sub = self.create_subscription(
            String, 'llm_response', self.llm_callback, 10)
            
        # Publishers
        self.speech_request_pub = self.create_publisher(String, 'request_speech', 10)
        self.llm_request_pub = self.create_publisher(String, 'llm_request', 10)
        
        # State
        self.last_spoken = 0
        self.current_user = None
        self.enrollment_mode = False
        self.conversation_mode = False
        
        pygame.mixer.init()
        self.get_logger().info("Speech Node Started")

    def speak(self, text, lang='hi'):
        """Speak text using gTTS"""
        try:
            tts = gTTS(text=text, lang=lang)
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as fp:
                tts.save(fp.name)
                pygame.mixer.music.load(fp.name)
                pygame.mixer.music.play()
                while pygame.mixer.music.get_busy():
                    pygame.time.Clock().tick(10)
            os.unlink(fp.name)
            return True
        except Exception as e:
            self.get_logger().error(f"Speech error: {str(e)}")
            return False

    def info_callback(self, msg):
        """Handle recognized person information"""
        current_time = time.time()
        
        # Greet recognized person (10-second cooldown)
        if current_time - self.last_spoken > 10 and not self.enrollment_mode:
            if msg.name != "Unknown":
                greeting = f"Namaste! {msg.name}"
                self.get_logger().info(f"Speaking: {greeting}")
                if self.speak(greeting, 'hi'):
                    self.last_spoken = current_time
                    self.current_user = msg.name
                    
                    # Follow up with conversation starter
                    time.sleep(2)
                    question = f"How may I help you, {msg.name}?"
                    self.speak(question, 'en')

    def command_callback(self, msg):
        """Handle voice commands"""
        command = msg.data.lower()
        self.get_logger().info(f"Received command: {command}")
        
        if "enroll" in command and "face" in command:
            self.start_enrollment()
        elif "call me" in command or "i am" in command or "my name is" in command:
            self.process_name_command(command)
        elif "help" in command or "assist" in command:
            self.start_conversation()
        elif "thank" in command or "bye" in command:
            self.end_conversation()
        else:
            # Forward to LLM for response
            llm_request = String()
            llm_request.data = command
            self.llm_request_pub.publish(llm_request)

    def process_name_command(self, command):
        """Extract name from voice command"""
        name = None
        if "call me" in command:
            name = command.split("call me")[1].strip()
        elif "i am" in command:
            name = command.split("i am")[1].strip()
        elif "my name is" in command:
            name = command.split("my name is")[1].strip()
            
        if name:
            # Clean up name
            name = name.split('.')[0].split('!')[0].strip()
            if len(name) > 1:
                self.get_logger().info(f"Extracted name: {name}")
                # Forward the name to the enrollment node
                name_msg = String()
                name_msg.data = name
                self.speech_request_pub.publish(name_msg)

    def start_enrollment(self):
        """Initiate face enrollment process"""
        self.enrollment_mode = True
        self.speak("Starting face enrollment. Please look at the camera.", 'en')
        time.sleep(1)
        self.speak("बोलिये 'मेरा नाम' फिर अपना नाम बताइए।", 'hi')

    def start_conversation(self):
        """Begin conversation with user"""
        self.conversation_mode = True
        self.speak("How may I assist you today?", 'en')
        time.sleep(0.5)
        self.speak("मैं आपकी किस प्रकार सहायता कर सकता हूँ?", 'hi')

    def end_conversation(self):
        """End conversation"""
        self.conversation_mode = False
        self.speak("You're welcome! Have a great day.", 'en')
        time.sleep(0.5)
        self.speak("आपका दिन शुभ हो!", 'hi')

    def llm_callback(self, msg):
        """Handle LLM responses"""
        if self.conversation_mode:
            self.speak(msg.data, 'en')
            time.sleep(0.5)
            self.speak("क्या आपको और कुछ चाहिए?", 'hi')

def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
