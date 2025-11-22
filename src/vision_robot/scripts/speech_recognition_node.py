#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import os

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')
        
        # Fix ALSA configuration issues
        os.environ['ALSA_CARD'] = 'default'  # Force default audio card
        
        # Publisher for recognized speech
        self.publisher = self.create_publisher(String, 'voice_command', 10)
        
        # Subscriber for enrollment triggers
        self.enrollment_sub = self.create_subscription(
            String, 'request_speech', self.enrollment_callback, 10)
        
        # Speech recognition setup with explicit device
        self.recognizer = sr.Recognizer()
        
        # Get available microphones
        mics = sr.Microphone.list_microphone_names()
        self.get_logger().info(f"Available microphones: {mics}")
        
        # Try to find a suitable microphone
        mic_index = None
        for i, name in enumerate(mics):
            if 'default' in name.lower() or 'pulse' in name.lower():
                mic_index = i
                break
        
        if mic_index is None:
            self.get_logger().warning("No suitable microphone found. Using default.")
            self.microphone = sr.Microphone()
        else:
            self.get_logger().info(f"Using microphone: {mics[mic_index]}")
            self.microphone = sr.Microphone(device_index=mic_index)
        
        # Adjust for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source, duration=1)
            
        self.get_logger().info("Speech Recognition Node Started")
        self.enrollment_active = False
        
        # Start continuous listening
        self.create_timer(0.5, self.listen)  # Faster polling

    def enrollment_callback(self, msg):
        """Handle enrollment requests"""
        self.enrollment_active = True
        self.get_logger().info("Enrollment mode activated")

    def listen(self):
        """Listen for voice commands"""
        try:
            with self.microphone as source:
                self.get_logger().debug("Listening...")
                audio = self.recognizer.listen(source, timeout=1, phrase_time_limit=5)
                
            try:
                # First try English
                text = self.recognizer.recognize_google(audio, language='en-IN')
                self.process_speech(text.lower())
            except sr.UnknownValueError:
                # Try Hindi if English fails
                try:
                    text = self.recognizer.recognize_google(audio, language='hi-IN')
                    self.process_speech(text.lower())
                except sr.UnknownValueError:
                    if self.enrollment_active:
                        self.get_logger().info("Could not understand speech")
                except sr.RequestError as e:
                    self.get_logger().error(f"Hindi recognition error: {str(e)}")
            except sr.RequestError as e:
                self.get_logger().error(f"English recognition error: {str(e)}")
                
        except sr.WaitTimeoutError:
            pass
        except Exception as e:
            self.get_logger().error(f"Listening error: {str(e)}")

    def process_speech(self, text):
        """Process recognized speech"""
        self.get_logger().info(f"Recognized: {text}")
        
        # Publish all speech during enrollment
        if self.enrollment_active:
            msg = String()
            msg.data = text
            self.publisher.publish(msg)
        # Publish specific commands
        elif any(keyword in text for keyword in ["enroll", "help", "assist", "thank", "bye"]):
            msg = String()
            msg.data = text
            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SpeechRecognitionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
