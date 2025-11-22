#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_robot.msg import PersonInfo
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import os
import time
import numpy as np
from ament_index_python.packages import get_package_share_directory

class FaceEnrollmentNode(Node):
    def __init__(self):
        super().__init__('face_enrollment_node')
        self.bridge = CvBridge()
        
        # Subscribers
        self.person_sub = self.create_subscription(
            PersonInfo, 'person_info', self.person_callback, 10)
        self.image_sub = self.create_subscription(
            Image, 'processed_image', self.image_callback, 10)
        self.name_sub = self.create_subscription(
            String, 'voice_command', self.name_callback, 10)
        
        # Publisher
        self.speech_request_pub = self.create_publisher(String, 'request_speech', 10)
        
        # Get known_faces directory
        package_share_dir = get_package_share_directory('vision_robot')
        self.known_faces_dir = os.path.join(package_share_dir, 'known_faces')
        os.makedirs(self.known_faces_dir, exist_ok=True)
        
        # Enrollment state
        self.enrollment_active = False
        self.capture_count = 0
        self.capture_start = 0
        self.last_capture = 0
        self.current_frame = None
        self.bbox = None
        self.person_name = None
        self.display_name = "Person Enrollment"
        self.current_person_info = None
        self.face_mesh = self.create_face_mesh()
        
        # Create OpenCV window
        cv2.namedWindow(self.display_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.display_name, 800, 600)
        
        self.get_logger().info("Face Enrollment Node Started")
        self.get_logger().info(f"Saving faces to: {self.known_faces_dir}")

    def create_face_mesh(self):
        """Create face mesh for visualization"""
        face_mesh = np.zeros((468, 2), dtype=np.int32)
        for i in range(468):
            angle = 2 * np.pi * i / 468
            face_mesh[i] = [int(200 + 100 * np.cos(angle)), int(200 + 100 * np.sin(angle))]
        return face_mesh

    def person_callback(self, msg):
        """Handle person recognition updates"""
        self.current_person_info = msg
        
        # Start enrollment for unknown persons not already in progress
        if msg.name == "Unknown" and not self.enrollment_active:
            self.start_enrollment(msg)

    def start_enrollment(self, msg):
        """Start enrollment process"""
        self.enrollment_active = True
        self.capture_count = 0
        self.capture_start = time.time()
        self.bbox = (
            int(msg.bbox_x - msg.bbox_width//2),
            int(msg.bbox_y - msg.bbox_height//2),
            int(msg.bbox_x + msg.bbox_width//2),
            int(msg.bbox_y + msg.bbox_height//2)
        )
        self.person_name = None
        
        # Request name through speech
        request_msg = String()
        request_msg.data = "Please say your name using 'My name is...'"
        self.speech_request_pub.publish(request_msg)
        self.get_logger().info("Starting enrollment for new person")

    def name_callback(self, msg):
        """Capture name from speech"""
        if self.enrollment_active:
            text = msg.data.lower()
            if "my name is" in text:
                self.person_name = text.split("my name is")[1].strip()
            elif "call me" in text:
                self.person_name = text.split("call me")[1].strip()
            elif "i am" in text:
                self.person_name = text.split("i am")[1].strip()
            
            if self.person_name:
                # Clean name for filename
                self.person_name = "".join(c for c in self.person_name if c.isalnum() or c in " _-")
                self.get_logger().info(f"Enrollment name set to: {self.person_name}")
                
                # Clear any previous "Unknown" files for this enrollment
                self.clear_unknown_files()

    def clear_unknown_files(self):
        """Remove any temporary Unknown files for this enrollment"""
        try:
            for file in os.listdir(self.known_faces_dir):
                if file.startswith("Unknown_") and file.endswith(".jpg"):
                    os.remove(os.path.join(self.known_faces_dir, file))
                    self.get_logger().info(f"Removed temporary file: {file}")
        except Exception as e:
            self.get_logger().error(f"Error clearing files: {str(e)}")

    def image_callback(self, msg):
        """Process image frames"""
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.update_display()
        
        # Process enrollment if active
        if self.enrollment_active and self.current_frame is not None:
            current_time = time.time()
            
            # Capture images at 5fps
            if current_time - self.last_capture > 0.2 and self.capture_count < 15:
                self.capture_face()
                self.last_capture = current_time
                
                # Complete enrollment after 15 images
                if self.capture_count >= 15:
                    self.complete_enrollment()

    def update_display(self):
        """Update the enrollment window"""
        if self.current_frame is None:
            return
            
        display_frame = self.current_frame.copy()
        
        # Add person info
        if self.current_person_info:
            info = self.current_person_info
            text = f"{info.name} (ID: {info.id})" if info.name != "Unknown" else "Unknown Person"
            
            # Draw bounding box
            x1 = int(info.bbox_x - info.bbox_width//2)
            y1 = int(info.bbox_y - info.bbox_height//2)
            x2 = int(x1 + info.bbox_width)
            y2 = int(y1 + info.bbox_height)
            
            # Ensure coordinates are valid
            h, w = display_frame.shape[:2]
            x1 = max(0, min(x1, w-1))
            y1 = max(0, min(y1, h-1))
            x2 = max(0, min(x2, w-1))
            y2 = max(0, min(y2, h-1))
            
            if x2 > x1 and y2 > y1:
                # Draw face mesh (modern phone-style)
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                scale = min(info.bbox_width, info.bbox_height) / 400
                for point in self.face_mesh:
                    px = cx + int(point[0] * scale - 100 * scale)
                    py = cy + int(point[1] * scale - 100 * scale)
                    cv2.circle(display_frame, (px, py), 1, (0, 255, 255), -1)
                
                # Draw bounding box
                cv2.rectangle(display_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Draw text background
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
                cv2.rectangle(display_frame, 
                             (x1, y1 - text_size[1] - 10), 
                             (x1 + text_size[0], y1), 
                             (0, 0, 0), -1)
                
                # Draw text
                cv2.putText(display_frame, text, (x1+5, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        
        # Add enrollment status
        if self.enrollment_active:
            status = f"Enrolling: {self.capture_count}/15"
            cv2.putText(display_frame, status, (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            if self.person_name:
                name_text = f"Name: {self.person_name}"
                cv2.putText(display_frame, name_text, (10, 70), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
            else:
                cv2.putText(display_frame, "Say: 'My name is...'", (10, 70), 
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        # Show the frame
        cv2.imshow(self.display_name, display_frame)
        cv2.waitKey(1)

    def capture_face(self):
        """Capture face image for enrollment"""
        try:
            if not self.bbox:
                return
                
            # Extract face region
            x1, y1, x2, y2 = self.bbox
            h, w = self.current_frame.shape[:2]
            x1 = max(0, min(x1, w-1))
            y1 = max(0, min(y1, h-1))
            x2 = max(0, min(x2, w-1))
            y2 = max(0, min(y2, h-1))
            
            if x2 > x1 and y2 > y1:
                face_crop = self.current_frame[y1:y2, x1:x2]
                
                # Generate filename
                if self.person_name:
                    filename = f"{self.person_name}_{self.capture_count}.jpg"
                else:
                    filename = f"Unknown_{int(time.time())}_{self.capture_count}.jpg"
                
                filepath = os.path.join(self.known_faces_dir, filename)
                
                # Save face image
                cv2.imwrite(filepath, face_crop)
                self.capture_count += 1
                self.get_logger().info(f"Captured face {self.capture_count}/15 as {filename}")
                
                # Verify file creation
                if not os.path.exists(filepath):
                    self.get_logger().error(f"Failed to save: {filepath}")
        except Exception as e:
            self.get_logger().error(f"Capture error: {str(e)}")

    def complete_enrollment(self):
        """Complete enrollment process"""
        if self.person_name:
            self.get_logger().info(f"Enrollment complete for {self.person_name}")
            # Speak greeting
            greeting = String()
            greeting.data = f"It's a pleasure to meet you {self.person_name}. How may I help you?"
            self.speech_request_pub.publish(greeting)
        else:
            self.get_logger().warning("Enrollment completed without name")
        
        # Reset enrollment state
        self.enrollment_active = False
        self.capture_count = 0
        self.capture_start = 0
        self.bbox = None
        self.person_name = None

    def destroy_node(self):
        """Clean up resources"""
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FaceEnrollmentNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
