#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        
        # Subscriber for user queries
        self.query_sub = self.create_subscription(
            String, 'llm_request', self.query_callback, 10)
            
        # Publisher for LLM responses
        self.response_pub = self.create_publisher(String, 'llm_response', 10)
        
        # LLM configuration
        self.llm_url = "http://localhost:11434/api/generate"
        self.get_logger().info("LLM Node Started")

    def query_callback(self, msg):
        """Handle user queries"""
        query = msg.data
        self.get_logger().info(f"Processing query: {query}")
        
        # Get response from LLM
        response = self.query_llm(query)
        
        # Publish response
        response_msg = String()
        response_msg.data = response
        self.response_pub.publish(response_msg)

    def query_llm(self, prompt):
        """Query the local LLM"""
        try:
            payload = {
                "model": "llama3",
                "prompt": prompt,
                "stream": False
            }
            response = requests.post(
                self.llm_url,
                data=json.dumps(payload),
                headers={'Content-Type': 'application/json'},
                timeout=10
            )
            if response.status_code == 200:
                return response.json().get("response", "I didn't understand that.")
            return "I'm having trouble thinking right now."
        except Exception as e:
            self.get_logger().error(f"LLM query error: {str(e)}")
            return "My brain isn't working properly at the moment."

def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
