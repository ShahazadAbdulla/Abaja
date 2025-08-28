#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vehiclecontrol.msg import Control
import struct
import can
import os

class CanSenderNode(Node):
    def __init__(self):
        super().__init__('can_sender_node')
        
        # Subscribe to the vehicle control topic
        self.subscription = self.create_subscription(
            Control,  # Your custom message type
            '/vehicle_control',
            self.vehicle_control_callback,
            10)
        
        # Initialize CAN interface
        try:
            # Using socketcan interface for CAN communication
            os.system('sudo ip link set can2 type can bitrate 500000')
            os.system('sudo ifconfig can2 up')
            self.can_bus = can.interface.Bus(channel='can2', bustype='socketcan')
            self.get_logger().info("CAN bus initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize CAN bus: {str(e)}")
            self.can_bus = None
        
        self.get_logger().info("CAN Sender Node started. Waiting for VehicleControl messages...")
    
    def vehicle_control_callback(self, msg):
        # Extract the control value from the message
        # You need to replace 'control_value' with the actual field name from your VehicleControl message
        control_value = msg.throttle  # Adjust this based on your message structure
        
        # Convert float to bytes (little-endian)
        hex_bytes = struct.pack('<f', control_value)
        
        # Create CAN message with ID 0x200
        can_id = 0x200
        can_data = list(hex_bytes) + [0x00, 0x00, 0x00, 0x00]  # Add padding to make 8 bytes
        
        # Send CAN message
        if self.can_bus is not None:
            try:
                message = can.Message(
                    arbitration_id=can_id,
                    data=can_data,
                    is_extended_id=False
                )
                self.can_bus.send(message)
                self.get_logger().info(f"Sent CAN message: ID=0x{can_id:03X}, Data={[f'0x{b:02X}' for b in can_data]}")
            except Exception as e:
                self.get_logger().error(f"Failed to send CAN message: {str(e)}")
        
        # Log the conversion
        self.get_logger().info(f"Converted {control_value:.1f} to CAN data: {[f'0x{b:02X}' for b in hex_bytes]}")
    
    def destroy_node(self):
        if self.can_bus is not None:
            self.can_bus.shutdown()
        os.system('sudo ifconfig can2 down')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    can_sender_node = CanSenderNode()
    
    try:
        rclpy.spin(can_sender_node)
    except KeyboardInterrupt:
        pass
    finally:
        can_sender_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
