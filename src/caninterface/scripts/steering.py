#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
# Import the custom message type from your 'vehiclecontrol' package
from vehiclecontrol.msg import Control 

import can
import struct
import threading

# --- Default Configuration ---
# These can be changed at runtime using ROS parameters
DEFAULT_CAN_INTERFACE = 'can1'
# CAN ID for SENDING steering commands
DEFAULT_COMMAND_CAN_ID = 0x201 
# CAN ID for RECEIVING steering angle feedback
DEFAULT_FEEDBACK_CAN_ID = 0x232 

class CanInterfaceNode(Node):
    """
    Acts as a bridge between ROS 2 topics and a CAN bus.
    - Subscribes to `/vehicle_control` to get steering commands.
    - Publishes steering angle feedback from the CAN bus to `/steering_angle`.
    """
    def __init__(self):
        super().__init__('can_interface_node')

        # --- Declare and Get ROS Parameters for Configurability ---
        self.declare_parameter('can_interface', DEFAULT_CAN_INTERFACE)
        self.declare_parameter('command_can_id', DEFAULT_COMMAND_CAN_ID)
        self.declare_parameter('feedback_can_id', DEFAULT_FEEDBACK_CAN_ID)

        self.can_interface = self.get_parameter('can_interface').get_parameter_value().string_value
        self.command_can_id = self.get_parameter('command_can_id').get_parameter_value().integer_value
        self.feedback_can_id = self.get_parameter('feedback_can_id').get_parameter_value().integer_value
        
        self.get_logger().info(f"Using CAN Interface: '{self.can_interface}'")
        self.get_logger().info(f"Sending commands with CAN ID: {hex(self.command_can_id)}")
        self.get_logger().info(f"Listening for feedback with CAN ID: {hex(self.feedback_can_id)}")

        self.bus = None

        try:
            # --- Initialize the CAN Bus Connection ---
            self.bus = can.interface.Bus(channel=self.can_interface, interface='socketcan')
            self.get_logger().info(f"Successfully connected to CAN bus on '{self.can_interface}'.")

            # --- Create ROS 2 Publishers and Subscribers ---
            
            # Publisher for the steering angle feedback received from the CAN bus
            self.feedback_publisher = self.create_publisher(Float32, 'steering_angle', 10)
            
            # Subscriber for the main vehicle control command
            self.command_subscriber = self.create_subscription(
                Control,
                'vehicle_control',
                self.control_callback, # Function to call when a message is received
                10)

            # --- Start a Separate Thread for Reading CAN Messages ---
            # This is critical. CAN reading is a blocking operation. Running it in a separate
            # thread ensures the node remains responsive to ROS 2 messages.
            self.receive_thread = threading.Thread(target=self.receive_can_messages, daemon=True)
            self.receive_thread.start()

        except OSError:
            self.get_logger().error(f"Could not find CAN interface '{self.can_interface}'.")
            self.get_logger().error("Please ensure the interface is up (e.g., 'sudo ip link set can0 up type can bitrate 500000')")
            rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred during initialization: {e}")
            rclpy.shutdown()

    def control_callback(self, msg):
        """
        This function is executed every time a message is received on the '/vehicle_control' topic.
        It extracts the steering angle and sends it over the CAN bus.
        """
        steering_command = msg.steering
        self.get_logger().info(f"Received steering command: {steering_command:.2f}")

        try:
            # Pack the float angle into 4 raw bytes in little-endian format ('<f').
            packed_data = struct.pack('<f', steering_command)
            
            # Create the CAN message.
            message = can.Message(
                arbitration_id=self.command_can_id,
                data=packed_data,
                is_extended_id=False
            )
            # Send the message on the bus.
            self.bus.send(message)
            self.get_logger().info(f"Sent angle {steering_command:.2f} to CAN ID {hex(self.command_can_id)}.")

        except Exception as e:
            self.get_logger().error(f"An error occurred while sending CAN message: {e}")

    def receive_can_messages(self):
        """
        This function runs in the background thread, continuously listening for CAN messages.
        When a steering feedback message is found, it's published to the '/steering_angle' topic.
        """
        self.get_logger().info("CAN receiver thread started.")
        while rclpy.ok():
            try:
                # Wait for the next message (with a 1-second timeout to allow shutdown checks)
                msg = self.bus.recv(timeout=1.0)
                if msg is not None:
                    # Check if the message ID matches the one we're looking for
                    if msg.arbitration_id == self.feedback_can_id:
                        # Ensure the message is long enough to contain a float (4 bytes)
                        if len(msg.data) >= 4:
                            # Unpack the first 4 bytes of data back into a float.
                            # The result is a tuple, so we take the first element [0].
                            feedback_angle = struct.unpack('<f', msg.data[:4])[0]
                            
                            self.get_logger().info(f"Received steering feedback: {feedback_angle:.2f}")
                            
                            # Create and publish the ROS 2 message
                            feedback_msg = Float32()
                            feedback_msg.data = feedback_angle
                            self.feedback_publisher.publish(feedback_msg)
            except Exception as e:
                self.get_logger().error(f"Error in CAN receiver thread: {e}")
                
def main(args=None):
    rclpy.init(args=args)
    can_interface_node = CanInterfaceNode()
    
    # Keep the node alive until shutdown is requested
    if rclpy.ok():
      rclpy.spin(can_interface_node)
    
    can_interface_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()