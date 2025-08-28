#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vehiclecontrol.msg import Control
from std_msgs.msg import Float32  # Import Float32 for the feedback subscriber
import sys, select, termios, tty
import threading

# --- User Instructions ---
MSG = """
---------------------------
Steering Teleop Controller
---------------------------
Use Left/Right Arrow Keys to change the steering angle.
The terminal will display both your command and the real-time
feedback from the /steering_angle topic.

Left Arrow : Decrease angle by 5 degrees
Right Arrow: Increase angle by 5 degrees

Press 'q' or 'Ctrl+C' to quit.
---------------------------
"""

# --- Key Mappings ---
KEY_BINDINGS = {
    '\x1b[C': 'RIGHT',
    '\x1b[D': 'LEFT',
}

# --- Configuration ---
COMMAND_STEP = 0.05
MAX_COMMAND = 0.3
MIN_COMMAND = -0.3
ANGLE_FACTOR = 100.0

def get_key(settings):
    """Function to get a single key press from the terminal (Linux/macOS)."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(3)
        if len(key) == 1:
            key = key[0]
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class TeleopKeyboardNode(Node):
    """
    A node that captures keyboard input to publish steering commands and
    subscribes to steering feedback to display it in the terminal.
    """
    def __init__(self):
        super().__init__('teleop_keyboard_node')
        
        # --- Create Publisher and Subscriber ---
        self.publisher = self.create_publisher(Control, 'vehicle_control', 10)
        self.feedback_subscriber = self.create_subscription(
            Float32,
            'steering_angle',      # Topic published by the CAN interface node
            self.feedback_callback,
            10)

        # --- Initialize State Variables ---
        self.current_command_value = 0.0
        self.target_command_value = 0.0
        self.latest_feedback_angle = 0.0
        
        self.terminal_settings = termios.tcgetattr(sys.stdin)
        self.stop_event = threading.Event()

    def feedback_callback(self, msg):
        """This function is called every time a message is received on /steering_angle."""
        self.latest_feedback_angle = msg.data
        self.update_display() # Update the display whenever new feedback arrives

    def run_loop(self):
        """The main loop that listens for keys and publishes commands."""
        print(MSG)
        
        try:
            # Initial publication to set a neutral state
            self.publish_command() 
            
            while not self.stop_event.is_set():
                key = get_key(self.terminal_settings)

                if key in KEY_BINDINGS.keys():
                    action = KEY_BINDINGS[key]
                    if action == 'RIGHT':
                        self.target_command_value += COMMAND_STEP
                    elif action == 'LEFT':
                        self.target_command_value -= COMMAND_STEP
                    
                    self.target_command_value = max(MIN_COMMAND, min(MAX_COMMAND, self.target_command_value))

                elif key == 'q':
                    break
                
                if self.target_command_value != self.current_command_value:
                    self.current_command_value = self.target_command_value
                    self.publish_command()
                
        except Exception as e:
            self.get_logger().error(f"An error occurred in the run loop: {e}")
        
        finally:
            self.stop_event.set()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
            self.current_command_value = 0.0
            self.publish_command()

    def publish_command(self):
        """Calculates steering angle, publishes the Control message, and updates the display."""
        steering_angle = self.current_command_value * ANGLE_FACTOR

        msg = Control()
        msg.steering = float(steering_angle)
        # Set other fields to neutral values
        msg.throttle = 0.0
        msg.brake = 0.0
        msg.longswitch = 0
        msg.latswitch = 0

        self.publisher.publish(msg)
        self.update_display()

    def update_display(self):
        """Updates the terminal display with the current command and feedback."""
        command_angle = self.current_command_value * ANGLE_FACTOR
        # The '\r' at the end brings the cursor to the beginning of the line
        # The end='' prevents a newline, so the line is overwritten
        print(f"Command: {command_angle:6.2f}° ({self.current_command_value:.2f}) | Feedback: {self.latest_feedback_angle:6.2f}°", end='\r')
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    teleop_node = TeleopKeyboardNode()

    # Create a thread to spin the node for processing callbacks
    spin_thread = threading.Thread(target=rclpy.spin, args=(teleop_node,), daemon=True)
    spin_thread.start()

    try:
        teleop_node.run_loop() # Start the main execution loop for keyboard input
    except KeyboardInterrupt:
        pass # Handle Ctrl+C gracefully
    
    # Cleanup
    teleop_node.stop_event.set()
    teleop_node.destroy_node()
    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()