import sys
import cv2
import numpy as np
import time
from ultralytics import YOLO
from collections import deque

# Add the UFLD directory to sys.path to recognize 'ultrafastLaneDetector' as a package
sys.path.append("../UFLD")
from ultrafastLaneDetector import UltrafastLaneDetector, ModelType


class PIDController:
    """Simple PID Controller for Lane Keeping Assist"""
    
    def __init__(self, kp=20.0, ki=0.1, kd=5.0, output_limits=(0, 270)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        
        # PID state variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None
        
        # For debugging
        self.p_term = 0.0
        self.i_term = 0.0
        self.d_term = 0.0
    
    def update(self, error, dt):
        """Update PID controller with current error and time delta"""
        if dt <= 0:
            return 135.0  # Center value if no time elapsed
        
        # Proportional term
        self.p_term = self.kp * error
        
        # Integral term
        self.integral += error * dt
        self.i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt
        self.d_term = self.kd * derivative
        
        # Calculate PID output
        pid_output = self.p_term + self.i_term + self.d_term
        
        # Convert to steering command (center around 135 degrees)
        steering_cmd = 135.0 - pid_output  # Negative error = left deviation = steer right
        
        # Clip to output limits
        steering_cmd = np.clip(steering_cmd, self.output_limits[0], self.output_limits[1])
        
        # Update for next iteration
        self.prev_error = error
        
        return steering_cmd
    
    def reset(self):
        """Reset PID controller state"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None


class LaneKeepingAssist:
    """Lane Keeping Assist system that combines lane detection with PID control"""
    
    def __init__(self):
        # PID Controller parameters (easily tunable)
        self.pid = PIDController(kp=20.0, ki=0.1, kd=5.0)
        
        # Fusion weights
        self.angle_weight = 0.7
        self.pid_weight = 0.3
        
        # Lane detection setup (copied from lane_1.py)
        self.setup_lane_detection()
        
        # Smoothing for reference line
        self.recent_slopes = deque(maxlen=10)
        self.recent_intercepts = deque(maxlen=10)
        
        # Timing
        self.prev_time = None
    
    def setup_lane_detection(self):
        """Initialize lane detection models and video capture"""
        # Configuration from lane_1.py
        lane_model_path = "../models/tusimple_18.pth"
        model_type = ModelType.TUSIMPLE
        yolo_model_path = "../models/yolov8n.pt"
        video_path = "~/Downloads/road0.mp4"
        use_gpu = True
        self.pixel_to_meter = 0.01
        
        try:
            self.lane_detector = UltrafastLaneDetector(lane_model_path, model_type, use_gpu)
            self.yolo_model = YOLO(yolo_model_path)
        except Exception as e:
            print(f"Error initializing models: {e}")
            sys.exit(1)
        
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            print(f"Error: Could not open video file at {video_path}")
            sys.exit(1)
    
    # Helper functions from lane_1.py
    def get_bottom_half_points(self, lane_points, frame_height):
        return [pt for pt in lane_points if pt[1] >= frame_height / 2]
    
    def fit_polynomial(self, lane_points, frame_height, degree=2):
        if not lane_points:
            return None
        
        lane_points_np = np.array(lane_points)
        x = lane_points_np[:, 0]
        y = lane_points_np[:, 1]
        y_flipped = frame_height - y
        
        try:
            coeffs = np.polyfit(y_flipped, x, deg=degree)
        except np.linalg.LinAlgError:
            return None
        return coeffs
    
    def evaluate_polynomial(self, coeffs, y_values_flipped):
        return np.polyval(coeffs, y_values_flipped)
    
    def draw_polyline(self, image, x_values, y_values, color, thickness=2):
        points = np.asarray([x_values, y_values]).T.astype(np.int32)
        cv2.polylines(image, [points], isClosed=False, color=color, thickness=thickness, lineType=cv2.LINE_AA)
    
    def get_lane_confidence(self):
        return np.random.uniform(0.7, 1.0)
    
    def calculate_lane_angle(self, x_center, y_values):
        n = len(y_values) // 2
        dx = x_center[-1] - x_center[-n]
        dy = y_values[-1] - y_values[-n]
        lane_vec = np.array([dx, -dy], dtype=float)
        lane_vec /= np.linalg.norm(lane_vec) + 1e-6
        ref_vec = np.array([0, -1], dtype=float)
        angle_rad = np.arctan2(np.cross(ref_vec, lane_vec), np.dot(ref_vec, lane_vec))
        return np.degrees(angle_rad)
    
    def map_steering_angle(self, lane_angle, max_input_angle=270.0):
        CENTER_CMD = 135.0
        if lane_angle < 0:
            return np.interp(abs(lane_angle), [0, max_input_angle], [CENTER_CMD, 0])
        else:
            return np.interp(lane_angle, [0, max_input_angle], [CENTER_CMD, 270])
    
    def process_frame(self, frame, frame_id):
        """Process a single frame and return steering command"""
        frame_height, frame_width, _ = frame.shape
        
        # Lane detection
        self.lane_detector.detect_lanes(frame)
        lanes_points = self.lane_detector.lanes_points
        
        valid_lanes = []
        confidences = []
        for lane in lanes_points:
            bottom_half = self.get_bottom_half_points(lane, frame_height)
            if len(bottom_half) >= 5:
                valid_lanes.append(bottom_half)
                confidences.append(self.get_lane_confidence())
        
        output_frame = frame.copy()
        
        # Default values
        raw_lane_angle = 0.0
        mapped_lane_angle = 135.0
        lateral_dev_m = 0.0
        pid_output = 135.0
        final_steering_cmd = 135.0
        
        if len(valid_lanes) >= 2:
            valid_lanes.sort(key=len, reverse=True)
            lane1_points, lane2_points = valid_lanes[0], valid_lanes[1]
            
            coeffs1 = self.fit_polynomial(lane1_points, frame_height)
            coeffs2 = self.fit_polynomial(lane2_points, frame_height)
            
            if coeffs1 is not None and coeffs2 is not None:
                y_values = np.linspace(frame_height*0.65, frame_height - 1, num=100)
                y_values_flipped = frame_height - y_values
                
                x1 = self.evaluate_polynomial(coeffs1, y_values_flipped)
                x2 = self.evaluate_polynomial(coeffs2, y_values_flipped)
                
                if np.mean(x1) > np.mean(x2):
                    x1, x2 = x2, x1
                    confidences[0], confidences[1] = confidences[1], confidences[0]
                
                x_center = (x1 + x2) / 2
                
                vehicle_center_x = frame_width // 2
                lateral_dev_px = x_center[-1] - vehicle_center_x
                lateral_dev_m = lateral_dev_px * self.pixel_to_meter
                
                # Calculate raw and mapped lane angle
                raw_lane_angle = self.calculate_lane_angle(x_center, y_values)
                mapped_lane_angle = self.map_steering_angle(raw_lane_angle)
                
                # PID Controller
                current_time = time.time()
                if self.prev_time is not None:
                    dt = current_time - self.prev_time
                    pid_output = self.pid.update(lateral_dev_m, dt)  # Error = lateral deviation
                    
                    # Fuse mapped angle with PID output
                    final_steering_cmd = (self.angle_weight * mapped_lane_angle + 
                                        self.pid_weight * pid_output)
                    final_steering_cmd = np.clip(final_steering_cmd, 0, 270)
                else:
                    final_steering_cmd = mapped_lane_angle
                
                self.prev_time = current_time
                
                # Draw lanes and center line
                self.draw_polyline(output_frame, x1, y_values, color=(255, 0, 0), thickness=2)
                self.draw_polyline(output_frame, x2, y_values, color=(0, 255, 0), thickness=2)
                self.draw_polyline(output_frame, x_center, y_values, color=(0, 255, 255), thickness=3)
                
                # Smoothed reference line
                n_bottom = len(y_values) // 5
                x_bottom, y_bottom = x_center[-n_bottom:], y_values[-n_bottom:]
                current_coeffs = np.polyfit(y_bottom, x_bottom, 1)
                self.recent_slopes.append(current_coeffs[0])
                self.recent_intercepts.append(current_coeffs[1])
                smoothed_slope = np.mean(self.recent_slopes)
                smoothed_intercept = np.mean(self.recent_intercepts)
                y1_smooth, y2_smooth = frame_height, int(frame_height * 0.7)
                x1_smooth = int(smoothed_slope * y1_smooth + smoothed_intercept)
                x2_smooth = int(smoothed_slope * y2_smooth + smoothed_intercept)
                cv2.line(output_frame, (x1_smooth, y1_smooth), (x2_smooth, y2_smooth), 
                        color=(255, 255, 255), thickness=2, lineType=cv2.LINE_AA)
        
        # Display debug information
        self.draw_debug_info(output_frame, raw_lane_angle, mapped_lane_angle, 
                           lateral_dev_m, pid_output, final_steering_cmd, frame_width)
        
        # Console output
        if frame_id % 10 == 0:  # Print every 10 frames to avoid spam
            print(f"Frame {frame_id:4d} | "
                  f"Mapped: {mapped_lane_angle:5.1f} | "
                  f"Lateral Dev: {lateral_dev_m:6.3f}m | "
                  f"PID: {pid_output:5.1f} | "
                  f"Final Cmd: {final_steering_cmd:5.1f}")
        
        return output_frame, final_steering_cmd
    
    def draw_debug_info(self, frame, raw_angle, mapped_angle, lateral_dev, pid_out, final_cmd, frame_width):
        """Draw debug information on the frame"""
        # Left side - lane info
        cv2.putText(frame, f"Raw Lane Angle: {raw_angle:.2f} deg", 
                   (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Mapped Angle: {mapped_angle:.1f}", 
                   (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(frame, f"Lateral Dev: {lateral_dev:.3f}m", 
                   (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Right side - PID info
        cv2.putText(frame, f"PID Output: {pid_out:.1f}", 
                   (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, f"P: {self.pid.p_term:.1f} I: {self.pid.i_term:.1f} D: {self.pid.d_term:.1f}", 
                   (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(frame, f"FINAL STEERING: {final_cmd:.1f}", 
                   (10, 180), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Status indicator
        status_color = (0, 255, 0)  # Green
        if abs(lateral_dev) > 0.3: status_color = (0, 255, 255)  # Yellow
        if abs(lateral_dev) > 0.6: status_color = (0, 0, 255)    # Red
        
        status_text = "LKA: ACTIVE"
        cv2.putText(frame, status_text, (frame_width - 150, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
    
    def run(self):
        """Main processing loop"""
        print("Starting Lane Keeping Assist System...")
        print("PID Parameters: Kp={}, Ki={}, Kd={}".format(self.pid.kp, self.pid.ki, self.pid.kd))
        print("Fusion weights: Angle={}, PID={}".format(self.angle_weight, self.pid_weight))
        print("-" * 80)
        
        frame_id = 0
        
        while self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                break
            
            frame_id += 1
            frame = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_AREA)
            
            start_time = time.time()
            
            # Process frame and get steering command
            output_frame, steering_cmd = self.process_frame(frame, frame_id)
            
            # Calculate and display FPS
            elapsed_time = time.time() - start_time
            fps = 1 / elapsed_time if elapsed_time > 0 else 0
            cv2.putText(output_frame, f"FPS: {fps:.1f}", 
                       (output_frame.shape[1] - 120, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.imshow('Lane Keeping Assist System', output_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        self.cap.release()
        cv2.destroyAllWindows()
        print("Lane Keeping Assist System finished.")


if __name__ == "__main__":
    lka = LaneKeepingAssist()
    lka.run()
