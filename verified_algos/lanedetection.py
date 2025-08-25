#!/usr/bin/env python3
# Enhanced Lane Detection Node with Robust Center Line Calculation
# Current Date and Time (UTC): 2025-06-20 00:23:49
# Current User's Login: Ashish480
# MODIFIED: Changed image source to /dev/video3

import sys
import os
import signal
import cv2
import torch
import numpy as np
import time
import scipy.special
import torchvision
import torchvision.transforms as transforms
from PIL import Image
from enum import Enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge
import json
import threading
import queue

# Suppress PyTorch warnings during initialization
import warnings
warnings.filterwarnings("ignore", category=UserWarning)
warnings.filterwarnings("ignore", category=FutureWarning)

# Set environment variables to prevent conflicts
os.environ['TORCH_SHOW_CPP_STACKTRACES'] = '0'
os.environ['CUDA_LAUNCH_BLOCKING'] = '0'

def signal_handler(signum, frame):
    print(f"\n🛑 Received signal {signum}. Shutting down lane detection...")
    rclpy.shutdown()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)

# ===== Lane Detection Model Classes =====
class conv_bn_relu(torch.nn.Module):
    def __init__(self, in_channels, out_channels, kernel_size, stride=1, padding=0, dilation=1, bias=False):
        super(conv_bn_relu, self).__init__()
        self.conv = torch.nn.Conv2d(in_channels, out_channels, kernel_size, 
            stride=stride, padding=padding, dilation=dilation, bias=bias)
        self.bn = torch.nn.BatchNorm2d(out_channels)
        self.relu = torch.nn.ReLU()
    def forward(self, x):
        x = self.conv(x)
        x = self.bn(x)
        x = self.relu(x)
        return x

class resnet(torch.nn.Module):
    def __init__(self, layers, pretrained=False):
        super(resnet, self).__init__()
        if layers == '18':
            model = torchvision.models.resnet18(pretrained=pretrained)
        elif layers == '34':
            model = torchvision.models.resnet34(pretrained=pretrained)
        elif layers == '50':
            model = torchvision.models.resnet50(pretrained=pretrained)
        else:
            raise NotImplementedError(f"ResNet-{layers} not implemented")
        
        self.conv1 = model.conv1
        self.bn1 = model.bn1
        self.relu = model.relu
        self.maxpool = model.maxpool
        self.layer1 = model.layer1
        self.layer2 = model.layer2
        self.layer3 = model.layer3
        self.layer4 = model.layer4

    def forward(self, x):
        x = self.conv1(x)
        x = self.bn1(x)
        x = self.relu(x)
        x = self.maxpool(x)
        x = self.layer1(x)
        x2 = self.layer2(x)
        x3 = self.layer3(x2)
        x4 = self.layer4(x3)
        return x2, x3, x4

class parsingNet(torch.nn.Module):
    def __init__(self, size=(288, 800), pretrained=True, backbone='18', cls_dim=(37, 10, 4), use_aux=False):
        super(parsingNet, self).__init__()
        self.size = size
        self.w = size[0]
        self.h = size[1]
        self.cls_dim = cls_dim
        self.use_aux = use_aux
        self.total_dim = np.prod(cls_dim)
        self.model = resnet(backbone, pretrained=pretrained)
        if self.use_aux:
            pass
        self.cls = torch.nn.Sequential(
            torch.nn.Linear(1800, 2048),
            torch.nn.ReLU(),
            torch.nn.Linear(2048, self.total_dim),
        )
        if backbone in ['34', '18']:
            self.pool = torch.nn.Conv2d(512, 8, 1)
        else:
            self.pool = torch.nn.Conv2d(2048, 8, 1)

    def forward(self, x):
        x2, x3, fea = self.model(x)
        fea = self.pool(fea).view(-1, 1800)
        group_cls = self.cls(fea).view(-1, *self.cls_dim)
        return group_cls

# ===== Lane Detection Configuration =====
lane_colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)]

tusimple_row_anchor = [64, 68, 72, 76, 80, 84, 88, 92, 96, 100, 104, 108, 112,
                       116, 120, 124, 128, 132, 136, 140, 144, 148, 152, 156, 160, 164,
                       168, 172, 176, 180, 184, 188, 192, 196, 200, 204, 208, 212, 216,
                       220, 224, 228, 232, 236, 240, 244, 248, 252, 256, 260, 264, 268,
                       272, 276, 280, 284]

class ModelType(Enum):
    TUSIMPLE = 0
    CULANE = 1

class ModelConfig:
    def __init__(self, model_type):
        if model_type == ModelType.TUSIMPLE:
            self.img_w = 1280
            self.img_h = 720
            self.row_anchor = tusimple_row_anchor
            self.griding_num = 100
            self.cls_num_per_lane = 56
        else:
            raise NotImplementedError("Only TUSIMPLE model supported")

class UltrafastLaneDetector:
    def __init__(self, model_path: str, model_type=ModelType.TUSIMPLE, use_gpu=False):
        self.use_gpu = use_gpu and torch.cuda.is_available()
        self.cfg = ModelConfig(model_type)
        self.model = None
        self.img_transform = None
        
        self.model = self._initialize_model(model_path)
        self.img_transform = self._initialize_image_transform()

    def _initialize_model(self, model_path: str):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found: {model_path}")
        net = parsingNet(
            pretrained=False, 
            backbone='18', 
            cls_dim=(self.cfg.griding_num + 1, self.cfg.cls_num_per_lane, 4),
            use_aux=False
        )
        if self.use_gpu:
            device = 'cuda'
            net = net.cuda()
        else:
            device = 'cpu'
        state_dict = torch.load(model_path, map_location=device)['model']
        compatible_state_dict = {}
        for k, v in state_dict.items():
            if 'module.' in k:
                compatible_state_dict[k[7:]] = v
            else:
                compatible_state_dict[k] = v
        net.load_state_dict(compatible_state_dict, strict=False)
        net.eval()
        return net

    def _initialize_image_transform(self):
        return transforms.Compose([
            transforms.Resize((288, 800)),
            transforms.ToTensor(),
            transforms.Normalize((0.485, 0.456, 0.406), (0.229, 0.224, 0.225)),
        ])

    def detect_lanes(self, image: np.ndarray):
        if self.model is None or self.img_transform is None:
            raise RuntimeError("Model not properly initialized")
        input_tensor = self._prepare_input(image)
        with torch.no_grad():
            output = self.model(input_tensor)
        lanes_points, lanes_detected = self._process_output(output)
        visualization_img = self._draw_lanes(image, lanes_points, lanes_detected)
        self.lanes_points = lanes_points
        self.lanes_detected = lanes_detected
        return visualization_img, True

    def _prepare_input(self, img: np.ndarray):
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_pil = Image.fromarray(img_rgb)
        input_img = self.img_transform(img_pil)
        input_tensor = input_img.unsqueeze(0)
        if self.use_gpu:
            input_tensor = input_tensor.cuda()
        return input_tensor

    def _process_output(self, output: torch.Tensor):
        processed_output = output[0].data.cpu().numpy()
        processed_output = processed_output[:, ::-1, :]
        prob = scipy.special.softmax(processed_output[:-1, :, :], axis=0)
        idx = np.arange(self.cfg.griding_num) + 1
        idx = idx.reshape(-1, 1, 1)
        loc = np.sum(prob * idx, axis=0)
        processed_output = np.argmax(processed_output, axis=0)
        loc[processed_output == self.cfg.griding_num] = 0
        processed_output = loc
        col_sample = np.linspace(0, 800 - 1, self.cfg.griding_num)
        col_sample_w = col_sample[1] - col_sample[0]
        lanes_points = []
        lanes_detected = []
        max_lanes = processed_output.shape[1]
        for lane_num in range(max_lanes):
            lane_points = []
            if np.sum(processed_output[:, lane_num] != 0) > 2:
                lanes_detected.append(True)
                for point_num in range(processed_output.shape[0]):
                    if processed_output[point_num, lane_num] > 0:
                        lane_point = [
                            int(processed_output[point_num, lane_num] * col_sample_w * self.cfg.img_w / 800) - 1,
                            int(self.cfg.img_h * (self.cfg.row_anchor[self.cfg.cls_num_per_lane - 1 - point_num] / 288)) - 1
                        ]
                        lane_points.append(lane_point)
            else:
                lanes_detected.append(False)
            lanes_points.append(lane_points)
        max_length = max(len(points) for points in lanes_points) if lanes_points else 0
        for points in lanes_points:
            while len(points) < max_length:
                points.append([0, 0])
        return np.array(lanes_points), np.array(lanes_detected)

    def _draw_lanes(self, input_img: np.ndarray, lanes_points: np.ndarray, lanes_detected: np.ndarray):
        visualization_img = cv2.resize(input_img, (self.cfg.img_w, self.cfg.img_h), interpolation=cv2.INTER_AREA)
        # Draw center line in bright green
        center_line_color = (0, 255, 0)  # Bright green for center
        
        for lane_num, lane_points in enumerate(lanes_points):
            if lane_num < len(lane_colors):
                color = lane_colors[lane_num]
                for lane_point in lane_points:
                    if lane_point[0] > 0 and lane_point[1] > 0:
                        cv2.circle(visualization_img, tuple(lane_point), 3, color, -1)
        return visualization_img

# ===== Lane Detection Node =====
class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')
        self.declare_parameter('use_gpu', True)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.declare_parameter('model_path', os.path.join(script_dir, 'models', 'tusimple_18.pth'))

        self.declare_parameter('display_enabled', True)
        self.declare_parameter('processing_fps', 10.0)
        self.use_gpu = self.get_parameter('use_gpu').value
        self.model_path = self.get_parameter('model_path').value
        self.display_enabled = self.get_parameter('display_enabled').value
        self.processing_fps = self.get_parameter('processing_fps').value
        self.bridge = CvBridge()
        self.lane_detector = None
        self.image_queue = queue.Queue(maxsize=5)
        self.result_queue = queue.Queue(maxsize=5)
        
        # Debug counters
        self.frame_count = 0
        self.center_success_count = 0
        self.fallback_count = 0
        
        self._initialize_model()
        self.lane_publisher = self.create_publisher(String, '/lane_coordinates', 10)

        # ===== CHANGE START: Use direct video capture instead of ROS topic =====
        self.get_logger().info("Attempting to open video source /dev/video3...")
        # Corresponds to /dev/video3
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error(f"❌ Failed to open video source at index {self.cap}. Please check camera connection and permissions.")
            rclpy.shutdown()
            return

        self.get_logger().info(f"✅ Successfully opened video source /dev/video{self.cap}")
        # Timer to capture frames from the video source
        self.capture_timer = self.create_timer(1.0 / 30.0, self.capture_frame_callback) # Capture at 30 FPS
        # ===== CHANGE END =====
        
        self.result_timer = self.create_timer(1.0 / self.processing_fps, self.publish_results)
        self.processing_thread = threading.Thread(target=self._processing_worker, daemon=True)
        self.processing_thread.start()
        if self.display_enabled:
            self._setup_display()
            
        print("🛣️" + "="*60)
        print(f"🚗 ENHANCED LANE DETECTION NODE")
        print(f"👤 User: Ashish480")
        print(f"🕐 Started: 2025-06-20 00:23:49 UTC")
        print(f"🎯 Mission: GUARANTEED center line for Stanley LKA")
        print(f"📹 Video Source: /dev/video{self.cap}")
        print("🛣️" + "="*60)
        
        self.get_logger().info("🛣️ Enhanced Lane Detection Node Started by Ashish480")
        self.get_logger().info("🎯 GUARANTEED center line output for Stanley controller")

    def _initialize_model(self):
        if not os.path.exists(self.model_path):
            self.get_logger().error(f"❌ Model file not found: {self.model_path}")
            self.get_logger().warn("🔄 Will use fallback mode with straight paths")
            self.lane_detector = None
        else:
            try:
                self.lane_detector = UltrafastLaneDetector(self.model_path, ModelType.TUSIMPLE, self.use_gpu)
                self.get_logger().info("✅ Lane detection model loaded successfully")
            except Exception as e:
                self.get_logger().error(f"❌ Failed to load model: {e}")
                self.get_logger().warn("🔄 Will use fallback mode with straight paths")
                self.lane_detector = None

    def _setup_display(self):
        try:
            cv2.namedWindow('Lane Detection', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Lane Detection', 1280, 720)
            self.get_logger().info("📺 Display window created")
        except Exception as e:
            self.get_logger().warn(f"⚠️ Could not setup display: {e}")
            self.display_enabled = False

    def pixel_to_map(self, x_px, y_px, img_w=1280, img_h=720, img_w_m=6.0, img_h_m=24.0):
        """Convert pixel coordinates to map frame coordinates"""
        x = (x_px - img_w/2) * (img_w_m / img_w)
        y = (img_h - y_px) * (img_h_m / img_h)
        return (x, y)

    # ===== CHANGE START: New callback for capturing frames from camera =====
    def capture_frame_callback(self):
        """Reads a frame from the video capture and adds it to the processing queue."""
        if not self.cap.isOpened():
            self.get_logger().warn("Video source is not open. Skipping frame capture.")
            return
        
        ret, cv_image = self.cap.read()
        if not ret:
            self.get_logger().warn("Failed to capture a frame from the video source.")
            return

        try:
            # Add the new frame to the queue for processing
            self.image_queue.put_nowait((cv_image, time.time()))
            self.frame_count += 1
        except queue.Full:
            # Queue is full, so we drop the oldest frame and add the new one
            try:
                self.image_queue.get_nowait()
                self.image_queue.put_nowait((cv_image, time.time()))
            except queue.Empty:
                pass # Should not happen, but good to handle
        except Exception as e:
            self.get_logger().debug(f"Frame capture callback error: {e}")
    # ===== CHANGE END =====
    
    # ===== CHANGE: REMOVED the old image_callback that used ROS messages =====
    # def image_callback(self, msg: RosImage):
    #     ... (This function has been deleted)

    def _processing_worker(self):
        """Background thread for lane detection processing"""
        while rclpy.ok():
            try:
                cv_image, timestamp = self.image_queue.get(timeout=1.0)
                
                # Attempt lane detection
                if self.lane_detector is not None:
                    try:
                        result_img, success = self.lane_detector.detect_lanes(cv_image)
                        if success:
                            lane_data = self._extract_lane_coordinates()
                        else:
                            lane_data = self._generate_fallback_data("detection_failed")
                            result_img = cv_image.copy()
                    except Exception as e:
                        self.get_logger().debug(f"Detection error: {e}")
                        lane_data = self._generate_fallback_data("detection_exception")
                        result_img = cv_image.copy()
                else:
                    # No model available, use fallback
                    result_img = cv_image.copy()
                    lane_data = self._generate_fallback_data("no_model")
                
                # Queue the result
                try:
                    self.result_queue.put_nowait((result_img, lane_data, timestamp))
                except queue.Full:
                    # Remove old result and add new one
                    try:
                        self.result_queue.get_nowait()
                        self.result_queue.put_nowait((result_img, lane_data, timestamp))
                    except queue.Empty:
                        pass
                        
            except queue.Empty:
                # No images available, generate fallback data
                lane_data = self._generate_fallback_data("no_image")
                try:
                    self.result_queue.put_nowait((None, lane_data, time.time()))
                except queue.Full:
                    pass
            except Exception as e:
                self.get_logger().debug(f"Processing worker error: {e}")
                time.sleep(0.1)

    def _extract_lane_coordinates(self) -> dict:
        """Extract lane coordinates and calculate center line with multiple strategies"""
        if self.lane_detector is None or not hasattr(self.lane_detector, 'lanes_points'):
            return self._generate_fallback_data("no_lane_data")
        
        try:
            lanes_points = self.lane_detector.lanes_points
            lanes_detected = self.lane_detector.lanes_detected
            
            # Convert all detected lanes to map coordinates
            filtered_lanes = {}
            detected_lane_indices = []
            
            if len(lanes_points) >= 1:
                for i, (lane_points, detected) in enumerate(zip(lanes_points, lanes_detected)):
                    if detected:
                        valid_points = [(int(pt[0]), int(pt[1])) for pt in lane_points if pt[0] > 0 and pt[1] > 0]
                        if len(valid_points) >= 3:  # Minimum 3 points for a valid lane
                            map_points = [self.pixel_to_map(x, y) for (x, y) in valid_points]
                            # Sort points by y-coordinate (distance from vehicle)
                            map_points.sort(key=lambda p: p[1])
                            filtered_lanes[f'lane{i+1}'] = map_points
                            detected_lane_indices.append(i)
            
            # GUARANTEED center line calculation
            center_points = []
            strategy_used = "none"
            
            # Strategy 1: Use any two detected lanes (best case)
            if len(detected_lane_indices) >= 2:
                lane_keys = [f'lane{i+1}' for i in detected_lane_indices]
                best_pair = None
                max_separation = 0
                
                # Find the best lane pair
                for i in range(len(lane_keys)):
                    for j in range(i+1, len(lane_keys)):
                        lane1_points = filtered_lanes[lane_keys[i]]
                        lane2_points = filtered_lanes[lane_keys[j]]
                        
                        if len(lane1_points) > 0 and len(lane2_points) > 0:
                            # Calculate average separation
                            separations = []
                            min_len = min(len(lane1_points), len(lane2_points))
                            for k in range(min(min_len, 5)):  # Check first 5 points max
                                x1, y1 = lane1_points[k]
                                x2, y2 = lane2_points[k]
                                sep = abs(x1 - x2)
                                separations.append(sep)
                            
                            avg_separation = np.mean(separations) if separations else 0
                            if avg_separation > max_separation and avg_separation > 1.0:  # At least 1m separation
                                max_separation = avg_separation
                                best_pair = (lane_keys[i], lane_keys[j])
                
                # Calculate center from best pair
                if best_pair:
                    lane1_points = filtered_lanes[best_pair[0]]
                    lane2_points = filtered_lanes[best_pair[1]]
                    min_len = min(len(lane1_points), len(lane2_points))
                    
                    for i in range(min_len):
                        x1, y1 = lane1_points[i]
                        x2, y2 = lane2_points[i]
                        center_x = (x1 + x2) / 2.0
                        center_y = (y1 + y2) / 2.0
                        center_points.append((center_x, center_y))
                    
                    self.center_success_count += 1
                    strategy_used = f"dual_lane_{best_pair[0]}_{best_pair[1]}"
            
            # Strategy 2: Single lane with offset (medium case)
            if not center_points and len(detected_lane_indices) >= 1:
                lane_key = f'lane{detected_lane_indices[0]+1}'
                lane_points = filtered_lanes[lane_key]
                
                if len(lane_points) > 0:
                    # Determine offset based on average x-position
                    avg_x = np.mean([pt[0] for pt in lane_points[:min(5, len(lane_points))]])
                    lane_width_offset = 1.75  # Half lane width
                    
                    if avg_x < -0.5:  # Clearly left lane
                        offset = lane_width_offset
                    elif avg_x > 0.5:  # Clearly right lane
                        offset = -lane_width_offset
                    else:  # Center-ish lane, assume it's close to center
                        offset = 0.0
                    
                    for x, y in lane_points:
                        center_x = x + offset
                        center_y = y
                        center_points.append((center_x, center_y))
                    
                    strategy_used = f"single_lane_offset_{offset:.2f}"
            
            # Strategy 3: GUARANTEED straight path (fallback - always works)
            if not center_points:
                center_points = self._generate_straight_path()
                strategy_used = "straight_fallback"
                self.fallback_count += 1
            
            # Ensure we have enough points (extend if necessary)
            if len(center_points) < 8:
                last_point = center_points[-1] if center_points else (0.0, 2.0)
                while len(center_points) < 8:
                    new_x = last_point[0]  # Keep same x (straight)
                    new_y = last_point[1] + 2.0  # Extend 2m forward
                    center_points.append((new_x, new_y))
                    last_point = (new_x, new_y)
            
            # Build return data
            filtered_lanes['center'] = center_points
            filtered_lanes['strategy'] = strategy_used
            filtered_lanes['status'] = 'success'
            
            # Periodic status logging
            if self.frame_count % 50 == 0:
                success_rate = (self.center_success_count / max(1, self.frame_count)) * 100
                fallback_rate = (self.fallback_count / max(1, self.frame_count)) * 100
                print(f"🛣️ Frame {self.frame_count}: Lanes={len(detected_lane_indices)}, "
                      f"Center={len(center_points)}pts, Strategy={strategy_used}")
                print(f"📊 Rates: Success={success_rate:.1f}%, Fallback={fallback_rate:.1f}%")
            
            return filtered_lanes
            
        except Exception as e:
            self.get_logger().error(f"❌ Lane extraction error: {e}")
            return self._generate_fallback_data("extraction_error")

    def _generate_straight_path(self) -> list:
        """Generate a straight path ahead of the vehicle"""
        center_points = []
        for i in range(12):  # 12 points, 2m intervals
            center_x = 0.0  # Straight ahead
            center_y = 2.0 + i * 2.0  # Start 2m ahead, go to 24m
            center_points.append((center_x, center_y))
        return center_points

    def _generate_fallback_data(self, reason: str) -> dict:
        """Generate guaranteed fallback data with straight path"""
        self.fallback_count += 1
        center_points = self._generate_straight_path()
        
        return {
            'lane1': [],
            'lane2': [],
            'center': center_points,
            'strategy': f'fallback_{reason}',
            'status': f'fallback_{reason}'
        }

    def publish_results(self):
        """Publish lane data - GUARANTEED to always publish valid center points"""
        try:
            result_img, lane_data, timestamp = self.result_queue.get_nowait()
            
            # TRIPLE SAFETY CHECK: Ensure center points always exist
            if not lane_data.get('center') or len(lane_data.get('center', [])) == 0:
                print("⚠️ EMERGENCY: Empty center detected, generating straight path")
                lane_data['center'] = self._generate_straight_path()
                lane_data['strategy'] = 'emergency_straight'
            
            # Ensure minimum point count
            if len(lane_data.get('center', [])) < 5:
                current_center = list(lane_data.get('center', []))
                while len(current_center) < 8:
                    if current_center:
                        last_x, last_y = current_center[-1]
                        new_x = last_x
                        new_y = last_y + 2.0
                    else:
                        new_x = 0.0
                        new_y = 2.0
                    current_center.append((new_x, new_y))
                lane_data['center'] = current_center
            
            # Publish the guaranteed valid data
            lane_msg = String()
            lane_msg.data = json.dumps(lane_data)
            self.lane_publisher.publish(lane_msg)
            
            # Periodic confirmation logging
            if self.frame_count % 100 == 0:
                center_count = len(lane_data.get('center', []))
                strategy = lane_data.get('strategy', 'unknown')
                print(f"📤 PUBLISHED: {center_count} center points using {strategy}")
            
            # Display visualization
            if self.display_enabled and result_img is not None:
                # Draw center line on the image for visualization
                center_points_px = []
                for x_map, y_map in lane_data.get('center', []):
                    # Convert back to pixel coordinates for display
                    x_px = int(x_map * (1280 / 6.0) + 1280/2)
                    y_px = int(720 - y_map * (720 / 24.0))
                    if 0 <= x_px < 1280 and 0 <= y_px < 720:
                        center_points_px.append((x_px, y_px))
                
                # Draw center line in bright green
                for i, (x, y) in enumerate(center_points_px):
                    cv2.circle(result_img, (x, y), 5, (0, 255, 0), -1)
                    if i == 0:
                        cv2.putText(result_img, "CENTER", (x+10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                cv2.imshow('Lane Detection', result_img)
                key = cv2.waitKey(1) & 0xFF
                if key == 27:  # ESC key
                    rclpy.shutdown()
                    
        except queue.Empty:
            # ULTIMATE FALLBACK: Always publish something
            fallback_data = self._generate_fallback_data("queue_empty")
            lane_msg = String()
            lane_msg.data = json.dumps(fallback_data)
            self.lane_publisher.publish(lane_msg)

    def destroy_node(self):
        print("🛑 Lane Detection Node shutting down...")
        # ===== CHANGE START: Release video capture device =====
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Video source released.")
        # ===== CHANGE END =====
        try:
            if self.display_enabled:
                cv2.destroyAllWindows()
        except:
            pass
        super().destroy_node()

def main(args=None):
    torch.set_num_threads(2)
    if torch.cuda.is_available():
        torch.backends.cudnn.benchmark = False
        torch.backends.cudnn.deterministic = True
    
    rclpy.init(args=args)
    node = None
    try:
        node = LaneDetectionNode()
        # Only spin if the node was successfully initialized (camera opened)
        if node and node.cap.isOpened():
            rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 Lane Detection shutting down by user request")
    except Exception as e:
        print(f"❌ Lane Detection error: {e}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("✅ Lane Detection shutdown complete")

if __name__ == '__main__':
    main()