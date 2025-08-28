#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# --- IMPORT YOUR NEW CUSTOM MESSAGES ---
from radar_msgs.msg import RadarTrackList

import can
import cantools
import time
import os
import csv
from datetime import datetime

# --- Configuration and TrackManager Class remain the same ---
CAN_INTERFACE = 'can2'
DBC_FILE = 'starkenn_radar.dbc'
FILTER_CONFIG = {
    'ROI_MIN_X': 0.8, 'ROI_MAX_X': 50.0, 'ROI_MIN_Y': -2.0, 'ROI_MAX_Y': 2.0,
    'CONFIRMATION_THRESHOLD': 3, 'MISS_THRESHOLD': 5,
    'MAX_ACCELERATION': 10.0, 'MAX_POS_JUMP': 5.0,
}
MASTER_OBJECT_MSG_PATTERN = 'Tracked_object_'

class TrackManager:
    """A class to manage and filter RADAR tracks."""
    # (This class is identical to the one in the previous answer, no changes needed)
    def __init__(self, config):
        self.config = config
        self.candidates = {}
        self.valid_tracks = {}
        self.last_update_time = {}

    def update(self, new_detections, csv_writer):
        current_time = time.time()
        current_time_iso = datetime.now().isoformat()
        seen_tids_this_frame = {d['tracking_id'] for d in new_detections}
        
        tids_to_delete = []
        for tid, track in self.valid_tracks.items():
            if tid not in seen_tids_this_frame:
                track['miss_count'] += 1
                if track['miss_count'] > self.config['MISS_THRESHOLD']:
                    tids_to_delete.append(tid)
        
        for tid in tids_to_delete:
            del self.valid_tracks[tid]
            if tid in self.last_update_time:
                del self.last_update_time[tid]

        for detection in new_detections:
            tid = detection['tracking_id']

            if not (self.config['ROI_MIN_X'] < detection['x'] < self.config['ROI_MAX_X'] and
                    self.config['ROI_MIN_Y'] < detection['y'] < self.config['ROI_MAX_Y']):
                continue

            time_delta = current_time - self.last_update_time.get(tid, current_time)
            self.last_update_time[tid] = current_time

            if tid in self.valid_tracks:
                prev_data = self.valid_tracks[tid]['data']
                if time_delta > 0:
                    accel_x = (detection['vx'] - prev_data['vx']) / time_delta
                    if abs(accel_x) > self.config['MAX_ACCELERATION']:
                        continue
                self.valid_tracks[tid]['data'] = detection
                self.valid_tracks[tid]['miss_count'] = 0
                csv_writer.writerow([current_time_iso, 'VALID', tid, detection['x'], detection['y'], detection['vx'], detection['vy']])
            elif tid in self.candidates:
                self.candidates[tid]['score'] += 1
                self.candidates[tid]['data'] = detection
                if self.candidates[tid]['score'] >= self.config['CONFIRMATION_THRESHOLD']:
                    self.valid_tracks[tid] = {'miss_count': 0, 'data': self.candidates[tid]['data']}
                    del self.candidates[tid]
                    csv_writer.writerow([current_time_iso, 'VALID', tid, detection['x'], detection['y'], detection['vx'], detection['vy']])
            else:
                self.candidates[tid] = {'score': 1, 'data': detection}

class RadarPublisherNode(Node):
    """A ROS 2 node to decode, filter, and publish RADAR data."""
    def __init__(self):
        super().__init__('radar_publisher_node')
        self.get_logger().info("--- Starting RADAR Publisher Node ---")

        # --- Publisher using the CUSTOM message type ---
        self.publisher_ = self.create_publisher(RadarTrackList, '/RadarObjects', 10)
        
        # --- Logging Setup ---
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_filename = f"radar_log_{timestamp}.csv"
        self.log_file = open(self.log_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['Timestamp', 'DataType', 'TrackingID', 'X_Distance', 'Y_Distance', 'Vx_Speed', 'Vy_Speed'])
        self.get_logger().info(f"Logging data to {self.log_filename}")
        
        # --- Find DBC file in the current working directory ---
        # Make sure you run this script from the directory containing the DBC file
        script_dir = os.getcwd()
        dbc_path = os.path.join(script_dir, DBC_FILE)
        if not os.path.exists(dbc_path):
            self.get_logger().error(f"DBC file not found at {dbc_path}. Exiting.")
            raise SystemExit

        self.db = cantools.database.load_file(dbc_path)
        self.bus = can.interface.Bus(channel=CAN_INTERFACE, interface='socketcan')
        self.track_manager = TrackManager(FILTER_CONFIG)
        
        self.timer_period = 0.05  # seconds (20 Hz)
        self.timer = self.create_timer(self.timer_period, self.run_loop)
        self.raw_detections_this_frame = []

    def run_loop(self):
        msg = self.bus.recv(timeout=0)
        while msg is not None:
            try:
                message_info = self.db.get_message_by_frame_id(msg.arbitration_id)
                msg_name = message_info.name
                
                if msg_name.startswith(MASTER_OBJECT_MSG_PATTERN):
                    decoded_message = self.db.decode_message(msg.arbitration_id, msg.data)
                    obj_num_str = msg_name.split('_')[-1]
                    tid = decoded_message.get(f'Tracked_obj_{obj_num_str}_Tracking_ID')

                    if tid is not None:
                        detection = {
                            'tracking_id': tid,
                            'x': decoded_message.get(f'Tracked_obj_{obj_num_str}_x_distance'),
                            'y': decoded_message.get(f'Tracked_obj_{obj_num_str}_Y_distance'),
                            'vx': decoded_message.get(f'Tracked_obj_{obj_num_str}_Vx'),
                            'vy': decoded_message.get(f'Tracked_obj_{obj_num_str}_Vy')
                        }
                        self.raw_detections_this_frame.append(detection)
                        self.csv_writer.writerow([datetime.now().isoformat(), 'RAW', tid, detection['x'], detection['y'], detection['vx'], detection['vy']])
            except (KeyError, ValueError):
                pass
            msg = self.bus.recv(timeout=0)

        self.track_manager.update(self.raw_detections_this_frame, self.csv_writer)
        self.publish_valid_tracks()
        self.raw_detections_this_frame = []

    def publish_valid_tracks(self):
        # Create an instance of our custom list message
        radar_list_msg = RadarTrackList()
        


        for tid, track in self.track_manager.valid_tracks.items():
            data = track['data']
            
            # Create an instance of our custom single object message
            obj_msg = RadarTrackList()
            
            # Populate the message fields with the filtered data
            obj_msg.tracking_id = tid
            obj_msg.x_distance = data['x']
            obj_msg.y_distance = data['y']
            obj_msg.vx= data['vx']
            obj_msg.vy = data['vy']
            
            # Add the completed object message to the list
            radar_list_msg.objects.append(obj_msg)
        
        self.publisher_.publish(radar_list_msg)

    def on_shutdown(self):
        self.get_logger().info("--- Program Stopping ---")
        self.log_file.close()
        self.bus.shutdown()
        self.get_logger().info(f"Log file '{self.log_filename}' has been saved.")

def main(args=None):
    rclpy.init(args=args)
    radar_node = RadarPublisherNode()
    try:
        rclpy.spin(radar_node)
    except KeyboardInterrupt:
        pass
    finally:
        radar_node.on_shutdown()
        radar_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()