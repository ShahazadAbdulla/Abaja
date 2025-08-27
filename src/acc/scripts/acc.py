#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time
from collections import deque
from enum import Enum

# Import required message types
from radar_msgs.msg import RadarTrackList
from feedback.msg import Velocity
from vehiclecontrol.msg import Control
from inertial_msgs.msg import Pose

class ACCState(Enum):
    DISABLED = 0
    STANDSTILL = 1
    CRUISING = 2
    FOLLOWING = 3
    EMERGENCY_BRAKING = 4

class ACCController(Node):
    def __init__(self):
        super().__init__('acc_controller')
        self.get_logger().info("Starting ACC Controller Node...")
        
        # ACC Parameters
        self.target_speed_kmph = 30.0 
        self.target_speed_ms = self.target_speed_kmph / 3.6
        self.target_gap = 9.0  
        self.time_gap = 0.05   
        
        # State transition thresholds
        self.follow_engage_threshold = 30.0 
        self.cruise_resume_threshold = 40.0 
        self.emergency_brake_distance = 8.0 
        self.min_gap = 7.0                  

        # Control parameters - TUNED FOR MAXIMUM SMOOTHNESS
        self.max_acceleration = 1.4      # Reduced for very smooth acceleration
        self.max_deceleration = -4.0     # Emergency braking limit
        self.comfort_deceleration = -1.4 # Softer normal braking target

        # Standstill and startup parameters - TUNED FOR MAXIMUM SMOOTHNESS
        self.standstill_speed_threshold = 0.2 
        self.startup_acceleration_value = 0.40 # Very gentle startup
        self.hold_brake_deceleration = -0.5   
        self.min_relative_speed_to_start_moving = 0.3

        # PID Controller parameters for speed control (Cruising) - Already quite smooth
        self.kp_speed = 0.5 # Slightly reduced
        self.ki_speed = 0.04 # Slightly reduced
        self.kd_speed = 0.02 # Slightly reduced
        self.speed_integral_limit = 8.0 
        
        # PID Controller parameters for gap control (Following) - TUNED FOR MAXIMUM SMOOTHNESS
        self.kp_gap = 0.20    # Significantly Reduced: very gentle response to gap error
        self.ki_gap = 0.002   # Extremely low: almost no integral action to prevent overshoot
        self.kd_gap = 0.15    # Moderate D-gain: provides damping and reacts to rate of gap change
        self.kp_rel_vel = 0.35 # Reduced: less aggressive speed matching, relies on gentle corrections
        self.kp_lead_accel = 0.30 # Reduced: gentle mirroring of lead vehicle's accel/decel
        self.gap_integral_limit = 2.0 # Very low limit for minimal integral effect

        # Prediction Parameters - Key for smooth, anticipatory control
        self.prediction_horizon = 1.2  # Increased: "sees" further ahead
        self.kp_pred_gap_error = 0.40   # Moderate predictive gain: aims for gentle, early corrections

        # State variables
        self.current_speed_ms = 0.0
        self.current_speed_kmph = 0.0
        self.current_acceleration_commanded = 0.0 
        self.leading_vehicle = None
        self.leading_distance = float('inf')
        self.leading_relative_speed = 0.0
        self.leading_vehicle_accel_x = 0.0

        self.acc_enabled = False
        self.acc_state = ACCState.DISABLED
        self.previous_acc_state = ACCState.DISABLED
        
        self.speed_error_integral = 0.0
        self.speed_error_previous = 0.0
        self.gap_error_integral = 0.0
        self.gap_error_previous = 0.0
        
        self.last_time = self.get_clock().now().nanoseconds / 1e9
        
        self.speed_profile_active = False
        self.target_speed_profile = []
        self.profile_start_time = None
        
        self.radar_sub = self.create_subscription(
            RadarTrackList, 'RadarObjects', self.radar_callback, 10)
        self.velocity_sub = self.create_subscription(
            Velocity, 'VehicleSpeed', self.velocity_callback, 10)
        
        self.control_pub = self.create_publisher(Control, '/vehicle_control', 10)
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.get_logger().info("ACC Controller initialized with gains tuned for maximum smoothness. System is OFF by default.")

    def _reset_speed_pid_integrals(self):
        self.speed_error_integral = 0.0
        self.speed_error_previous = 0.0
        self.get_logger().debug("Speed PID integrals reset.")

    def _reset_gap_pid_integrals(self):
        self.gap_error_integral = 0.0
        self.gap_error_previous = 0.0
        self.get_logger().debug("Gap PID integrals reset.")

    def _reset_pid_integrals(self):
        self._reset_speed_pid_integrals()
        self._reset_gap_pid_integrals()
        self.get_logger().debug("All PID integrals reset.")

    def radar_callback(self, msg: RadarTrackList):
        closest_distance = float('inf')
        closest_vehicle_obj = None
        
        for obj in msg.objects:
            if obj.x_distance > 0.1 and abs(obj.y_distance) < 2.0: 
                if obj.x_distance < closest_distance:
                    closest_distance = obj.x_distance
                    closest_vehicle_obj = obj
        
        old_lead_dist = self.leading_distance
        old_lead_rel_speed = self.leading_relative_speed

        if closest_vehicle_obj:
            self.leading_vehicle = closest_vehicle_obj 
            self.leading_distance = closest_vehicle_obj.x_distance
            self.leading_relative_speed = closest_vehicle_obj.vx 

            try:
                self.leading_vehicle_accel_x = closest_vehicle_obj.ax 
            except AttributeError:
                self.leading_vehicle_accel_x = 0.0 
            
            if abs(old_lead_dist - self.leading_distance) > 0.5 or abs(old_lead_rel_speed - self.leading_relative_speed) > 0.2:
                 self.get_logger().debug(f"Radar Update: LeadDist={self.leading_distance:.1f}m, RelVel={self.leading_relative_speed:.1f}m/s, LeadAccelX={self.leading_vehicle_accel_x:.2f}m/s^2")
        elif self.leading_vehicle is not None: 
            self.get_logger().info("Radar Update: Lead vehicle lost.")
            self.leading_vehicle = None
            self.leading_distance = float('inf')
            self.leading_relative_speed = 0.0
            self.leading_vehicle_accel_x = 0.0
            
    def velocity_callback(self, msg: Velocity):
        old_speed_ms = self.current_speed_ms
        self.current_speed_ms = msg.vehicle_velocity
        self.current_speed_kmph = self.current_speed_ms * 3.6
        if abs(old_speed_ms - self.current_speed_ms) > 0.1 : 
            self.get_logger().debug(f"Velocity Update: {self.current_speed_kmph:.1f} km/h ({self.current_speed_ms:.2f} m/s)")
        
    def inertial_callback(self, msg: Pose):
        pass 
        
    def set_speed_profile(self, speed_profile):
        self.speed_profile_active = True
        self.target_speed_profile = speed_profile
        self.profile_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"Speed profile activated: {speed_profile}")
        
    def get_target_speed_from_profile(self):
        if not self.speed_profile_active or not self.target_speed_profile:
            return self.target_speed_ms
        
        elapsed_time = (self.get_clock().now().nanoseconds / 1e9) - self.profile_start_time
        
        for time_point, speed_kmph in self.target_speed_profile:
            if elapsed_time <= time_point:
                return speed_kmph / 3.6
        
        return self.target_speed_profile[-1][1] / 3.6
    
    def _calculate_safe_following_distance(self, ego_speed_ms):
        calculated_gap = self.target_gap + ego_speed_ms * self.time_gap
        safe_dist = max(self.min_gap, calculated_gap)
        return safe_dist

    def _update_acc_state(self):
        self.previous_acc_state = self.acc_state
        new_state = self.acc_state 

        self.get_logger().debug(
            f"UPDATE_STATE_CHECK: ACC_Enabled={self.acc_enabled}, CurrentSpeed={self.current_speed_ms:.2f} (StandstillThresh={self.standstill_speed_threshold:.2f}), "
            f"LeadVehicle={'Present' if self.leading_vehicle else 'None'}, LeadDist={self.leading_distance:.2f} (EmergencyThresh={self.emergency_brake_distance:.2f}, FollowEngageThresh={self.follow_engage_threshold:.2f}, CruiseResumeThresh={self.cruise_resume_threshold:.2f})"
        )

        if not self.acc_enabled:
             new_state = ACCState.DISABLED
        elif self.leading_vehicle and self.leading_distance < self.emergency_brake_distance:
            new_state = ACCState.EMERGENCY_BRAKING
        elif self.current_speed_ms < self.standstill_speed_threshold: 
            if self.acc_state != ACCState.EMERGENCY_BRAKING:
                 new_state = ACCState.STANDSTILL
        else: 
            if self.leading_vehicle:
                if (self.previous_acc_state == ACCState.CRUISING or self.previous_acc_state == ACCState.STANDSTILL) and \
                   self.leading_distance < self.follow_engage_threshold:
                    new_state = ACCState.FOLLOWING
                elif self.previous_acc_state == ACCState.FOLLOWING and self.leading_distance > self.cruise_resume_threshold:
                    new_state = ACCState.CRUISING
                elif self.acc_state == ACCState.FOLLOWING: 
                    if self.leading_distance > self.cruise_resume_threshold:
                         new_state = ACCState.CRUISING
                elif self.leading_distance < self.follow_engage_threshold: 
                    new_state = ACCState.FOLLOWING
                else: 
                    new_state = ACCState.CRUISING
            else: 
                new_state = ACCState.CRUISING
        
        if new_state != self.acc_state:
            self.get_logger().info(f"ACC State Change: {self.acc_state.name} -> {new_state.name}")
            if self.previous_acc_state == ACCState.FOLLOWING and new_state == ACCState.CRUISING:
                self._reset_speed_pid_integrals() 
            elif (self.previous_acc_state == ACCState.CRUISING or self.previous_acc_state == ACCState.STANDSTILL) and \
                 new_state == ACCState.FOLLOWING:
                self._reset_gap_pid_integrals() 
            elif (new_state == ACCState.STANDSTILL and self.previous_acc_state != ACCState.STANDSTILL) or \
                 (new_state != ACCState.DISABLED and self.previous_acc_state == ACCState.DISABLED) :
                 self._reset_pid_integrals() 
            self.acc_state = new_state
        elif self.acc_state == ACCState.DISABLED and self.acc_enabled: 
            self.get_logger().warn("ACC enabled but state is still DISABLED. Forcing re-evaluation of state.")
            self._reset_pid_integrals() 
            if self.current_speed_ms < self.standstill_speed_threshold:
                self.acc_state = ACCState.STANDSTILL
            elif self.leading_vehicle and self.leading_distance < self.follow_engage_threshold:
                self.acc_state = ACCState.FOLLOWING
            else:
                self.acc_state = ACCState.CRUISING
            self.get_logger().info(f"ACC state forced to: {self.acc_state.name} after re-evaluation.")

    def _calculate_standstill_action(self, dt):
        self.get_logger().info(
            f"STANDSTILL_ACTION: EgoSpeed={self.current_speed_ms:.2f}, TargetSpeedProfile={self.get_target_speed_from_profile():.2f}m/s, "
            f"LeadVeh={'Present' if self.leading_vehicle else 'None'}, LeadDist={self.leading_distance:.2f}, RelSpeed={self.leading_relative_speed:.2f}"
        )
        
        desired_acceleration = self.hold_brake_deceleration 

        if self.leading_vehicle and self.leading_distance < self.follow_engage_threshold :
            cond_rel_speed_ok = self.leading_relative_speed > self.min_relative_speed_to_start_moving
            cond_min_gap_ok = self.leading_distance > self.min_gap 

            if cond_rel_speed_ok and cond_min_gap_ok:
                self.get_logger().info("STANDSTILL_ACTION: Conditions MET to start following lead vehicle. Requesting startup acceleration.")
                desired_acceleration = self.startup_acceleration_value
            else:
                self.get_logger().info(f"STANDSTILL_ACTION: Conditions NOT MET to start following. Holding brake. RelSpeedOK={cond_rel_speed_ok} (Thresh={self.min_relative_speed_to_start_moving}), MinGapOK={cond_min_gap_ok} (MinGap={self.min_gap})")
        else: 
            if self.get_target_speed_from_profile() > self.standstill_speed_threshold:
                 if self.leading_vehicle: 
                     self.get_logger().info(f"STANDSTILL_ACTION: Lead vehicle PRESENT but FAR ({self.leading_distance:.1f}m > {self.follow_engage_threshold:.1f}m). Clear to cruise. Requesting startup acceleration.")
                 else: 
                     self.get_logger().info("STANDSTILL_ACTION: NO lead vehicle. Clear to cruise. Requesting startup acceleration.")
                 desired_acceleration = self.startup_acceleration_value
            else:
                 self.get_logger().info("STANDSTILL_ACTION: Target speed is zero or very low, no lead to follow. Holding brake.")
        
        self.get_logger().info(f"STANDSTILL_ACTION End: Returning desired_acceleration = {desired_acceleration:.2f} m/s^2")
        return desired_acceleration

    def _calculate_cruising_acceleration(self, target_speed, dt):
        speed_error = target_speed - self.current_speed_ms
        
        self.speed_error_integral += speed_error * dt
        self.speed_error_integral = max(min(self.speed_error_integral, self.speed_integral_limit), -self.speed_integral_limit) 
        
        speed_error_derivative = 0.0
        if dt > 0.001: 
            speed_error_derivative = (speed_error - self.speed_error_previous) / dt
        
        self.speed_error_previous = speed_error
        
        desired_acceleration_raw = (self.kp_speed * speed_error + 
                                    self.ki_speed * self.speed_error_integral + 
                                    self.kd_speed * speed_error_derivative)
        
        self.get_logger().debug(
            f"CRUISE_ACCEL: TargetSpd={target_speed:.2f}, CurrSpd={self.current_speed_ms:.2f}, Err={speed_error:.2f}, "
            f"Integral={self.speed_error_integral:.2f}, Deriv={speed_error_derivative:.2f}, Raw PID DesiredAccel={desired_acceleration_raw:.2f}"
        )

        desired_acceleration = desired_acceleration_raw 

        if desired_acceleration < 0: 
            desired_acceleration = max(desired_acceleration_raw, self.comfort_deceleration) 
            if desired_acceleration != desired_acceleration_raw:
                 self.get_logger().debug(f"CRUISE_ACCEL: Applying comfort decel limit. Raw={desired_acceleration_raw:.2f}, ComfortLimited={desired_acceleration:.2f}")
        
        if self.current_speed_ms < (self.standstill_speed_threshold * 1.5) and \
           desired_acceleration > 0.01 and desired_acceleration < self.startup_acceleration_value :
             adjusted_accel = max(self.startup_acceleration_value, desired_acceleration) 
             self.get_logger().info(f"CRUISING: Low speed startup boost: orig_A={desired_acceleration:.2f}, boost_A={adjusted_accel:.2f}")
             desired_acceleration = adjusted_accel
             
        return desired_acceleration

    def _predict_future_states(self, current_commanded_accel):
        t_h = self.prediction_horizon 

        ego_speed_future = self.current_speed_ms + current_commanded_accel * t_h
        ego_speed_future = max(0, ego_speed_future) 
        ego_dist_traveled_future = self.current_speed_ms * t_h + 0.5 * current_commanded_accel * (t_h**2)

        if self.leading_vehicle:
            v_lead_current_abs = self.leading_relative_speed + self.current_speed_ms 
            lead_speed_future = v_lead_current_abs + self.leading_vehicle_accel_x * t_h
            lead_speed_future = max(0, lead_speed_future)
            lead_dist_traveled_future = v_lead_current_abs * t_h + 0.5 * self.leading_vehicle_accel_x * (t_h**2)
            predicted_gap = (self.leading_distance + lead_dist_traveled_future) - ego_dist_traveled_future
        else: 
            lead_speed_future = 0.0 
            lead_dist_traveled_future = 0.0
            predicted_gap = float('inf')


        predicted_safe_gap = self._calculate_safe_following_distance(ego_speed_future)
        predicted_gap_error = predicted_gap - predicted_safe_gap 

        self.get_logger().debug(
            f"PREDICT: Horizon={t_h:.2f}s. EgoSpdFut={ego_speed_future:.2f} (CmdAccel={current_commanded_accel:.2f}). "
            f"LeadSpdFut={lead_speed_future:.2f} (LeadAx={self.leading_vehicle_accel_x:.2f}). "
            f"PredGap={predicted_gap:.2f}, PredSafeGap={predicted_safe_gap:.2f}, PredGapErr={predicted_gap_error:.2f}"
        )
        return predicted_gap_error


    def _calculate_following_acceleration(self, target_speed, dt):
        if not self.leading_vehicle:
            self.get_logger().warn("FOLLOWING_ACCEL: No lead vehicle. Reverting to cruise.")
            return self._calculate_cruising_acceleration(target_speed, dt)

        current_safe_dist = self._calculate_safe_following_distance(self.current_speed_ms)
        gap_error = self.leading_distance - current_safe_dist 
        
        self.gap_error_integral += gap_error * dt
        self.gap_error_integral = max(min(self.gap_error_integral, self.gap_integral_limit), -self.gap_integral_limit)

        gap_error_derivative = 0.0
        if dt > 0.001: 
            gap_error_derivative = (gap_error - self.gap_error_previous) / dt
        self.gap_error_previous = gap_error

        pid_gap_output = (self.kp_gap * gap_error + 
                          self.ki_gap * self.gap_error_integral + 
                          self.kd_gap * gap_error_derivative) 
        
        relative_speed_component = self.kp_rel_vel * self.leading_relative_speed 
        lead_accel_component = self.kp_lead_accel * self.leading_vehicle_accel_x
        
        base_desired_acceleration = pid_gap_output + relative_speed_component + lead_accel_component
        
        self.get_logger().debug(
            f"FOLLOW_ACCEL (Base): LeadD={self.leading_distance:.2f}, SafeD={current_safe_dist:.2f}, GapErr={gap_error:.2f} (Integral={self.gap_error_integral:.2f}), Deriv={gap_error_derivative:.2f} "
            f"RelVel={self.leading_relative_speed:.2f} (Comp={relative_speed_component:.2f}), LeadAx={self.leading_vehicle_accel_x:.2f} (Comp={lead_accel_component:.2f}), "
            f"PIDGapOut={pid_gap_output:.2f}, BaseDesiredAccel={base_desired_acceleration:.2f}"
        )

        predicted_gap_error = self._predict_future_states(base_desired_acceleration) 
        predictive_accel_adjustment = self.kp_pred_gap_error * predicted_gap_error
        
        self.get_logger().debug(f"FOLLOW_ACCEL (Predictive): PredGapErr={predicted_gap_error:.2f}, PredAccelAdjust={predictive_accel_adjustment:.2f} (Gain={self.kp_pred_gap_error})")

        desired_acceleration = base_desired_acceleration + predictive_accel_adjustment
        self.get_logger().debug(f"FOLLOW_ACCEL (Combined): BaseAccel={base_desired_acceleration:.2f} + PredAdjust={predictive_accel_adjustment:.2f} = CombinedAccel={desired_acceleration:.2f}")
        
        if self.current_speed_ms > target_speed and desired_acceleration > 0:
            speed_limit_factor = -0.5 * (self.current_speed_ms - target_speed) 
            if desired_acceleration > speed_limit_factor: 
                self.get_logger().debug(f"FOLLOW_ACCEL: Speed limit applied (ego_v > driver_set_v & accel > 0). Orig Combined Accel {desired_acceleration:.2f}, New Accel {speed_limit_factor:.2f}")
                desired_acceleration = speed_limit_factor

        if self.current_speed_ms < (self.standstill_speed_threshold * 1.5) and \
           desired_acceleration > 0.01 and desired_acceleration < self.startup_acceleration_value:
             adjusted_accel = max(self.startup_acceleration_value, desired_acceleration)
             self.get_logger().info(f"FOLLOWING: Low speed startup boost: orig_A={desired_acceleration:.2f}, boost_A={adjusted_accel:.2f}")
             desired_acceleration = adjusted_accel
             
        return desired_acceleration

    def calculate_desired_acceleration(self):
        current_time_sec = self.get_clock().now().nanoseconds / 1e9
        dt = current_time_sec - self.last_time
        if dt <= 0.0001: 
            dt = 0.02 
        self.last_time = current_time_sec
            
        self.get_logger().debug(f"CALC_DESIRED_ACCEL Begin: dt = {dt:.4f}, PrevCmdAccel={self.current_acceleration_commanded:.2f}")
            
        target_speed_for_mode = self.get_target_speed_from_profile() 
        self._update_acc_state() 
        
        desired_acceleration_raw = 0.0 
        
        if self.acc_state == ACCState.DISABLED:
            self.current_acceleration_commanded = 0.0
            return 0.0 
        elif self.acc_state == ACCState.EMERGENCY_BRAKING:
            self.get_logger().warn(f"EMERGENCY BRAKING ACTIVE! LeadD={self.leading_distance:.1f}m. Cmd_A={self.max_deceleration}")
            self.current_acceleration_commanded = self.max_deceleration
            return self.max_deceleration 
        elif self.acc_state == ACCState.STANDSTILL:
            desired_acceleration_raw = self._calculate_standstill_action(dt)
        elif self.acc_state == ACCState.CRUISING:
            desired_acceleration_raw = self._calculate_cruising_acceleration(target_speed_for_mode, dt)
        elif self.acc_state == ACCState.FOLLOWING:
            desired_acceleration_raw = self._calculate_following_acceleration(target_speed_for_mode, dt)
        
        final_accel = max(self.max_deceleration, min(self.max_acceleration, desired_acceleration_raw))
        
        self.current_acceleration_commanded = final_accel 
        self.get_logger().debug(f"CALC_DESIRED_ACCEL End: RawDesAccel={desired_acceleration_raw:.2f}, FinalClampedAccel={final_accel:.2f} (Stored as CmdAccel)")
        return final_accel
    
    def acceleration_to_control_signals(self, desired_acceleration):
        throttle, brake = 0.0, 0.0
        
        if desired_acceleration > 0.05: 
            throttle = min(1.0, desired_acceleration / self.max_acceleration)
        elif desired_acceleration < -0.05: 
            brake = min(1.0, abs(desired_acceleration) / abs(self.max_deceleration))
            
        self.get_logger().debug(f"ACCEL_TO_CONTROL: DesiredAccel={desired_acceleration:.2f} -> T={throttle:.2f}, B={brake:.2f} (Brake scaled against max_decel={self.max_deceleration})")
        return throttle, brake
    
    def control_loop(self):
        self.get_logger().debug(f"CONTROL_LOOP Start: ACC_Enabled={self.acc_enabled}, ACC_State={self.acc_state.name}")
        
        if not self.acc_enabled:
            if self.acc_state != ACCState.DISABLED: 
                self.get_logger().info("Control loop: acc_enabled is FALSE. Forcing DISABLED state and zero controls.")
                self.acc_state = ACCState.DISABLED 
                self._reset_pid_integrals() 
                control_msg = Control(throttle=0.0, brake=0.0, steering=0.0, longswitch=0, latswitch=0)
                self.control_pub.publish(control_msg)
            return 
        
        if self.acc_state == ACCState.DISABLED and self.acc_enabled:
            self.get_logger().warn("Control loop: ACC enabled but internal state is DISABLED. Re-evaluating state.")
            self._update_acc_state() 
            if self.acc_state == ACCState.DISABLED: 
                self.get_logger().error("Control loop: ACC enabled, state remains DISABLED after update. Zeroing controls.")
                control_msg = Control(throttle=0.0, brake=0.0, steering=0.0, longswitch=0, latswitch=0)
                self.control_pub.publish(control_msg)
                return

        desired_acceleration = self.calculate_desired_acceleration()
        throttle, brake = self.acceleration_to_control_signals(desired_acceleration)
        
        control_msg = Control()
        control_msg.throttle = throttle
        control_msg.brake = brake
        control_msg.steering = 0.0 
        control_msg.longswitch = 1 if self.acc_enabled and self.acc_state != ACCState.DISABLED else 0
        control_msg.latswitch = 0   
        
        self.control_pub.publish(control_msg)
        
        if not hasattr(self, '_summary_log_counter'): self._summary_log_counter = 0
        self._summary_log_counter = (self._summary_log_counter + 1) 
        if self._summary_log_counter % 25 == 0: 
            lead_info = "NoLead"
            if self.leading_vehicle:
                safe_gap_distance = self._calculate_safe_following_distance(self.current_speed_ms)
                lead_info = f"LeadD: {self.leading_distance:.1f}m (TargetGap: {safe_gap_distance:.1f}m)"

            rel_vel_info = f"RelV: {self.leading_relative_speed:.1f}m/s" if self.leading_vehicle else ""
            target_speed_val = self.get_target_speed_from_profile()
            self.get_logger().info(
                f"ACC_SUMMARY [{self.acc_state.name}] V_ego:{self.current_speed_kmph:.1f}km/h | "
                f"V_target:{target_speed_val*3.6:.1f}km/h | "
                f"Accel_desired:{desired_acceleration:.2f}m/s^2 | T:{throttle:.2f} B:{brake:.2f} | LS:{control_msg.longswitch} | "
                f"{lead_info} {rel_vel_info}"
            )
    
    def enable_acc(self):
        if not self.acc_enabled:
            self.acc_enabled = True 
            self.get_logger().info("ACC system ENABLING... Resetting PIDs.")
            self._reset_pid_integrals() 
            self._update_acc_state() 
            self.get_logger().info(f"ACC system ENABLED. Initial operational state: {self.acc_state.name}")
        else:
            self.get_logger().info("enable_acc() called, but ACC system was already enabled.")
    
    def disable_acc(self):
        if self.acc_enabled:
            self.acc_enabled = False 
            self.get_logger().info("ACC system DISABLING...")
            self.acc_state = ACCState.DISABLED 
            self.previous_acc_state = ACCState.DISABLED 
            self._reset_pid_integrals() 
            control_msg = Control(throttle=0.0, brake=0.0, steering=0.0, longswitch=0, latswitch=0)
            self.control_pub.publish(control_msg)
            self.get_logger().info("ACC system DISABLED. Internal state set to DISABLED and zero controls sent.")
        else:
            self.get_logger().info("disable_acc() called, but ACC system was already disabled.")
    
    def set_target_speed(self, speed_kmph): 
        try:
            s_kmph = float(speed_kmph)
            if s_kmph < 0:
                self.get_logger().warn(f"Target speed cannot be negative ({s_kmph} km/h). Setting to 0 km/h.")
                s_kmph = 0.0
            self.target_speed_kmph = s_kmph
            self.target_speed_ms = self.target_speed_kmph / 3.6
            log_msg = f"Global target speed set to {self.target_speed_kmph:.1f} km/h ({self.target_speed_ms:.2f} m/s)."
            if self.speed_profile_active:
                log_msg += " Profile is active and may override this if conditions match."
            self.get_logger().info(log_msg)
        except ValueError:
            self.get_logger().error(f"Invalid target speed format: {speed_kmph}. Please provide a number.")
    
    def configure_for_scenario(self, scenario_type): 
        self.get_logger().info(f"Configuring for scenario: {scenario_type}")
        self.speed_profile_active = False 
        
        if scenario_type == "stop_and_go_1":
            self.set_target_speed(30.0) 
        elif scenario_type == "stop_and_go_2":
            profile = [(3,20),(5,15),(7,30),(12,20),(14,15),(20,30)]
            self.set_speed_profile(profile)
        elif scenario_type == "stop_and_go_3":
            profile = [(5,20),(7,30),(12,20),(15,10),(17,15),(22,30)]
            self.set_speed_profile(profile)
        elif scenario_type == "cut_in":
            self.set_target_speed(50.0) 
        elif scenario_type == "cut_out":
            self.set_target_speed(50.0)
        else:
            self.get_logger().warn(f"Unknown scenario: {scenario_type}. Using current/default target speed.")

def main(args=None):
    rclpy.init(args=args)
    acc_controller_node = None 
    try:
        acc_controller_node = ACCController() 
        acc_controller_node.get_logger().info("ACC node created. Waiting for system to stabilize before enabling ACC.")
        
        time.sleep(3.0)

        acc_controller_node.get_logger().info("Setting initial target speed (30 km/h) and enabling ACC.")
        acc_controller_node.set_target_speed(30.0) 
        acc_controller_node.enable_acc() 
        
        if acc_controller_node.acc_enabled and acc_controller_node.acc_state != ACCState.DISABLED:
            acc_controller_node.get_logger().info(f"ACC successfully enabled. MasterEnable={acc_controller_node.acc_enabled}, Current State={acc_controller_node.acc_state.name}")
        else:
            acc_controller_node.get_logger().error(f"Failed to properly enable ACC or stuck in DISABLED state. MasterEnable={acc_controller_node.acc_enabled}, Current State={acc_controller_node.acc_state.name}")

        rclpy.spin(acc_controller_node)

    except KeyboardInterrupt:
        if acc_controller_node: acc_controller_node.get_logger().info("KeyboardInterrupt. Shutting down...")
    except Exception as e: 
        if acc_controller_node: acc_controller_node.get_logger().fatal(f"Unhandled exception in main loop: {e}", exc_info=True)
        else: print(f"Unhandled exception before node init or after destruction: {e}")
    finally:
        if acc_controller_node and rclpy.ok() and acc_controller_node.context.ok(): 
            acc_controller_node.get_logger().info("Finally block: Disabling ACC and cleaning up node.")
            if acc_controller_node.acc_enabled: 
                acc_controller_node.disable_acc() 
            acc_controller_node.destroy_node() 
        if rclpy.ok(): 
            rclpy.shutdown() 
        print("ACC Controller shutdown sequence complete.")

if __name__ == '__main__':
    main()