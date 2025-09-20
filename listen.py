import socket
import struct
import io
import threading
import time
from time import monotonic
import RPi.GPIO as GPIO
from picamera2 import Picamera2
import math

# Network Configuration
HOST = '0.0.0.0'
WHEEL_PORT = 8000
CAMERA_PORT = 8001
PID_CONFIG_PORT = 8002
ROTATE_PORT = 8003           # New: rotation command server (relative angle in degrees)
PID_ROT_CONFIG_PORT = 8004   # New: rotation PID config server
ROTATE_STATUS_PORT = 8005    # New: rotation status query server

# Pins
RIGHT_MOTOR_ENA = 18
RIGHT_MOTOR_IN1 = 17
RIGHT_MOTOR_IN2 = 27
LEFT_MOTOR_ENB = 25
LEFT_MOTOR_IN3 = 23
LEFT_MOTOR_IN4 = 24
LEFT_ENCODER = 26
RIGHT_ENCODER = 16

# TODO: Mechanical/encoder params (set these for our robot)
BASELINE_M = 0.16          # Wheel separation (meters) - example, calibrate on your robot
WHEEL_DIAMETER_M = 0.065    # Wheel diameter (meters) - example, calibrate
TICKS_PER_REV = 20          # Encoder ticks per wheel revolution (effective edges you count)
DIST_PER_TICK = math.pi * WHEEL_DIAMETER_M / TICKS_PER_REV

# TODO: Tune Rotation PID defaults - UPDATED GAINS
Kp_rot = 6.0    # Reduced from 20 - less aggressive
Ki_rot = 0.1    # Small integral for steady-state accuracy
Kd_rot = 1.0    # Damping term

# Velocity loop gains (inner loop) - NEW
Kp_vel = 25.0   # Responsive to velocity errors
Ki_vel = 5.0    # Eliminate steady-state velocity errors
Kd_vel = 1.0    # Smooth velocity tracking

# Feedforward gain (convert rad/s to PWM) - NEW
FEEDFORWARD_GAIN = 40.0  # PWM per rad/s

# Improved tolerances - UPDATED
ROT_TOL_DEG = 1.0           # Tighter tolerance
ROT_TOL_RAD = math.radians(ROT_TOL_DEG)
MAX_ROT_PWM = 50            # Increased for better performance
ROT_SETTLE_RATE = math.radians(2.0)  # Slower settle requirement
ROT_SETTLE_TIME = 0.25      # Longer settle time for stability
ROT_MAX_TIME = 5.0

# PID Constants (default values, will be overridden by client)
use_PID = 0
KP, Ki, KD = 0, 0, 0
MAX_CORRECTION = 30  # Maximum PWM correction value

# Global variables
running = True
left_pwm, right_pwm = 0, 0
left_count, right_count = 0, 0
prev_left_state, prev_right_state = None, None
use_ramping = True
RAMP_RATE = 250  # PWM units per second (adjust this value to tune ramp speed)
MIN_RAMP_THRESHOLD = 15  # Only ramp if change is greater than this
MIN_PWM_THRESHOLD = 15
current_movement, prev_movement = 'stop', 'stop'

###### New: control mode and rotation state - ENHANCED
control_mode = 'velocity'   # 'velocity' (existing) or 'rotate' (angle control)
rot_target_rad = 0.0
rot_integral = 0.0
rot_last_error = 0.0
rot_last_theta = 0.0
rot_last_time = None
rot_in_progress = False
rot_done = False
theta_rad = 0.0             # Integrated heading (relative within a rotation command)
last_left_count = 0         # For incremental delta counts
last_right_count = 0
rot_start_time = None

# NEW: Cascaded control variables
target_angular_velocity = 0.0
angular_velocity_pid_integral = 0.0
angular_velocity_pid_last_error = 0.0

# NEW: Encoder filtering variables
encoder_filter_alpha = 0.8  # Low-pass filter coefficient
filtered_angular_velocity = 0.0
velocity_history = []
VELOCITY_HISTORY_SIZE = 5

def rot_debug(msg):
    if DEBUG_ROT:
        print(f"[ROTDBG] {msg}")
DEBUG_ROT = True

def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Motor
    GPIO.setup(RIGHT_MOTOR_ENA, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN1, GPIO.OUT)
    GPIO.setup(RIGHT_MOTOR_IN2, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_ENB, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN3, GPIO.OUT)
    GPIO.setup(LEFT_MOTOR_IN4, GPIO.OUT)
    
    # This prevents slight motor jerk when connection is established
    GPIO.output(RIGHT_MOTOR_ENA, GPIO.LOW)
    GPIO.output(LEFT_MOTOR_ENB, GPIO.LOW)
    
    # Encoder setup and interrupt (both activated and deactivated)
    GPIO.setup(LEFT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(RIGHT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.add_event_detect(LEFT_ENCODER, GPIO.BOTH, callback=left_encoder_callback)
    GPIO.add_event_detect(RIGHT_ENCODER, GPIO.BOTH, callback=right_encoder_callback)
    
    # Initialize PWM (frequency: 100Hz)
    global left_motor_pwm, right_motor_pwm
    left_motor_pwm = GPIO.PWM(LEFT_MOTOR_ENB, 100)
    right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_ENA, 100)
    left_motor_pwm.start(0)
    right_motor_pwm.start(0)

def left_encoder_callback(channel):
    global left_count, prev_left_state
    current_state = GPIO.input(LEFT_ENCODER)
    
    # Check for actual state change. Without this, false positive happens due to electrical noise
    # After testing, debouncing not needed
    if (prev_left_state is not None and current_state != prev_left_state):       
        left_count += 1
        prev_left_state = current_state
    
    elif prev_left_state is None:
        # First reading
        prev_left_state = current_state

def right_encoder_callback(channel):
    global right_count, prev_right_state, prev_right_time
    current_state = GPIO.input(RIGHT_ENCODER)
    
    if (prev_right_state is not None and current_state != prev_right_state): 
        right_count += 1
        prev_right_state = current_state
        
    elif prev_right_state is None:
        prev_right_state = current_state
    
def reset_encoder():
    global left_count, right_count
    left_count, right_count = 0, 0

def set_motors(left, right):
    global prev_movement, current_movement
    
    # Pre-Start Kick (Motor Priming), to reduce initial jerk and slight orientation change
    if prev_movement == 'stop' and current_movement in ['forward', 'backward']:
        if current_movement  == 'forward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        elif current_movement == 'backward':
            GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
            GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
            GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
            GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)
        right_motor_pwm.ChangeDutyCycle(100)
        time.sleep(0.05)

    if right > 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
        right_motor_pwm.ChangeDutyCycle(min(right, 100))
    elif right < 0:
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(min(abs(right), 100))
    else:
        # when pwm = 0, implement Active Braking mode, better than putting duty cycle to 0 which may cause uneven stopping
        GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
        right_motor_pwm.ChangeDutyCycle(100)
    
    if left > 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
        left_motor_pwm.ChangeDutyCycle(min(left, 100))
    elif left < 0:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(min(abs(left), 100))
    else:
        GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
        left_motor_pwm.ChangeDutyCycle(100)
    
    
def apply_min_threshold(pwm_value, min_threshold):
    if pwm_value == 0:
        return 0  # Zero means stop
    elif abs(pwm_value) < min_threshold:
        # Boost small values to minimum threshold, preserving direction
        return min_threshold if pwm_value > 0 else -min_threshold
    else:
        return pwm_value

def apply_deadband_compensation(pwm_value, min_threshold=15, deadband_offset=8):
    """Improved deadband compensation with smooth transition"""
    if pwm_value == 0:
        return 0
    elif abs(pwm_value) < min_threshold:
        # Add deadband offset to overcome static friction
        sign = 1 if pwm_value > 0 else -1
        return sign * (min_threshold + deadband_offset)
    else:
        # Add smaller offset for running friction
        sign = 1 if pwm_value > 0 else -1
        return pwm_value + sign * (deadband_offset * 0.5)

##################### Rotational PID Helpers - ENHANCED #####################
def counts_to_dtheta_rad(dleft, dright):
    # dtheta = (sr - sl) / baseline
    sl = dleft * DIST_PER_TICK
    sr = dright * DIST_PER_TICK
    return (sr - sl) / BASELINE_M

def counts_to_dtheta_rad_improved(dleft, dright, dt):
    """Enhanced kinematics with filtering and validation"""
    global filtered_angular_velocity, velocity_history
    
    # Basic differential kinematics
    sl = dleft * DIST_PER_TICK
    sr = dright * DIST_PER_TICK
    raw_dtheta = (sr - sl) / BASELINE_M
    
    # Calculate instantaneous angular velocity
    raw_angular_velocity = raw_dtheta / dt if dt > 0 else 0.0
    
    # Apply low-pass filter to reduce encoder noise
    filtered_angular_velocity = (encoder_filter_alpha * filtered_angular_velocity + 
                                (1 - encoder_filter_alpha) * raw_angular_velocity)
    
    # Maintain velocity history for better derivative estimation
    velocity_history.append(filtered_angular_velocity)
    if len(velocity_history) > VELOCITY_HISTORY_SIZE:
        velocity_history.pop(0)
    
    # Use filtered velocity for integration
    filtered_dtheta = filtered_angular_velocity * dt
    
    return filtered_dtheta, filtered_angular_velocity

def rotation_begin():
    # Prepare rotation state (do not reset encoders; we integrate deltas)
    global rot_integral, rot_last_error, theta_rad, rot_last_theta, rot_last_time
    global last_left_count, last_right_count, rot_in_progress, rot_done, rot_start_time
    global angular_velocity_pid_integral, angular_velocity_pid_last_error, target_angular_velocity
    rot_integral = 0.0
    rot_last_error = 0.0
    theta_rad = 0.0
    rot_last_theta = 0.0
    rot_last_time = monotonic()
    last_left_count = left_count
    last_right_count = right_count
    rot_in_progress = True
    rot_done = False
    rot_start_time = monotonic()
    # Reset cascaded control state
    angular_velocity_pid_integral = 0.0
    angular_velocity_pid_last_error = 0.0
    target_angular_velocity = 0.0
    rot_debug(f"BEGIN target_rad={rot_target_rad:.4f} left_count={left_count} right_count={right_count}")

def rotation_finish():
    global rot_in_progress, rot_done, control_mode
    global angular_velocity_pid_integral, angular_velocity_pid_last_error, target_angular_velocity
    rot_in_progress = False
    rot_done = True
    control_mode = 'velocity'
    # Reset cascaded control state
    angular_velocity_pid_integral = 0.0
    angular_velocity_pid_last_error = 0.0
    target_angular_velocity = 0.0
    rot_debug(f"FINISH theta_rad={theta_rad:.4f} target_rad={rot_target_rad:.4f} error_rad={(rot_target_rad-theta_rad):.4f}")

def rotation_diagnostics(position_error_deg, current_vel_deg, target_vel_deg, final_left_pwm, final_right_pwm):
    """Print diagnostic information during rotation"""
    if control_mode == 'rotate' and rot_in_progress:
        print(f"[DIAG] Pos_err: {position_error_deg:6.2f}°, "
              f"Vel_curr: {current_vel_deg:6.1f}°/s, "
              f"Vel_tgt: {target_vel_deg:6.1f}°/s, "
              f"PWM: L={final_left_pwm:5.1f} R={final_right_pwm:5.1f}")

###################################################################
def pid_control():
    # Only applies for forward/backward, not turning
    global left_pwm, right_pwm, left_count, right_count, use_PID, KP, KI, KD, prev_movement, current_movement
    global control_mode, rot_target_rad, rot_integral, rot_last_error, theta_rad
    global last_left_count, last_right_count, rot_last_theta, rot_last_time
    global target_angular_velocity, angular_velocity_pid_integral, angular_velocity_pid_last_error

    integral = 0
    last_error = 0
    last_time = monotonic()
    
    # Ramping variables & params
    ramp_left_pwm = 0
    ramp_right_pwm = 0
    previous_left_target = 0
    previous_right_target = 0
    
    while running:          
        current_time = monotonic()
        dt = current_time - last_time
        last_time = current_time
        dt = max(dt, 1e-3)

        # Compute movement state from commanded wheel PWM (for velocity mode)
        prev_movement = current_movement
        if (left_pwm > 0 and right_pwm > 0): current_movement = 'forward'
        elif (left_pwm < 0 and right_pwm < 0): current_movement = 'backward'
        elif (left_pwm < 0 and right_pwm > 0): current_movement = 'turnleft'
        elif (left_pwm > 0 and right_pwm < 0): current_movement = 'turnright'
        elif (left_pwm == 0 and right_pwm == 0): current_movement = 'stop'

        if control_mode == 'rotate':
            # Incremental encoder integration with improved filtering
            dleft = left_count - last_left_count
            dright = right_count - last_right_count
            last_left_count = left_count
            last_right_count = right_count

            dtheta, current_angular_velocity = counts_to_dtheta_rad_improved(dleft, dright, dt)
            theta_rad += dtheta

            # OUTER LOOP: Position PID to generate target angular velocity
            position_error = rot_target_rad - theta_rad
            
            # Adaptive target velocity based on error magnitude
            max_angular_vel = 2.0  # rad/s - tune based on your robot
            target_angular_velocity = max(-max_angular_vel, min(max_angular_vel, 
                                         Kp_rot * position_error))
            
            # Add derivative term for position loop damping
            position_derivative = -current_angular_velocity  # derivative of position is velocity
            target_angular_velocity += Kd_rot * position_derivative

            # INNER LOOP: Angular velocity PID
            velocity_error = target_angular_velocity - current_angular_velocity
            angular_velocity_pid_integral += velocity_error * dt
            angular_velocity_pid_integral = max(-1.0, min(1.0, angular_velocity_pid_integral))  # Anti-windup
            
            velocity_derivative = (velocity_error - angular_velocity_pid_last_error) / dt
            angular_velocity_pid_last_error = velocity_error
            
            angular_control_output = (Kp_vel * velocity_error + 
                                     Ki_vel * angular_velocity_pid_integral + 
                                     Kd_vel * velocity_derivative)
            
            # Convert angular velocity command to differential wheel speeds
            # For pure rotation: v_left = -omega * baseline/2, v_right = +omega * baseline/2
            wheel_speed_diff = target_angular_velocity * BASELINE_M / 2
            
            # Convert to PWM with feedforward + feedback
            base_pwm = wheel_speed_diff * FEEDFORWARD_GAIN  # Feedforward gain
            correction_pwm = angular_control_output
            
            target_left_pwm = +(base_pwm + correction_pwm)
            target_right_pwm = -(base_pwm + correction_pwm)
            
            # Apply constraints
            target_left_pwm = max(-MAX_ROT_PWM, min(MAX_ROT_PWM, target_left_pwm))
            target_right_pwm = max(-MAX_ROT_PWM, min(MAX_ROT_PWM, target_right_pwm))

            final_left_pwm = apply_deadband_compensation(target_left_pwm)
            final_right_pwm = apply_deadband_compensation(target_right_pwm)
            set_motors(final_left_pwm, final_right_pwm)

            now = current_time

            # Improved completion detection
            within_error = abs(position_error) <= ROT_TOL_RAD
            velocity_settled = abs(current_angular_velocity) <= ROT_SETTLE_RATE
            small_control_effort = abs(angular_control_output) <= 5.0  # PWM units
            
            if within_error and velocity_settled and small_control_effort:
                if not hasattr(pid_control, "_settle_start") or pid_control._settle_start is None:
                    pid_control._settle_start = now
                elif (now - pid_control._settle_start) >= ROT_SETTLE_TIME:
                    set_motors(0, 0)
                    pid_control._settle_start = None
                    rotation_finish()
            else:
                pid_control._settle_start = None
                # Uncomment the next line during tuning
                # rotation_diagnostics(math.degrees(position_error), math.degrees(current_angular_velocity), 
                #                     math.degrees(target_angular_velocity), final_left_pwm, final_right_pwm)
            
            if rot_start_time is not None and (current_time - rot_start_time) > ROT_MAX_TIME:
                print("[ROT] Timeout reached, forcing finish.")
                set_motors(0,0)
                rotation_finish()
            time.sleep(0.01)
            continue

        # VELOCITY MODE: existing behavior (with straight-line/turn correction)
        if not use_PID:
            target_left_pwm = left_pwm
            target_right_pwm = right_pwm
        else:
            if current_movement in ('forward','backward','turnleft','turnright'):
                error = left_count - right_count
                proportional = KP * error
                integral += KI * error * dt
                integral = max(-MAX_CORRECTION, min(integral, MAX_CORRECTION))  # Anti-windup
                derivative = KD * (error - last_error) / dt if dt > 0 else 0
                correction = proportional + integral + derivative
                correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
                last_error = error

                if current_movement == 'forward':
                    target_left_pwm = left_pwm - correction
                    target_right_pwm = right_pwm + correction        
                elif current_movement == 'backward':       
                    target_left_pwm = left_pwm + correction
                    target_right_pwm = right_pwm - correction
                elif current_movement == 'turnleft':
                    target_left_pwm = left_pwm + correction
                    target_right_pwm = right_pwm + correction 
                elif current_movement == 'turnright':
                    target_left_pwm = left_pwm - correction
                    target_right_pwm = right_pwm - correction 
            else:
                # Reset when stopped (velocity mode only)
                integral = 0
                last_error = 0
                reset_encoder()
                target_left_pwm = left_pwm
                target_right_pwm = right_pwm
        
        if use_ramping and use_PID:
            # PWM Ramping Logic
            max_change_per_cycle = RAMP_RATE * dt
            
            # Calculate differences for both motors
            left_diff = target_left_pwm - ramp_left_pwm
            right_diff = target_right_pwm - ramp_right_pwm
            
            # Determine if either motor needs ramping
            left_needs_ramp = abs(left_diff) > MIN_RAMP_THRESHOLD
            right_needs_ramp = abs(right_diff) > MIN_RAMP_THRESHOLD
            
            # Check for direction change conditions (but not stops)
            left_direction_change = (target_left_pwm * previous_left_target < 0) and target_left_pwm != 0 and previous_left_target != 0
            right_direction_change = (target_right_pwm * previous_right_target < 0) and target_right_pwm != 0 and previous_right_target != 0
            
            # Apply immediate changes for direction changes only (for safety)
            if left_direction_change:
                ramp_left_pwm = target_left_pwm
            if right_direction_change:
                ramp_right_pwm = target_right_pwm
            
            # Synchronized ramping - both motors ramp together or not at all
            if not left_direction_change and not right_direction_change:
                if left_needs_ramp or right_needs_ramp:
                    
                    # Left motor ramping (including ramp-down to zero)
                    if abs(left_diff) <= max_change_per_cycle:
                        ramp_left_pwm = target_left_pwm  # Close enough, set to target
                    else:
                        # Ramp towards target (up or down)
                        if left_diff > 0:
                            ramp_left_pwm += max_change_per_cycle
                        else:
                            ramp_left_pwm -= max_change_per_cycle
                    
                    # Right motor ramping (including ramp-down to zero)
                    if abs(right_diff) <= max_change_per_cycle:
                        ramp_right_pwm = target_right_pwm  # Close enough, set to target
                    else:
                        # Ramp towards target (up or down)
                        if right_diff > 0:
                            ramp_right_pwm += max_change_per_cycle
                        else:
                            ramp_right_pwm -= max_change_per_cycle
                else:
                    # Neither motor needs ramping - apply targets directly
                    ramp_left_pwm = target_left_pwm
                    ramp_right_pwm = target_right_pwm
            
            # Store previous targets for next iteration
            previous_left_target = target_left_pwm
            previous_right_target = target_right_pwm
        
        else:
            # Ramping disabled - apply target values directly
            ramp_left_pwm = target_left_pwm
            ramp_right_pwm = target_right_pwm
            
        final_left_pwm = apply_min_threshold(ramp_left_pwm, MIN_PWM_THRESHOLD)
        final_right_pwm = apply_min_threshold(ramp_right_pwm, MIN_PWM_THRESHOLD)
        set_motors(final_left_pwm, final_right_pwm)
        
        if ramp_left_pwm != 0: # print for debugging purpose
            print(f"(Left PWM, Right PWM)=({ramp_left_pwm:.2f},{ramp_right_pwm:.2f}), (Left Enc, Right Enc)=({left_count}, {right_count})")
        
        time.sleep(0.01)


def camera_stream_server():
    # Initialize camera
    picam2 = Picamera2()
    camera_config = picam2.create_preview_configuration(lores={"size": (640,480)})
    picam2.configure(camera_config)
    picam2.start()
    
    # Create socket for streaming
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, CAMERA_PORT))
    server_socket.listen(1)
    print(f"Camera stream server started on port {CAMERA_PORT}")
    
    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"Camera stream client connected")
            
            while running:
                # Capture frame and convert to bytes
                stream = io.BytesIO()
                picam2.capture_file(stream, format='jpeg')
                stream.seek(0)
                jpeg_data = stream.getvalue()
                jpeg_size = len(jpeg_data)
                
                try:
                    client_socket.sendall(struct.pack("!I", jpeg_size))
                    client_socket.sendall(jpeg_data)
                except:
                    print("Camera stream client disconnected")
                    break
                
                # Small delay to avoid hogging CPU
                time.sleep(0.01)
                
        except Exception as e:
            print(f"Camera stream server error: {str(e)}")
        
        if 'client_socket' in locals() and client_socket:
            client_socket.close()
    
    server_socket.close()
    picam2.stop()

def pid_config_server():
    global use_PID, KP, KI, KD
    
    # Create socket for receiving PID configuration
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PID_CONFIG_PORT))
    server_socket.listen(1)
    print(f"PID config server started on port {PID_CONFIG_PORT}")
    
    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"PID config client connected")
            
            try:
                # Receive PID constants (4 floats)
                data = client_socket.recv(16)
                if data and len(data) == 16:
                    use_PID, KP, KI, KD = struct.unpack("!ffff", data)
                    if use_PID: print(f"Updated PID constants: KP={KP}, KI={KI}, KD={KD}")
                    else: print("The robot is not using PID.")
                    
                    # Send acknowledgment (1 for success)
                    response = struct.pack("!i", 1)
                else:
                    # Send failure response
                    response = struct.pack("!i", 0)
                
                client_socket.sendall(response)
                    
            except Exception as e:
                print(f"PID config socket error: {str(e)}")
                try:
                    response = struct.pack("!i", 0)
                    client_socket.sendall(response)
                except:
                    pass
                    
            client_socket.close()
                    
        except Exception as e:
            print(f"PID config server error: {str(e)}")
    
    server_socket.close()
    

def wheel_server():
    global left_pwm, right_pwm, running, left_count, right_count
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, WHEEL_PORT))
    server_socket.listen(1)
    print(f"Wheel server started on port {WHEEL_PORT}")
    
    while running:
        try:
            client_socket, _ = server_socket.accept()
            print(f"Wheel client connected")
            
            while running:
                try:
                    # Receive speed (4 bytes for each value)
                    data = client_socket.recv(8)
                    if not data or len(data) != 8:
                        print("Wheel client sending speed error")
                        break
                    
                    # Unpack speed values and convert to PWM
                    left_speed, right_speed = struct.unpack("!ff", data)
                    # print(f"Received wheel: left_speed={left_speed:.4f}, right_speed={right_speed:.4f}")
                    left_pwm, right_pwm = left_speed*100, right_speed*100
                    
                    # Send encoder counts back
                    response = struct.pack("!ii", left_count, right_count)
                    client_socket.sendall(response)
                    
                except Exception as e:
                    print(f"Wheel client disconnected")
                    break
                    
        except Exception as e:
            print(f"Wheel server error: {str(e)}")
        
        if 'client_socket' in locals() and client_socket:
            client_socket.close()
    
    server_socket.close()
###################################################################################################
def rotation_server():
    global control_mode, rot_target_rad
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, ROTATE_PORT))
    server_socket.listen(1)
    print(f"Rotation server started on port {ROTATE_PORT} (radians)")
    while running:
        try:
            client_socket, _ = server_socket.accept()
            try:
                data = client_socket.recv(4)
                if not data or len(data) != 4:
                    client_socket.sendall(struct.pack("!i", 0))
                    client_socket.close()
                    continue
                target_rad = struct.unpack("!f", data)[0]
                rot_debug(f"COMMAND received target_rad={target_rad:.4f}")
                rot_target_rad = target_rad
                control_mode = 'rotate'
                rotation_begin()
                client_socket.sendall(struct.pack("!i", 1))
            except:
                try:
                    client_socket.sendall(struct.pack("!i", 0))
                except:
                    pass
            finally:
                client_socket.close()
        except Exception as e:
            print(f"Rotation server error: {e}")
    server_socket.close()

def rotation_status_server():
    # Respond with: in_progress(int), done(int), current_rad(float), target_rad(float)
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, ROTATE_STATUS_PORT))
    server_socket.listen(1)
    print(f"Rotation status server started on port {ROTATE_STATUS_PORT} (radians)")
    while running:
        try:
            client_socket, _ = server_socket.accept()
            try:
                in_progress = 1 if rot_in_progress else 0
                done = 1 if rot_done else 0
                current_rad = float(theta_rad)
                target_rad = float(rot_target_rad) if rot_in_progress or rot_done else 0.0
                payload = struct.pack("!iiff", in_progress, done, current_rad, target_rad)
                client_socket.sendall(payload)
            except Exception:
                pass
            finally:
                client_socket.close()
        except Exception as e:
            print(f"Rotation status server error: {e}")
    server_socket.close()

def pid_rot_config_server():
    global Kp_rot, Ki_rot, Kd_rot
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PID_ROT_CONFIG_PORT))
    server_socket.listen(1)
    print(f"Rotation PID config server started on port {PID_ROT_CONFIG_PORT}")
    
    while running:
        try:
            client_socket, _ = server_socket.accept()
            try:
                data = client_socket.recv(12)
                if data and len(data) == 12:
                    kp, ki, kd = struct.unpack("!fff", data)
                    Kp_rot, Ki_rot, Kd_rot = float(kp), float(ki), float(kd)
                    print(f"Updated rotation PID: Kp={Kp_rot}, Ki={Ki_rot}, Kd={Kd_rot}")
                    response = struct.pack("!i", 1)
                else:
                    response = struct.pack("!i", 0)
                client_socket.sendall(response)
            except Exception as e:
                print(f"Rotation PID config socket error: {str(e)}")
                try:
                    response = struct.pack("!i", 0)
                    client_socket.sendall(response)
                except:
                    pass
            finally:
                client_socket.close()
        except Exception as e:
            print(f"Rotation PID config server error: {str(e)}")
    
    server_socket.close()
###################################################################################################

def main():
    try:
        setup_gpio()
        
        # Start PID control thread
        pid_thread = threading.Thread(target=pid_control)
        pid_thread.daemon = True
        pid_thread.start()
        
        # Start camera streaming thread
        camera_thread = threading.Thread(target=camera_stream_server)
        camera_thread.daemon = True
        camera_thread.start()
        
        # Start PID configuration server thread
        pid_config_thread = threading.Thread(target=pid_config_server)
        pid_config_thread.daemon = True
        pid_config_thread.start()

        # Start rotation PID configuration server
        pid_rot_config_thread = threading.Thread(target=pid_rot_config_server)
        pid_rot_config_thread.daemon = True
        pid_rot_config_thread.start()

        # Start rotation server (angle commands)
        rotation_thread = threading.Thread(target=rotation_server)
        rotation_thread.daemon = True
        rotation_thread.start()

        # Start rotation status server
        rotation_status_thread = threading.Thread(target=rotation_status_server)
        rotation_status_thread.daemon = True
        rotation_status_thread.start()
        # Start wheel server (main thread)
        wheel_server()
        
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        global running
        running = False
        GPIO.cleanup()
        print("Cleanup complete")

if __name__ == "__main__":
    main()

# import socket
# import struct
# import io
# import threading
# import time
# from time import monotonic
# import RPi.GPIO as GPIO
# from picamera2 import Picamera2

# # Network Configuration
# HOST = '0.0.0.0'
# WHEEL_PORT = 8000
# CAMERA_PORT = 8001
# PID_CONFIG_PORT = 8002

# # Pins
# RIGHT_MOTOR_ENA = 18
# RIGHT_MOTOR_IN1 = 17
# RIGHT_MOTOR_IN2 = 27
# LEFT_MOTOR_ENB = 25
# LEFT_MOTOR_IN3 = 23
# LEFT_MOTOR_IN4 = 24
# LEFT_ENCODER = 26
# RIGHT_ENCODER = 16

# # PID Constants (default values, will be overridden by client)
# use_PID = 0
# KP, Ki, KD = 0, 0, 0
# MAX_CORRECTION = 30  # Maximum PWM correction value

# # Global variables
# running = True
# left_pwm, right_pwm = 0, 0
# left_count, right_count = 0, 0
# prev_left_state, prev_right_state = None, None
# use_ramping = True
# RAMP_RATE = 250  # PWM units per second (adjust this value to tune ramp speed)
# MIN_RAMP_THRESHOLD = 15  # Only ramp if change is greater than this
# MIN_PWM_THRESHOLD = 15
# current_movement, prev_movement = 'stop', 'stop'

# def setup_gpio():
#     GPIO.setmode(GPIO.BCM)
#     GPIO.setwarnings(False)
    
#     # Motor
#     GPIO.setup(RIGHT_MOTOR_ENA, GPIO.OUT)
#     GPIO.setup(RIGHT_MOTOR_IN1, GPIO.OUT)
#     GPIO.setup(RIGHT_MOTOR_IN2, GPIO.OUT)
#     GPIO.setup(LEFT_MOTOR_ENB, GPIO.OUT)
#     GPIO.setup(LEFT_MOTOR_IN3, GPIO.OUT)
#     GPIO.setup(LEFT_MOTOR_IN4, GPIO.OUT)
    
#     # This prevents slight motor jerk when connection is established
#     GPIO.output(RIGHT_MOTOR_ENA, GPIO.LOW)
#     GPIO.output(LEFT_MOTOR_ENB, GPIO.LOW)
    
#     # Encoder setup and interrupt (both activated and deactivated)
#     GPIO.setup(LEFT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#     GPIO.setup(RIGHT_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
#     GPIO.add_event_detect(LEFT_ENCODER, GPIO.BOTH, callback=left_encoder_callback)
#     GPIO.add_event_detect(RIGHT_ENCODER, GPIO.BOTH, callback=right_encoder_callback)
    
#     # Initialize PWM (frequency: 100Hz)
#     global left_motor_pwm, right_motor_pwm
#     left_motor_pwm = GPIO.PWM(LEFT_MOTOR_ENB, 100)
#     right_motor_pwm = GPIO.PWM(RIGHT_MOTOR_ENA, 100)
#     left_motor_pwm.start(0)
#     right_motor_pwm.start(0)

# def left_encoder_callback(channel):
#     global left_count, prev_left_state
#     current_state = GPIO.input(LEFT_ENCODER)
    
#     # Check for actual state change. Without this, false positive happens due to electrical noise
#     # After testing, debouncing not needed
#     if (prev_left_state is not None and current_state != prev_left_state):       
#         left_count += 1
#         prev_left_state = current_state
    
#     elif prev_left_state is None:
#         # First reading
#         prev_left_state = current_state

# def right_encoder_callback(channel):
#     global right_count, prev_right_state, prev_right_time
#     current_state = GPIO.input(RIGHT_ENCODER)
    
#     if (prev_right_state is not None and current_state != prev_right_state): 
#         right_count += 1
#         prev_right_state = current_state
        
#     elif prev_right_state is None:
#         prev_right_state = current_state
    
# def reset_encoder():
#     global left_count, right_count
#     left_count, right_count = 0, 0

# def set_motors(left, right):
#     global prev_movement, current_movement
    
#     # Pre-Start Kick (Motor Priming), to reduce initial jerk and slight orientation change
#     if prev_movement == 'stop' and current_movement in ['forward', 'backward']:
#         if current_movement  == 'forward':
#             GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
#             GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
#             GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
#             GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
#         elif current_movement == 'backward':
#             GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
#             GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
#             GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
#             GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
#         left_motor_pwm.ChangeDutyCycle(100)
#         right_motor_pwm.ChangeDutyCycle(100)
#         time.sleep(0.05)

#     if right > 0:
#         GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
#         GPIO.output(RIGHT_MOTOR_IN2, GPIO.LOW)
#         right_motor_pwm.ChangeDutyCycle(min(right, 100))
#     elif right < 0:
#         GPIO.output(RIGHT_MOTOR_IN1, GPIO.LOW)
#         GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
#         right_motor_pwm.ChangeDutyCycle(min(abs(right), 100))
#     else:
#         # when pwm = 0, implement Active Braking mode, better than putting duty cycle to 0 which may cause uneven stopping
#         GPIO.output(RIGHT_MOTOR_IN1, GPIO.HIGH)
#         GPIO.output(RIGHT_MOTOR_IN2, GPIO.HIGH)
#         right_motor_pwm.ChangeDutyCycle(100)
    
#     if left > 0:
#         GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
#         GPIO.output(LEFT_MOTOR_IN4, GPIO.LOW)
#         left_motor_pwm.ChangeDutyCycle(min(left, 100))
#     elif left < 0:
#         GPIO.output(LEFT_MOTOR_IN3, GPIO.LOW)
#         GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
#         left_motor_pwm.ChangeDutyCycle(min(abs(left), 100))
#     else:
#         GPIO.output(LEFT_MOTOR_IN3, GPIO.HIGH)
#         GPIO.output(LEFT_MOTOR_IN4, GPIO.HIGH)
#         left_motor_pwm.ChangeDutyCycle(100)
    
    
# def apply_min_threshold(pwm_value, min_threshold):
#     if pwm_value == 0:
#         return 0  # Zero means stop
#     elif abs(pwm_value) < min_threshold:
#         # Boost small values to minimum threshold, preserving direction
#         return min_threshold if pwm_value > 0 else -min_threshold
#     else:
#         return pwm_value

# def pid_control():
#     # Only applies for forward/backward, not turning
#     global left_pwm, right_pwm, left_count, right_count, use_PID, KP, KI, KD, prev_movement, current_movement
    
#     integral = 0
#     last_error = 0
#     last_time = monotonic()
    
#     # Ramping variables & params
#     ramp_left_pwm = 0
#     ramp_right_pwm = 0
#     previous_left_target = 0
#     previous_right_target = 0
    
#     while running:          
#         current_time = monotonic()
#         dt = current_time - last_time
#         last_time = current_time
        
#         prev_movement = current_movement
#         if (left_pwm > 0 and right_pwm > 0): current_movement = 'forward'
#         elif (left_pwm < 0 and right_pwm < 0): current_movement = 'backward'
#         elif (left_pwm < 0 and right_pwm > 0): current_movement = 'turnleft'
#         elif (left_pwm > 0 and right_pwm < 0): current_movement = 'turnright'
#         elif (left_pwm == 0 and right_pwm == 0): current_movement = 'stop'
        
#         if not use_PID:
#             target_left_pwm = left_pwm
#             target_right_pwm = right_pwm
#         else:
#             if current_movement == 'forward' or current_movement == 'backward' or current_movement == 'turnleft'or current_movement == 'turnright':
                
#                 error = left_count - right_count
#                 proportional = KP * error
#                 integral += KI * error * dt
#                 integral = max(-MAX_CORRECTION, min(integral, MAX_CORRECTION))  # Anti-windup
#                 derivative = KD * (error - last_error) / dt if dt > 0 else 0
#                 correction = proportional + integral + derivative
#                 correction = max(-MAX_CORRECTION, min(correction, MAX_CORRECTION))
#                 last_error = error

#                 if current_movement == 'forward':
#                     target_left_pwm = left_pwm - correction
#                     target_right_pwm = right_pwm + correction        
#                 elif current_movement == 'backward':       
#                     target_left_pwm = left_pwm + correction
#                     target_right_pwm = right_pwm - correction
#                 elif current_movement == 'turnleft':
#                     target_left_pwm = left_pwm + correction
#                     target_right_pwm = right_pwm + correction 
#                 elif current_movement == 'turnright':
#                     target_left_pwm = left_pwm - correction
#                     target_right_pwm = right_pwm - correction 

#             else:
#                 # Reset when stopped
#                 integral = 0
#                 last_error = 0
#                 reset_encoder()
#                 target_left_pwm = left_pwm
#                 target_right_pwm = right_pwm
        
#         if use_ramping and use_PID:
#             # PWM Ramping Logic
#             max_change_per_cycle = RAMP_RATE * dt
            
#             # Calculate differences for both motors
#             left_diff = target_left_pwm - ramp_left_pwm
#             right_diff = target_right_pwm - ramp_right_pwm
            
#             # Determine if either motor needs ramping
#             left_needs_ramp = abs(left_diff) > MIN_RAMP_THRESHOLD
#             right_needs_ramp = abs(right_diff) > MIN_RAMP_THRESHOLD
            
#             # Check for direction change conditions (but not stops)
#             left_direction_change = (target_left_pwm * previous_left_target < 0) and target_left_pwm != 0 and previous_left_target != 0
#             right_direction_change = (target_right_pwm * previous_right_target < 0) and target_right_pwm != 0 and previous_right_target != 0
            
#             # Apply immediate changes for direction changes only (for safety)
#             if left_direction_change:
#                 ramp_left_pwm = target_left_pwm
#             if right_direction_change:
#                 ramp_right_pwm = target_right_pwm
            
#             # Synchronized ramping - both motors ramp together or not at all
#             if not left_direction_change and not right_direction_change:
#                 if left_needs_ramp or right_needs_ramp:
                    
#                     # Left motor ramping (including ramp-down to zero)
#                     if abs(left_diff) <= max_change_per_cycle:
#                         ramp_left_pwm = target_left_pwm  # Close enough, set to target
#                     else:
#                         # Ramp towards target (up or down)
#                         if left_diff > 0:
#                             ramp_left_pwm += max_change_per_cycle
#                         else:
#                             ramp_left_pwm -= max_change_per_cycle
                    
#                     # Right motor ramping (including ramp-down to zero)
#                     if abs(right_diff) <= max_change_per_cycle:
#                         ramp_right_pwm = target_right_pwm  # Close enough, set to target
#                     else:
#                         # Ramp towards target (up or down)
#                         if right_diff > 0:
#                             ramp_right_pwm += max_change_per_cycle
#                         else:
#                             ramp_right_pwm -= max_change_per_cycle
#                 else:
#                     # Neither motor needs ramping - apply targets directly
#                     ramp_left_pwm = target_left_pwm
#                     ramp_right_pwm = target_right_pwm
            
#             # Store previous targets for next iteration
#             previous_left_target = target_left_pwm
#             previous_right_target = target_right_pwm
        
#         else:
#             # Ramping disabled - apply target values directly
#             ramp_left_pwm = target_left_pwm
#             ramp_right_pwm = target_right_pwm
            
#         final_left_pwm = apply_min_threshold(ramp_left_pwm, MIN_PWM_THRESHOLD)
#         final_right_pwm = apply_min_threshold(ramp_right_pwm, MIN_PWM_THRESHOLD)
#         set_motors(final_left_pwm, final_right_pwm)
        
#         if ramp_left_pwm != 0: # print for debugging purpose
#             print(f"(Left PWM, Right PWM)=({ramp_left_pwm:.2f},{ramp_right_pwm:.2f}), (Left Enc, Right Enc)=({left_count}, {right_count})")
        
#         time.sleep(0.01)


# def camera_stream_server():
#     # Initialize camera
#     picam2 = Picamera2()
#     camera_config = picam2.create_preview_configuration(lores={"size": (640,480)})
#     picam2.configure(camera_config)
#     picam2.start()
    
#     # Create socket for streaming
#     server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#     server_socket.bind((HOST, CAMERA_PORT))
#     server_socket.listen(1)
#     print(f"Camera stream server started on port {CAMERA_PORT}")
    
#     while running:
#         try:
#             client_socket, _ = server_socket.accept()
#             print(f"Camera stream client connected")
            
#             while running:
#                 # Capture frame and convert to bytes
#                 stream = io.BytesIO()
#                 picam2.capture_file(stream, format='jpeg')
#                 stream.seek(0)
#                 jpeg_data = stream.getvalue()
#                 jpeg_size = len(jpeg_data)
                
#                 try:
#                     client_socket.sendall(struct.pack("!I", jpeg_size))
#                     client_socket.sendall(jpeg_data)
#                 except:
#                     print("Camera stream client disconnected")
#                     break
                
#                 # Small delay to avoid hogging CPU
#                 time.sleep(0.01)
                
#         except Exception as e:
#             print(f"Camera stream server error: {str(e)}")
        
#         if 'client_socket' in locals() and client_socket:
#             client_socket.close()
    
#     server_socket.close()
#     picam2.stop()


# def pid_config_server():
#     global use_PID, KP, KI, KD
    
#     # Create socket for receiving PID configuration
#     server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#     server_socket.bind((HOST, PID_CONFIG_PORT))
#     server_socket.listen(1)
#     print(f"PID config server started on port {PID_CONFIG_PORT}")
    
#     while running:
#         try:
#             client_socket, _ = server_socket.accept()
#             print(f"PID config client connected")
            
#             try:
#                 # Receive PID constants (4 floats)
#                 data = client_socket.recv(16)
#                 if data and len(data) == 16:
#                     use_PID, KP, KI, KD = struct.unpack("!ffff", data)
#                     if use_PID: print(f"Updated PID constants: KP={KP}, KI={KI}, KD={KD}")
#                     else: print("The robot is not using PID.")
                    
#                     # Send acknowledgment (1 for success)
#                     response = struct.pack("!i", 1)
#                 else:
#                     # Send failure response
#                     response = struct.pack("!i", 0)
                
#                 client_socket.sendall(response)
                    
#             except Exception as e:
#                 print(f"PID config socket error: {str(e)}")
#                 try:
#                     response = struct.pack("!i", 0)
#                     client_socket.sendall(response)
#                 except:
#                     pass
                    
#             client_socket.close()
                    
#         except Exception as e:
#             print(f"PID config server error: {str(e)}")
    
#     server_socket.close()
    

# def wheel_server():
#     global left_pwm, right_pwm, running, left_count, right_count
    
#     server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
#     server_socket.bind((HOST, WHEEL_PORT))
#     server_socket.listen(1)
#     print(f"Wheel server started on port {WHEEL_PORT}")
    
#     while running:
#         try:
#             client_socket, _ = server_socket.accept()
#             print(f"Wheel client connected")
            
#             while running:
#                 try:
#                     # Receive speed (4 bytes for each value)
#                     data = client_socket.recv(8)
#                     if not data or len(data) != 8:
#                         print("Wheel client sending speed error")
#                         break
                    
#                     # Unpack speed values and convert to PWM
#                     left_speed, right_speed = struct.unpack("!ff", data)
#                     # print(f"Received wheel: left_speed={left_speed:.4f}, right_speed={right_speed:.4f}")
#                     left_pwm, right_pwm = left_speed*100, right_speed*100
                    
#                     # Send encoder counts back
#                     response = struct.pack("!ii", left_count, right_count)
#                     client_socket.sendall(response)
                    
#                 except Exception as e:
#                     print(f"Wheel client disconnected")
#                     break
                    
#         except Exception as e:
#             print(f"Wheel server error: {str(e)}")
        
#         if 'client_socket' in locals() and client_socket:
#             client_socket.close()
    
#     server_socket.close()


# def main():
#     try:
#         setup_gpio()
        
#         # Start PID control thread
#         pid_thread = threading.Thread(target=pid_control)
#         pid_thread.daemon = True
#         pid_thread.start()
        
#         # Start camera streaming thread
#         camera_thread = threading.Thread(target=camera_stream_server)
#         camera_thread.daemon = True
#         camera_thread.start()
        
#         # Start PID configuration server thread
#         pid_config_thread = threading.Thread(target=pid_config_server)
#         pid_config_thread.daemon = True
#         pid_config_thread.start()
        
#         # Start wheel server (main thread)
#         wheel_server()
        
#     except KeyboardInterrupt:
#         print("Stopping...")
#     finally:
#         global running
#         running = False
#         GPIO.cleanup()
#         print("Cleanup complete")


# if __name__ == "__main__":
#     main()