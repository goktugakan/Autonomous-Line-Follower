# -*- coding: utf-8 -*-
"""
Autonomous Line Follower Robot with Obstacle Avoidance
------------------------------------------------------
Hardware: Raspberry Pi 5, Camera Module 3, L298N Driver
Author: Melih AKAN
Description: Uses Computer Vision (HSV Thresholding + Histogram) to follow a black line 
and detect blue obstacles. Implements a PI Controller for smooth steering.
"""

import cv2
import numpy as np
import time
from gpiozero import PWMOutputDevice, DigitalOutputDevice
from picamera2 import Picamera2

# --- PIN CONFIGURATION (L298N Driver) ---
# Left Motor
ENA = PWMOutputDevice(12, frequency=1000)
IN1 = DigitalOutputDevice(5)
IN2 = DigitalOutputDevice(6)

# Right Motor
ENB = PWMOutputDevice(13, frequency=1000)
IN3 = DigitalOutputDevice(16)
IN4 = DigitalOutputDevice(20)

# --- SPEED SETTINGS ---
MAX_SPEED = 0.25
MIN_SPEED = -0.25
BASE_SPEED = 0.25

# --- PI CONTROLLER CONSTANTS ---
KP = 0.5
KI = 0.08

def set_motor(pwm_device, in_a, in_b, speed):
    """
    Controls individual motor speed and direction.
    """
    # Clip speed to defined limits
    if speed > MAX_SPEED: speed = MAX_SPEED
    if speed < MIN_SPEED: speed = MIN_SPEED
    
    # Deadzone to prevent buzzing at low power
    if -0.10 < speed < 0.10: speed = 0
    
    if speed > 0:
        in_a.on(); in_b.off(); pwm_device.value = speed
    elif speed < 0:
        in_a.off(); in_b.on(); pwm_device.value = abs(speed)
    else:
        in_a.off(); in_b.off(); pwm_device.value = 0.0
    
    return speed

def stop_all():
    """Stops all motors immediately."""
    ENA.value = 0; ENB.value = 0
    IN1.off(); IN2.off(); IN3.off(); IN4.off()

def pi_controller(error, integral, dt):
    """
    Calculates the correction value using Proportional-Integral control.
    Includes anti-windup for the integral term.
    """
    P = KP * error
    integral += error * dt
    
    # Anti-windup clamping
    if integral > 5: integral = 5
    if integral < -5: integral = -5
    
    I = KI * integral
    return P + I, integral

# --- OBSTACLE AVOIDANCE ROUTINE ---
def blue_obstacle_maneuver(picam2):
    """
    Executes a hardcoded maneuver to bypass an obstacle when detected.
    """
    print("[WARNING] Obstacle Detected! Initiating evasion maneuver...")
    stop_all()
    time.sleep(0.5)

    # STEP 1: Turn Left (Escape obstacle)
    print("-> Step 1: Turning Left...")
    set_motor(ENA, IN1, IN2, -1.5)  # Left Back
    set_motor(ENB, IN3, IN4, 1.5)   # Right Forward
    time.sleep(1.3) 

    # STEP 2: Arc Right to find the line again
    print("-> Step 2: Searching for line (Right Arc)...")
    
    # Left motor faster, Right motor slower = Right Arc
    set_motor(ENA, IN1, IN2, 0.6) 
    set_motor(ENB, IN3, IN4, 0.6) 
    
    start_search = time.time()
    
    # Search for the line for max 10 seconds
    while (time.time() - start_search) < 10.0:
        image = picam2.capture_array()
        if image is None: continue
        
        # BGR -> HSV Conversion
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        
        # Line Mask (Black)
        lower_black = np.array([0, 94, 50]) 
        upper_black = np.array([180, 194, 140]) 
        mask_black = cv2.inRange(hsv, lower_black, upper_black)
        
        # Morphological operations to clean noise
        kernel = np.ones((7,7), np.uint8)
        mask_black = cv2.erode(mask_black, kernel, iterations=1)
        mask_black = cv2.dilate(mask_black, kernel, iterations=1)
        
        histogram = np.sum(mask_black, axis=0)
        max_val = np.max(histogram)
        
        cv2.imshow("Maneuver View", mask_black)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        # If we see a significant amount of the line
        if max_val > 55000:
            print(f"[SUCCESS] Line Regained! (Signal Strength: {max_val}) - Maneuver Complete.")
            break 
        
    stop_all()
    time.sleep(0.2)

def main():
    print("[INFO] Initializing Robot Systems...")
    
    try:
        picam2 = Picamera2()
        config = picam2.create_video_configuration(
            main={"size": (1280, 720), "format": "BGR888"}
        )
        picam2.configure(config)
        picam2.start()
        print("[INFO] Camera Active (Resolution: 1280x720).")
    except Exception as e:
        print(f"[ERROR] Camera initialization failed: {e}")
        return

    WIDTH = 1280
    HEIGHT = 720
    CENTER_X = WIDTH / 2 
    
    prev_time = time.time()
    integral = 0
    frame_count = 0
    fps_start_time = time.time()
    fps = 0
    
    time.sleep(1.0) # Warmup
    
    try:
        while True:
            image = picam2.capture_array()
            if image is None: continue
            
            # Convert frame to HSV (Hue, Saturation, Value)
            hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
            
            # --- 1. OBSTACLE DETECTION (BLUE OBJECT) ---
            # Hue: 90-130 (Blue tones)
            # Saturation: 120-255 (Filters out pale colors/glare)
            # Value: 50-255 (Filters out dark shadows)
            lower_blue = np.array([91, 100, 181]) 
            upper_blue = np.array([105, 255, 255])
            
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
            
            # Noise reduction for blue mask
            kernel = np.ones((5,5), np.uint8)
            mask_blue = cv2.erode(mask_blue, kernel, iterations=2) 
            mask_blue = cv2.dilate(mask_blue, kernel, iterations=2)
            
            blue_pixels = np.sum(mask_blue > 0)
            
            # Debug: Log excessive blue pixels
            if blue_pixels > 1000 and frame_count % 10 == 0:
                 print(f"DEBUG: Blue Pixel Count: {blue_pixels}")
            
            # Threshold for obstacle reaction
            if blue_pixels > 100000:
                print(f"[ALERT] OBSTACLE DETECTED: {blue_pixels} px")
                
                cv2.putText(image, "OBSTACLE!", (50, 200), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 3)
                cv2.imshow("Camera View", image)
                cv2.waitKey(1)
                
                blue_obstacle_maneuver(picam2)
                
                # Reset PID terms after maneuver
                integral = 0
                prev_time = time.time()
                frame_count = 0
                continue 

            # --- 2. LINE FOLLOWING (BLACK LINE) ---
            lower_black = np.array([0, 94, 50]) 
            upper_black = np.array([180, 194, 140]) 
            mask_black = cv2.inRange(hsv, lower_black, upper_black)

            histogram = np.sum(mask_black, axis=0)
            max_val = np.max(histogram)
            
            if max_val < 3000:
                # Line lost or not visible
                stop_all()
                integral = 0
            else:
                line_x = np.argmax(histogram)
                raw_error = (line_x - CENTER_X) / CENTER_X
               
                # --- TIME CALCULATION --- 
                curr_time = time.time()
                dt = curr_time - prev_time
                prev_time = curr_time
                if dt > 0.5: dt = 0.0 # Safety clamp for dt
                
                # --- PID CALCULATION ---
                correction, integral = pi_controller(raw_error, integral, dt)
               
                # --- DIFFERENTIAL STEERING ---
                left_speed = BASE_SPEED - correction
                right_speed = BASE_SPEED + correction
                
                # Apply speeds to motors
                set_motor(ENA, IN1, IN2, left_speed)
                set_motor(ENB, IN3, IN4, right_speed)

            # --- FPS & DISPLAY ---
            frame_count += 1
            if frame_count % 10 == 0:
                end_time = time.time()
                fps = 10 / (end_time - fps_start_time)
                fps_start_time = end_time
                print(f"FPS: {fps:.1f}")

            if frame_count % 3 == 0:
                display_img = cv2.resize(image, (640, 360))
                disp_center = int(CENTER_X / 2)
                disp_height = int(HEIGHT / 2)
                
                cv2.putText(display_img, f"FPS: {fps:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(display_img, f"BluePx: {blue_pixels}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                
                # Draw Center Line
                cv2.line(display_img, (disp_center, 0), (disp_center, disp_height), (255, 0, 0), 2)
                
                # Draw Detected Line Position
                if max_val > 3000:
                    disp_line_x = int(line_x / 2)
                    cv2.circle(display_img, (disp_line_x, int(disp_height/2)), 5, (0, 255, 0), -1)
                
                # Show Windows
                cv2.imshow("Camera View", mask_black) # Showing mask for debugging
                cv2.imshow("Blue Mask", cv2.resize(mask_blue, (640, 360)))
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
    except KeyboardInterrupt:
        print("\n[INFO] Stopping Robot...")
    finally:
        stop_all()
        picam2.stop()
        cv2.destroyAllWindows()
        print("[INFO] Program Terminated.")

if __name__ == "__main__":
    main()