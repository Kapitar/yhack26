import pyrealsense2 as rs
import cv2
import numpy as np
import logging
from ultralytics import YOLO

# Import local UART driver for motor control
from uart_driver import MiniAutoDriver
# Import our new A* Path Planner
from path_planner import PathPlanner

logging.basicConfig(level=logging.INFO)

# Configuration for Avoidance
CRITICAL_DISTANCE = 0.5 # meters
DEFAULT_SPEED = 100

# Init Path Planner (Grid starts at robot: width 4m, height 4m, 10cm cells)
planner = PathPlanner(width=4.0, height=4.0, resolution=0.1, inflation_radius=0.4)

#you only look once model
model = YOLO("yolov8n.pt")

#connection to the camera
pipeline = rs.pipeline()
config = rs.config()

#we want two streams (color & depth), both at 640x480 / 15fps
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
pipeline.start(config)
#aligns both streams
align = rs.align(rs.stream.color)

print("Starting Camera & Motor Driver...")
with MiniAutoDriver() as driver:
    print("Driver connected. Starting A* planning loop.")
    
    while True:
        #latest frames
        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)
        
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        if not color_frame or not depth_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        results = model(frame)
        
        should_stop = False
        planner.reset_map() # Clear last frame's grid
        
        #loop through all objects yolo detected
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            label = model.names[int(box.cls[0])]
            conf = float(box.conf[0])
            
            # center point of box
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            distance = depth_frame.get_distance(cx, cy)
            
            # RealSense invalid distance check
            if distance <= 0.01:
                continue
                
            # Emergency Stop Check
            if distance < CRITICAL_DISTANCE and (200 < cx < 440):
                should_stop = True
            
            # MATH: Project pixel coordinates into physical meters (approx 87 deg H-FOV)
            focal_length = 337.0 
            world_X = ((cx - 320) / focal_length) * distance
            world_Y = distance
            
            # Register on 2D mapping grid
            planner.add_obstacle(world_X, world_Y)
            
            # Draw visual
            color = (0, 0, 255) if distance < CRITICAL_DISTANCE else (255, 165, 0)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{label} | X:{world_X:.1f} Y:{world_Y:.1f}m", 
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # ======== MOTOR CONTROL LOGIC (A* + Pure Pursuit) ========
        
        if should_stop:
            driver.stop()
            cv2.putText(frame, "STATE: EMERGENCY STOP", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        else:
            # Plan path from robot (0,0) to target point 3 meters directly ahead (0, 3)
            path = planner.astar(start_world=(0.0, 0.0), goal_world=(0.0, 3.0))
            
            if not path:
                # No path found around objects -> blocked completely
                driver.stop()
                cv2.putText(frame, "STATE: PATH BLOCKED", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
            else:
                # We have a mathematical trajectory! Feed to Pure Pursuit controller.
                # Use SHORT lookahead (1.0m) to catch the curve EARLY, not overshoot it
                target_angle = planner.pure_pursuit((0.0, 0.0), path, lookahead=1.0)
                
                # AMPLIFY the angle: A* produces subtle 5-15° deviations because the path
                # only needs to shift ~0.5m sideways over 3m forward. But Mecanum wheels
                # need 45°+ angles to produce visible lateral motion. Multiply by 4x.
                amplified_angle = target_angle * 4.0
                
                # Clamp to max ±90° (pure lateral strafe)
                amplified_angle = max(-90.0, min(90.0, amplified_angle))
                
                # Dead zone: if angle is tiny (<3°), just go straight — no jitter
                if abs(amplified_angle) < 3.0:
                    amplified_angle = 0.0
                
                # Dynamic speed: slow down if the curve is sharp
                if abs(amplified_angle) > 45:
                    vel = 60
                else:
                    vel = DEFAULT_SPEED
                    
                # Map to Arduino angle convention:
                # Arduino: 0=Forward, 90=Left_Strafe, 270=Right_Strafe
                # amplified_angle: negative=Left, positive=Right
                if amplified_angle < 0: # Path is Left
                    motor_angle = abs(amplified_angle)  # 0→90
                else: # Path is Right  
                    motor_angle = 360 - amplified_angle  # 270→360
                    
                driver.move(angle=int(motor_angle), velocity=int(vel), rot=0)
                cv2.putText(frame, f"Raw:{target_angle:.1f} Amp:{amplified_angle:.1f} Motor:{int(motor_angle)}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Visualize Trajectory Line
                if len(path) > 1:
                    pixel_points = []
                    for pt in path:
                        wX, wY = pt[0], pt[1]
                        if wY < 0.1: wY = 0.1 # Prevent div/0
                        
                        # Reverse project meters back into 2D camera pixels
                        px = int((wX / wY) * 337.0 + 320)
                        py = int(480 - (wY / 4.0) * 240) # map 4 meters to bottom half of height
                        pixel_points.append((max(0, min(px, 639)), max(0, min(py, 479))))
                        
                    for i in range(1, len(pixel_points)):
                        cv2.line(frame, pixel_points[i-1], pixel_points[i], (255, 0, 255), 6) # Thick Magenta line

        #show annotated frame in window on screen
        cv2.imshow("Cane Detection", frame)
        #q to break the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

#destroy all processes
pipeline.stop()
cv2.destroyAllWindows()