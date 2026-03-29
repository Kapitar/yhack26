import pyrealsense2 as rs
import cv2
import numpy as np
import logging
from ultralytics import YOLO

# Import local UART driver for motor control
from uart_driver import MiniAutoDriver

logging.basicConfig(level=logging.INFO)

# Configuration for Avoidance
DANGER_ZONE_X_MIN = 200 # pixels (assuming 640 width)
DANGER_ZONE_X_MAX = 440
CRITICAL_DISTANCE = 0.4 # meters
AVOID_DISTANCE = 1.2    # meters
DEFAULT_SPEED = 60

#you only look once model
#a bigger model seems to be slower, keep using this model but maybe train more sign data onto it
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
# Connect to Arduino driver to control the motors
with MiniAutoDriver() as driver:
    print("Driver connected. Starting detection loop.")
    
    while True:
        #latest frames
        frames = pipeline.wait_for_frames()
        #apply allied pixel for pixel
        aligned = align.process(frames)
        
        #pullout color and depth frames (independently)
        color_frame = aligned.get_color_frame()
        depth_frame = aligned.get_depth_frame()
        #if any frame missing, try again
        if not color_frame or not depth_frame:
            continue

        #color to numpy array (grid of pixel values), work with OpenCV
        frame = np.asanyarray(color_frame.get_data())
        #same w/ depth, each pixel is a value of distance (mm)
        depth_image = np.asanyarray(depth_frame.get_data())

        #yolo runs
        results = model(frame)
        
        closest_distance = float('inf')
        obstacle_cx = -1
        should_stop = False
        
        #loop through all objects yolo detected
        for box in results[0].boxes:
            #coordinates of the bounding box
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            #human readable label
            label = model.names[int(box.cls[0])]
            #confidence score
            conf = float(box.conf[0])
            
            #center point of box, use for depth calculation
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            #calculate actual distance
            distance = depth_frame.get_distance(cx, cy)
            
            # RealSense returns 0.0 when depth sensing fails for a pixel (e.g. reflective surfaces)
            # We should ignore 0.0 distances so the robot doesn't instantly panic/stop.
            if distance <= 0.01:
                continue
                
            # Check if obstacle is in our front-facing safety zone
            in_danger_zone = (DANGER_ZONE_X_MIN < cx < DANGER_ZONE_X_MAX)
            color = (0, 255, 0)
            
            if in_danger_zone:
                if distance < CRITICAL_DISTANCE:
                    should_stop = True
                    color = (0, 0, 255) # Red for critical
                elif distance < AVOID_DISTANCE:
                    color = (0, 165, 255) # Orange for warning
                    if distance < closest_distance:
                        closest_distance = distance
                        obstacle_cx = cx
            
            # Draw box around object (green initially, red/orange if dangerous)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            # Box label, confidence, distance above the box
            cv2.putText(frame, f"{label} {conf:.2f} | {distance:.2f}m", 
                        (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # ======== MOTOR CONTROL LOGIC ========
        
        if should_stop:
            driver.stop()
            cv2.putText(frame, "STATE: STOP", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        
        elif closest_distance < AVOID_DISTANCE:
            # We have an obstacle in the distance we need to avoid.
            # Decide left or right based on which side the obstacle center is leaning towards.
            # If obstacle center is left of the screen center (320), strafe right. Else, strafe left.
            if obstacle_cx < 320:
                driver.strafe_right(speed=DEFAULT_SPEED)
                cv2.putText(frame, "STATE: EVADE RIGHT", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 3)
            else:
                driver.strafe_left(speed=DEFAULT_SPEED)
                cv2.putText(frame, "STATE: EVADE LEFT", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 3)
                
        else:
            # Path is clear, proceed forward safely!
            driver.forward(speed=DEFAULT_SPEED)
            cv2.putText(frame, "STATE: FORWARD", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 3)

        # Draw Danger Zone lines on screen for visual debugging
        cv2.line(frame, (DANGER_ZONE_X_MIN, 0), (DANGER_ZONE_X_MIN, 480), (255, 255, 255), 1)
        cv2.line(frame, (DANGER_ZONE_X_MAX, 0), (DANGER_ZONE_X_MAX, 480), (255, 255, 255), 1)

        #show annotated frame in window on screen
        cv2.imshow("Cane Detection", frame)
        #q to break the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

#destroy all processes
pipeline.stop()
cv2.destroyAllWindows()