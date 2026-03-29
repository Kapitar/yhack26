import pyrealsense2 as rs
import cv2
import numpy as np
from ultralytics import YOLO

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
    
    #loop through all obkects yolo detected
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
        
        #green box around object
        color = (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        #box label, confidence, distance above the box
        cv2.putText(frame, f"{label} {conf:.2f} | {distance:.2f}m", 
                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    #show annotated frame in window on screen
    cv2.imshow("Cane Detection", frame)
    #q to break the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#destroy all processes
pipeline.stop()
cv2.destroyAllWindows()