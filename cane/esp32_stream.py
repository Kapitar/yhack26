"""
ESP32-S3 Camera Stream Viewer
Connects to the onboard ESP32-S3 OV2640 MJPEG stream and displays it in an OpenCV window.
Press 'q' to quit.
"""

import cv2
import numpy as np
import urllib.request

# The ESP32 camera web server serves MJPEG on port 81 at /stream
STREAM_URL = "http://192.168.5.1:81/stream"

def main():
    print(f"Connecting to ESP32-S3 stream at {STREAM_URL} ...")
    
    # Manual MJPEG parsing — the ESP32 CameraWebServer sends multipart/x-mixed-replace
    stream = urllib.request.urlopen(STREAM_URL, timeout=10)
    print("Connected! Streaming...")
    
    raw_bytes = b''
    
    while True:
        raw_bytes += stream.read(4096)
        
        # MJPEG frames are bounded by FFD8 (JPEG start) and FFD9 (JPEG end)
        start = raw_bytes.find(b'\xff\xd8')
        end = raw_bytes.find(b'\xff\xd9')
        
        if start != -1 and end != -1 and end > start:
            jpg = raw_bytes[start:end + 2]
            raw_bytes = raw_bytes[end + 2:]
            
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow("ESP32-S3 Camera", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    stream.close()
    cv2.destroyAllWindows()
    print("Stream closed.")

if __name__ == "__main__":
    main()
