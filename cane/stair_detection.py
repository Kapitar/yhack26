"""
AI Stair Detection - ESP32-S3 Camera or Webcam fallback
Detects stairs using edge detection + Hough Line pattern analysis.
Press 'q' to quit. Press 'd' to toggle debug edge view.
"""

import cv2
import numpy as np
import urllib.request
import time
from collections import deque

# ESP32 stream endpoint
STREAM_URL = "http://192.168.5.1:81/stream"

# Detection parameters (tuned to be sensitive)
MIN_STAIR_LINES = 3
ANGLE_TOLERANCE = 20
MIN_LINE_LENGTH = 50
LINE_GAP = 15
VERTICAL_SPACING_MIN = 10
VERTICAL_SPACING_MAX = 100
CONFIDENCE_FRAMES = 4

detection_history = deque(maxlen=CONFIDENCE_FRAMES)
show_debug = False


def detect_stairs(frame):
    h, w = frame.shape[:2]
    
    # Use lower 70% of frame
    roi_top = int(h * 0.3)
    roi = frame[roi_top:, :]
    
    # Preprocess
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 30, 100)
    
    # Connect horizontal edges
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 1))
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)
    
    # Hough Lines
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=30,
                            minLineLength=MIN_LINE_LENGTH, maxLineGap=LINE_GAP)
    
    horizontal_lines = []
    all_lines = []
    
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = abs(np.degrees(np.arctan2(y2 - y1, x2 - x1)))
            
            all_lines.append({
                'x1': x1, 'y1': y1 + roi_top,
                'x2': x2, 'y2': y2 + roi_top,
                'angle': angle
            })
            
            if angle < ANGLE_TOLERANCE or angle > (180 - ANGLE_TOLERANCE):
                mid_y = (y1 + y2) // 2 + roi_top
                length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                horizontal_lines.append({
                    'y': mid_y,
                    'x1': x1, 'y1': y1 + roi_top,
                    'x2': x2, 'y2': y2 + roi_top,
                    'length': length
                })
    
    horizontal_lines.sort(key=lambda l: l['y'])
    
    # Merge nearby lines into single steps
    stair_lines = []
    if len(horizontal_lines) >= MIN_STAIR_LINES:
        merged = []
        for line in horizontal_lines:
            if not merged or abs(line['y'] - merged[-1]['y']) > 8:
                merged.append(line)
            elif line['length'] > merged[-1]['length']:
                merged[-1] = line
        
        if len(merged) >= MIN_STAIR_LINES:
            spacings = []
            for i in range(1, len(merged)):
                spacings.append(merged[i]['y'] - merged[i-1]['y'])
            
            valid = [s for s in spacings if VERTICAL_SPACING_MIN <= s <= VERTICAL_SPACING_MAX]
            
            if len(valid) >= MIN_STAIR_LINES - 1:
                mean_sp = np.mean(valid)
                std_sp = np.std(valid) if len(valid) > 1 else 0
                
                if mean_sp > 0 and (std_sp / mean_sp) < 0.45:
                    stair_lines = merged
    
    is_detected = len(stair_lines) >= MIN_STAIR_LINES
    confidence = min(1.0, len(stair_lines) / 5.0) if is_detected else 0.0
    
    # Build full-frame edge image for debug view
    full_edges = np.zeros((h, w), dtype=np.uint8)
    full_edges[roi_top:, :] = edges
    
    return is_detected, stair_lines, horizontal_lines, all_lines, confidence, full_edges, roi_top


def try_esp32_stream():
    """Try to connect to ESP32 MJPEG stream."""
    try:
        print(f"Trying ESP32 stream at {STREAM_URL} ...")
        stream = urllib.request.urlopen(STREAM_URL, timeout=3)
        print("ESP32 connected!")
        return stream
    except Exception as e:
        print(f"ESP32 not reachable: {e}")
        return None


def main():
    global show_debug
    
    esp_stream = try_esp32_stream()
    use_webcam = esp_stream is None
    
    if use_webcam:
        print("Falling back to laptop webcam (index 0)...")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("ERROR: Could not open webcam either!")
            return
        print("Webcam opened! Point it at stairs to test.")
    
    raw_bytes = b''
    fps_time = time.time()
    frame_count = 0
    fps = 0
    
    print("Running AI Stair Detection. Press 'q' to quit, 'd' to toggle debug view.")
    
    while True:
        frame = None
        
        if use_webcam:
            ret, frame = cap.read()
            if not ret:
                continue
        else:
            raw_bytes += esp_stream.read(4096)
            start = raw_bytes.find(b'\xff\xd8')
            end = raw_bytes.find(b'\xff\xd9')
            if start != -1 and end != -1 and end > start:
                jpg = raw_bytes[start:end + 2]
                raw_bytes = raw_bytes[end + 2:]
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            
        if frame is None:
            continue
        
        # Run detection
        is_detected, stair_lines, horiz_lines, all_lines, confidence, edges, roi_top = detect_stairs(frame)
        
        detection_history.append(is_detected)
        confirmed = sum(detection_history) >= (CONFIDENCE_FRAMES * 0.5)
        
        # FPS counter
        frame_count += 1
        elapsed = time.time() - fps_time
        if elapsed > 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            fps_time = time.time()
        
        # Annotate frame
        annotated = frame.copy()
        h, w = annotated.shape[:2]
        
        # Draw ROI line
        cv2.line(annotated, (0, roi_top), (w, roi_top), (80, 80, 80), 1)
        cv2.putText(annotated, "Detection Zone", (5, roi_top - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)
        
        # Draw all horizontal lines faintly (green)
        for line in horiz_lines:
            cv2.line(annotated, (line['x1'], line['y1']), (line['x2'], line['y2']),
                    (0, 80, 0), 1)
        
        if confirmed and stair_lines:
            # Draw stair lines boldly (cyan)
            for line in stair_lines:
                cv2.line(annotated, (line['x1'], line['y1']), (line['x2'], line['y2']),
                        (255, 255, 0), 3)
                cv2.circle(annotated, ((line['x1'] + line['x2']) // 2, line['y']), 6, (0, 255, 255), -1)
            
            # Bounding box
            min_y = min(l['y1'] for l in stair_lines) - 15
            max_y = max(l['y2'] for l in stair_lines) + 15
            min_x = min(min(l['x1'], l['x2']) for l in stair_lines) - 15
            max_x = max(max(l['x1'], l['x2']) for l in stair_lines) + 15
            cv2.rectangle(annotated, (max(0, min_x), max(0, min_y)), 
                         (min(w, max_x), min(h, max_y)), (0, 0, 255), 3)
            
            # Warning banner
            overlay = annotated.copy()
            cv2.rectangle(overlay, (0, 0), (w, 50), (0, 0, 180), -1)
            annotated = cv2.addWeighted(overlay, 0.6, annotated, 0.4, 0)
            cv2.putText(annotated, f"WARNING: STAIRS DETECTED ({len(stair_lines)} steps) [{confidence:.0%}]",
                       (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        else:
            status = f"Scanning... ({len(horiz_lines)} lines found)"
            cv2.putText(annotated, status, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Info bar
        source = "ESP32-S3" if not use_webcam else "Webcam"
        cv2.putText(annotated, f"[{source}] FPS: {fps:.1f} | Press 'd' for debug", 
                   (10, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
        
        # Show main window
        cv2.imshow("AI Stair Detection", annotated)
        
        # Show debug edge view if toggled
        if show_debug:
            edge_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            # Draw all detected lines on edge view
            for line in all_lines:
                color = (0, 255, 0) if line['angle'] < ANGLE_TOLERANCE else (0, 0, 255)
                cv2.line(edge_color, (line['x1'], line['y1']), (line['x2'], line['y2']), color, 2)
            cv2.imshow("Debug: Edges + Lines (Green=Horizontal, Red=Other)", edge_color)
        else:
            try:
                cv2.destroyWindow("Debug: Edges + Lines (Green=Horizontal, Red=Other)")
            except cv2.error:
                pass
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('d'):
            show_debug = not show_debug
    
    if use_webcam:
        cap.release()
    else:
        esp_stream.close()
    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()
