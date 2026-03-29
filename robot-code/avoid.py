"""
LeadMe — Obstacle Avoidance (2-wheel differential drive)
Runs YOLO object detection on the RealSense camera.
When an obstacle is in the forward path, the robot arcs around it
(forward + rotation) so overall forward progress is maintained.

Visualization:
  Window 1 — camera feed with YOLO boxes + A* path overlay (smooth bezier curves)
  Window 2 — 3D matplotlib view: detected objects + 3D path curve

Motor layout (front two wheels only):
    [0 LEFT] ─── [1 RIGHT]

Usage:
    python avoid.py                      # auto-detect Arduino port
    python avoid.py --port COM3
    python avoid.py --port /dev/ttyACM0
    python avoid.py --no-display         # headless

Requires:
    pip install pyrealsense2 ultralytics opencv-python pyserial matplotlib numpy
"""

import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

import argparse
import io
import tempfile
import threading
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import requests
import serial
import serial.tools.list_ports
from ultralytics import YOLO
from staticmap import CircleMarker, Line, StaticMap

import matplotlib
matplotlib.use("Agg")   # off-screen renderer — no Tk window, no GIL conflict
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401 (registers 3d projection)
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ── Tuning ────────────────────────────────────────────────────────────────────

STOP_DISTANCE_M  = 2.0
DRIVE_SPEED      = 70
ARC_SPEED        = 80
ARC_ROT          = 70
BACK_SPEED       = 45
BACK_DURATION_S  = 0.5

LEFT_END    = 0.25
RIGHT_START = 0.75

CONF_THRESH      = 0.45
OBSTACLE_CLASSES = None

BAUD = 9600

# ── Voice warning (ElevenLabs via lava.so) ────────────────────────────────────

LAVA_API_KEY     = "aks_live_Xj5cToeOUuuLka5pePg_AFLp2Hsprn_C_mutwlSMEyve9M0i_K51P-n"
ELEVENLABS_VOICE = "21m00Tcm4TlvDq8ikWAM"   # Rachel — change to any ElevenLabs voice ID
WARN_INTERVAL_S = 1.5   # minimum seconds between successive "stop" warnings

# Approximate RealSense focal length (pixels) at 640×480
FOCAL_PX = 600.0

# How many frames between 3-D plot refreshes (matplotlib is slow)
PLOT_3D_EVERY = 6

# ── Map config ────────────────────────────────────────────────────────────────
MAP_W      = 900
MAP_H      = 700
OSM_TILES  = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
NOMINATIM  = "https://nominatim.openstreetmap.org/search"
OSRM_BASE  = "https://router.project-osrm.org/route/v1/foot/{lon1},{lat1};{lon2},{lat2}?overview=full&geometries=geojson"


# ── Speech warning ───────────────────────────────────────────────────────────

class SpeechWarner:
    """
    Calls lava.so ElevenLabs TTS in a background thread.
    Rate-limited per instance so obstacle and sidewalk warnings don't collide.
    """

    def __init__(self, message: str, interval_s: float = WARN_INTERVAL_S):
        self._message     = message
        self._interval_s  = interval_s
        self._last_warned = 0.0
        self._lock        = threading.Lock()
        self._playing     = False

    def warn(self):
        now = time.monotonic()
        with self._lock:
            if self._playing:
                return
            if now - self._last_warned < self._interval_s:
                return
            self._last_warned = now
            self._playing     = True

        threading.Thread(target=self._speak, daemon=True).start()

    def _speak(self):
        try:
            import urllib.parse

            provider_url = (
                f"https://api.elevenlabs.io/v1/text-to-speech/{ELEVENLABS_VOICE}"
            )
            forward_url = (
                "https://api.lava.so/v1/forward?u="
                + urllib.parse.quote(provider_url, safe="")
            )

            resp = requests.post(
                forward_url,
                headers={
                    "Authorization": f"Bearer {LAVA_API_KEY}",
                    "Content-Type":  "application/json",
                    "Accept":        "audio/mpeg",
                },
                json={
                    "text":     self._message,
                    "model_id": "eleven_flash_v2",
                    "voice_settings": {"stability": 0.4, "similarity_boost": 0.8},
                },
                timeout=8,
            )
            resp.raise_for_status()

            # Write mp3 to a temp file and play with pygame
            with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as f:
                f.write(resp.content)
                tmp_path = f.name

            import pygame
            pygame.mixer.init()
            pygame.mixer.music.load(tmp_path)
            pygame.mixer.music.play()
            while pygame.mixer.music.get_busy():
                time.sleep(0.05)
            pygame.mixer.music.unload()

            import os
            os.unlink(tmp_path)

        except requests.HTTPError as e:
            print(f"[speech] warning: {e} — {e.response.text}")
        except Exception as e:
            print(f"[speech] warning: {e}")
        finally:
            with self._lock:
                self._playing = False


# ── Map viewer ───────────────────────────────────────────────────────────────

class MapViewer:
    """
    Background thread: reads destination from stdin, fetches route from OSRM,
    renders onto OSM tiles. Main thread calls get_frame() and shows with cv2.
    Current location is auto-detected via IP geolocation.
    """

    def __init__(self):
        self._lock  = threading.Lock()
        self._frame = None          # BGR numpy image, set after each render
        threading.Thread(target=self._run, daemon=True).start()

    def get_frame(self):
        with self._lock:
            return self._frame

    # ── helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _get_current_location():
        try:
            d = requests.get("http://ip-api.com/json/", timeout=5).json()
            if d.get("status") == "success":
                return d["lat"], d["lon"], d.get("city", "")
        except Exception:
            pass
        return None

    @staticmethod
    def _geocode(place: str):
        try:
            results = requests.get(
                NOMINATIM,
                params={"q": place, "format": "json", "limit": 1},
                headers={"User-Agent": "leadme-cane"},
                timeout=10,
            ).json()
            if results:
                return float(results[0]["lat"]), float(results[0]["lon"])
        except Exception as e:
            print(f"[map] geocode error: {e}")
        return None

    @staticmethod
    def _get_route(start, end):
        try:
            url = OSRM_BASE.format(
                lon1=start[1], lat1=start[0],
                lon2=end[1],   lat2=end[0],
            )
            data = requests.get(url, timeout=15).json()
            if data.get("code") != "Ok":
                print(f"[map] OSRM: {data.get('message', data.get('code'))}")
                return None, None
            route = data["routes"][0]
            coords = [(c[1], c[0]) for c in route["geometry"]["coordinates"]]
            dist_km = route["distance"] / 1000
            return coords, dist_km
        except Exception as e:
            print(f"[map] route error: {e}")
            return None, None

    def _render(self, route, start, end, dest_name, dist_km):
        m = StaticMap(MAP_W, MAP_H, url_template=OSM_TILES)
        m.add_line(Line([(lon, lat) for lat, lon in route], "#3388ff", 5))
        m.add_marker(CircleMarker((start[1], start[0]), "#00cc44", 14))
        m.add_marker(CircleMarker((end[1],   end[0]),   "#ff3333", 14))
        img = cv2.cvtColor(np.array(m.render()), cv2.COLOR_RGB2BGR)
        cv2.putText(img, f"To: {dest_name}  ({dist_km:.1f} km)",
                    (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0),   4)
        cv2.putText(img, f"To: {dest_name}  ({dist_km:.1f} km)",
                    (12, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        with self._lock:
            self._frame = img

    # ── main thread ───────────────────────────────────────────────────────────

    @staticmethod
    def _transcribe(audio_np, samplerate: int) -> str | None:
        """Send audio to ElevenLabs scribe_v1 via lava.so and return transcript."""
        import io as _io
        import urllib.parse
        import soundfile as sf

        wav_buf = _io.BytesIO()
        sf.write(wav_buf, audio_np, samplerate, format="WAV", subtype="PCM_16")
        wav_buf.seek(0)

        provider_url = "https://api.elevenlabs.io/v1/speech-to-text"
        forward_url  = "https://api.lava.so/v1/forward?u=" + urllib.parse.quote(provider_url, safe="")

        try:
            resp = requests.post(
                forward_url,
                headers={"Authorization": f"Bearer {LAVA_API_KEY}"},
                files={"file": ("audio.wav", wav_buf, "audio/wav")},
                data={"model_id": "scribe_v1"},
                timeout=20,
            )
            resp.raise_for_status()
            return resp.json().get("text", "").strip()
        except requests.HTTPError as e:
            print(f"[map] STT error: {e} — {e.response.text}")
        except Exception as e:
            print(f"[map] STT error: {e}")
        return None

    def _run(self):
        import sounddevice as sd

        print("\n[map] detecting current location via IP …")
        loc_result = self._get_current_location()
        if loc_result:
            current_loc = (loc_result[0], loc_result[1])
            print(f"[map] current location: {loc_result[2]} "
                  f"({current_loc[0]:.4f}, {current_loc[1]:.4f})")
        else:
            print("[map] could not auto-detect location — map disabled")
            return

        samplerate = 16000

        print("[map] ready — press Enter to speak a destination\n")
        while True:
            try:
                input("[map] Press Enter to speak…")
            except EOFError:
                break

            print("[map] Listening… press Enter when done speaking")
            chunks = []

            def _cb(indata, frames, t, status):
                chunks.append(indata.copy())

            stream = sd.InputStream(samplerate=samplerate, channels=1,
                                    dtype="int16", callback=_cb)
            stream.start()
            try:
                input()
            except EOFError:
                pass
            stream.stop()
            stream.close()

            if not chunks:
                continue

            audio_np = np.concatenate(chunks, axis=0).flatten()

            print("[map] transcribing…")
            dest = self._transcribe(audio_np, samplerate)
            if not dest:
                print("[map] could not understand — try again")
                continue
            print(f"[map] heard: \"{dest}\"")

            print(f"[map] geocoding '{dest}' …")
            end = self._geocode(dest)
            if not end:
                print(f"[map] could not find: {dest}")
                continue

            print("[map] fetching route …")
            route, dist_km = self._get_route(current_loc, end)
            if not route:
                continue

            print("[map] rendering …")
            self._render(route, current_loc, end, dest, dist_km)
            print("[map] map updated\n")


# ── Arduino serial ────────────────────────────────────────────────────────────

def find_arduino() -> str | None:
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        mfr  = (p.manufacturer or "").lower()
        if any(k in desc or k in mfr for k in
               ("arduino", "ch340", "ch341", "ftdi", "usb serial")):
            return p.device
    for p in serial.tools.list_ports.comports():
        if any(k in p.device for k in
               ("ttyUSB", "ttyACM", "cu.usbmodem", "cu.usbserial")):
            return p.device
    return None


class Robot:
    def __init__(self, port: str | None):
        port = port or find_arduino()
        if port is None:
            raise RuntimeError("Arduino not found. Connect USB or pass --port.")
        self._ser  = serial.Serial(port=port, baudrate=BAUD, timeout=1)
        time.sleep(2.0)
        self._ser.reset_input_buffer()
        self._lock = threading.Lock()
        print(f"[robot] connected on {port}")

    def _set(self, angle: int, velocity: int, rot: int):
        cmd = f"{angle % 360}|{max(0, min(100, velocity))}|{max(-100, min(100, rot))}\n"
        with self._lock:
            self._ser.write(cmd.encode("ascii"))

    def forward(self):   self._set(0,   DRIVE_SPEED,  0)
    def arc_left(self):  self._set(0,   ARC_SPEED,  -ARC_ROT)
    def arc_right(self): self._set(0,   ARC_SPEED,   ARC_ROT)
    def backward(self):  self._set(180, BACK_SPEED,   0)

    def stop(self):
        with self._lock:
            self._ser.write(b"S\n")

    def close(self):
        self.stop()
        self._ser.close()


# ── Depth zone helper ─────────────────────────────────────────────────────────

def zone_distance(depth_frame, x0r: float, x1r: float,
                  y0r: float = 0.25, y1r: float = 0.75) -> float:
    w = depth_frame.get_width()
    h = depth_frame.get_height()
    xs = np.clip(np.linspace(int(w * x0r), int(w * x1r), 7, dtype=int), 0, w - 1)
    ys = np.clip(np.linspace(int(h * y0r), int(h * y1r), 7, dtype=int), 0, h - 1)
    dists = [
        depth_frame.get_distance(int(x), int(y))
        for y in ys for x in xs
        if 0.6 <= depth_frame.get_distance(int(x), int(y)) <= 8.0
    ]
    if not dists:
        return float("inf")
    dists.sort()
    return dists[max(0, int(len(dists) * 0.10))]


# ── Bezier / path helpers ─────────────────────────────────────────────────────

def cubic_bezier_2d(p0, p1, p2, p3, n: int = 80) -> np.ndarray:
    """Return (n, 2) array of points along a cubic bezier curve."""
    t  = np.linspace(0, 1, n)[:, None]
    mt = 1 - t
    return (mt**3 * p0 + 3 * mt**2 * t * p1
            + 3 * mt * t**2 * p2 + t**3 * p3)


def cubic_bezier_3d(p0, p1, p2, p3, n: int = 80) -> np.ndarray:
    """Return (n, 3) array of points along a cubic bezier curve."""
    t  = np.linspace(0, 1, n)[:, None]
    mt = 1 - t
    return (mt**3 * p0 + 3 * mt**2 * t * p1
            + 3 * mt * t**2 * p2 + t**3 * p3)


def draw_curve(img, pts, color, thickness=2, glow=True):
    """Draw a smooth polyline; optionally add a soft glow underneath."""
    pts_i = pts.astype(np.int32)
    if glow:
        cv2.polylines(img, [pts_i], False, color, thickness + 6, cv2.LINE_AA)
        overlay = img.copy()
        cv2.polylines(overlay, [pts_i], False, color, thickness + 6, cv2.LINE_AA)
        cv2.addWeighted(overlay, 0.25, img, 0.75, 0, img)
    cv2.polylines(img, [pts_i], False, color, thickness, cv2.LINE_AA)


def draw_path_overlay(frame, action: str, obstacle_boxes: list):
    """
    Draw an A*-style planned path overlay on the frame.

    obstacle_boxes : list of (x1,y1,x2,y2) for obstacles in the center zone.
    action         : current robot action string.
    """
    fh, fw = frame.shape[:2]

    # Robot position at bottom-centre, goal at top-centre
    start = np.array([[fw / 2, fh - 10]], dtype=float)
    goal  = np.array([[fw / 2, 10]],      dtype=float)

    if action == "BACK UP":
        # Dashed downward arrow
        back_goal = np.array([[fw / 2, fh - 1]], dtype=float)
        p1 = start + [0,  20]
        p2 = back_goal + [0, -20]
        pts = cubic_bezier_2d(start, p1, p2, back_goal)
        draw_curve(frame, pts, (0, 60, 220), thickness=3, glow=True)
        # Arrow head
        cv2.arrowedLine(frame,
                        (int(fw / 2), fh - 20), (int(fw / 2), fh - 5),
                        (0, 60, 220), 3, tipLength=0.5)
        return

    if "ARC LEFT" in action or (not obstacle_boxes and action == "FORWARD"):
        side_shift = -fw * 0.22 if "ARC LEFT" in action else 0.0
    else:
        side_shift = fw * 0.22 if "ARC RIGHT" in action else 0.0

    if obstacle_boxes and action != "FORWARD":
        # Use the nearest obstacle's bounding box to compute control points
        # that pass cleanly to the side of it
        ox1, oy1, ox2, oy2 = obstacle_boxes[0]
        ocx = (ox1 + ox2) / 2.0
        ocy = (oy1 + oy2) / 2.0

        if "ARC LEFT" in action:
            ctrl_x = ox1 - fw * 0.12          # pass to the left of the box
        else:
            ctrl_x = ox2 + fw * 0.12          # pass to the right

        p1 = np.array([[fw / 2, fh * 0.7]])
        p2 = np.array([[ctrl_x, ocy]])
        p3 = goal + [side_shift, 0]
        pts = cubic_bezier_2d(start, p1, p2, p3)
    else:
        # Straight or gentle forward path
        p1 = start + [side_shift * 0.3, -(fh * 0.3)]
        p2 = goal   + [side_shift * 0.7,  (fh * 0.3)]
        pts = cubic_bezier_2d(start, p1, p2, goal)

    path_color = (0, 220, 100) if action == "FORWARD" else (0, 160, 255)
    draw_curve(frame, pts, path_color, thickness=3, glow=True)

    # Draw animated chevrons along the path (every ~12th point)
    for i in range(4, len(pts) - 4, 12):
        p      = pts[i].astype(int)
        ahead  = pts[min(i + 4, len(pts) - 1)].astype(int)
        behind = pts[max(i - 4, 0)].astype(int)
        angle  = np.degrees(np.arctan2(ahead[1] - behind[1],
                                       ahead[0] - behind[0]))
        cv2.circle(frame, tuple(p), 3, path_color, -1, cv2.LINE_AA)

    # Goal marker
    goal_i = goal.astype(int)[0]
    cv2.circle(frame, tuple(goal_i), 8,  path_color, -1,  cv2.LINE_AA)
    cv2.circle(frame, tuple(goal_i), 12, path_color,  2,  cv2.LINE_AA)

    # Robot marker at bottom
    start_i = start.astype(int)[0]
    cv2.circle(frame, tuple(start_i), 8, (255, 255, 255), -1, cv2.LINE_AA)
    cv2.circle(frame, tuple(start_i), 12, path_color, 2, cv2.LINE_AA)


# ── 3-D coordinate helpers ────────────────────────────────────────────────────

def pixel_to_3d(cx: float, cy: float, depth: float,
                fw: int = 640, fh: int = 480) -> tuple:
    """Back-project a pixel + depth to (X, Y, Z) in metres, robot-centred."""
    x =  (cx - fw / 2) * depth / FOCAL_PX
    y = -(cy - fh / 2) * depth / FOCAL_PX   # flip: up is +Y
    z =  depth
    return x, y, z


def box_faces(cx, cy, cz, hw, hh, hd):
    """Return 6 face vertex lists for a cuboid centred at (cx,cy,cz)."""
    x0, x1 = cx - hw, cx + hw
    y0, y1 = cy - hh, cy + hh
    z0, z1 = cz - hd, cz + hd
    return [
        [[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0]],  # front
        [[x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]],  # back
        [[x0,y0,z0],[x0,y0,z1],[x0,y1,z1],[x0,y1,z0]],  # left
        [[x1,y0,z0],[x1,y0,z1],[x1,y1,z1],[x1,y1,z0]],  # right
        [[x0,y1,z0],[x1,y1,z0],[x1,y1,z1],[x0,y1,z1]],  # top
        [[x0,y0,z0],[x1,y0,z0],[x1,y0,z1],[x0,y0,z1]],  # bottom
    ]


# ── 3-D plot setup ────────────────────────────────────────────────────────────

def setup_3d_plot():
    fig = plt.figure(figsize=(7, 6), facecolor="#0d0d0d")
    ax  = fig.add_subplot(111, projection="3d")
    ax.set_facecolor("#0d0d0d")
    fig.subplots_adjust(left=0, right=1, bottom=0, top=1)
    return fig, ax


def update_3d_plot(fig, ax, detections_3d: list, action: str):
    """
    detections_3d : list of dicts with keys:
        label, x, y, z, w_m (half-width), h_m (half-height), d_m (half-depth), is_obstacle
    action        : current robot action string
    """
    ax.cla()

    # ── Styling ───────────────────────────────────────────────────────────────
    ax.set_facecolor("#0d0d0d")
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.fill = False
        pane.set_edgecolor("#333333")
    ax.grid(True, color="#222222", linewidth=0.5)
    ax.set_xlabel("X (m)", color="#888888", labelpad=6)
    ax.set_ylabel("Z / depth (m)", color="#888888", labelpad=6)
    ax.set_zlabel("Y (m)", color="#888888", labelpad=6)
    ax.tick_params(colors="#555555")
    for spine in ax.spines.values():
        spine.set_edgecolor("#333333")

    # Fixed axes so the view doesn't jump
    ax.set_xlim(-3,  3)
    ax.set_ylim( 0,  6)
    ax.set_zlim(-2,  2)

    # ── Floor grid ────────────────────────────────────────────────────────────
    gx = np.linspace(-3, 3, 7)
    gz = np.linspace( 0, 6, 7)
    GX, GZ = np.meshgrid(gx, gz)
    GY = np.zeros_like(GX) - 1.0
    ax.plot_surface(GX, GZ, GY, alpha=0.07, color="#00ff88", linewidth=0)

    # ── Robot marker at origin ────────────────────────────────────────────────
    ax.scatter([0], [0], [0], c="#ffffff", s=80, zorder=5)
    ax.quiver(0, 0, 0, 0, 0.6, 0, color="#00ffcc", linewidth=2,
              arrow_length_ratio=0.3)
    ax.text(0, 0, 0.12, "ROBOT", color="#00ffcc", fontsize=7,
            ha="center", va="bottom")

    # ── Obstacle boxes ────────────────────────────────────────────────────────
    for det in detections_3d:
        cx, cy, cz = det["x"], det["y"], det["z"]
        hw, hh, hd = det["w_m"], det["h_m"], det["d_m"]
        obstacle   = det["is_obstacle"]
        face_color = "#ff3333" if obstacle else "#33aaff"
        edge_color = "#ff6666" if obstacle else "#66ccff"

        faces = box_faces(cx, cz, cy, hw, hd, hh)   # swap Y/Z for matplotlib
        poly  = Poly3DCollection(faces, alpha=0.25,
                                 facecolor=face_color, edgecolor=edge_color,
                                 linewidth=0.6)
        ax.add_collection3d(poly)

        ax.text(cx, cz, cy + hh + 0.05, det["label"],
                color="#ffffff", fontsize=7, ha="center", va="bottom")

    # ── 3-D path curve ────────────────────────────────────────────────────────
    if "ARC LEFT" in action:
        side_x = -1.2
    elif "ARC RIGHT" in action:
        side_x =  1.2
    elif "BACK UP" in action:
        side_x =  0.0
    else:
        side_x =  0.0

    goal_z = 5.0
    if "BACK UP" in action:
        p0 = np.array([[ 0.0,  0.0, 0.0]])
        p1 = np.array([[ 0.0, -0.8, 0.0]])
        p2 = np.array([[ 0.0, -1.5, 0.0]])
        p3 = np.array([[ 0.0, -2.0, 0.0]])
        path_color = "#ff3333"
    else:
        p0 = np.array([[ 0.0,   0.0, 0.0]])
        p1 = np.array([[ side_x * 0.3, goal_z * 0.3, 0.0]])
        p2 = np.array([[ side_x,       goal_z * 0.7, 0.0]])
        p3 = np.array([[ side_x * 0.2, goal_z,       0.0]])
        path_color = "#00ff88" if action == "FORWARD" else "#ffaa00"

    curve = cubic_bezier_3d(p0, p1, p2, p3, n=100)
    # curve columns: [x, z_world, y_world]
    ax.plot(curve[:, 0], curve[:, 1], curve[:, 2],
            color=path_color, linewidth=2.5, zorder=10)

    # Chevron markers along path
    for i in range(5, len(curve) - 5, 15):
        ax.scatter(curve[i, 0], curve[i, 1], curve[i, 2],
                   c=path_color, s=20, zorder=11)

    # Goal marker
    ax.scatter([p3[0, 0]], [p3[0, 1]], [p3[0, 2]],
               c=path_color, s=80, marker="*", zorder=12)

    action_colors = {"FORWARD": "#00ff88", "BACK UP": "#ff3333"}
    color = action_colors.get(
        next((k for k in action_colors if k in action), ""), "#ffaa00"
    )
    ax.set_title(f"  {action}", color=color, fontsize=9,
                 loc="left", pad=4, fontweight="bold")

    fig.canvas.draw()
    w, h = fig.canvas.get_width_height()
    img = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8).reshape(h, w, 4)
    cv2.imshow("LeadMe 3D", cv2.cvtColor(img, cv2.COLOR_RGBA2BGR))


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",       default=None)
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--model",      default="yolov8n.pt")
    args = parser.parse_args()

    model      = YOLO(args.model)
    robot      = Robot(args.port)
    warner     = SpeechWarner("Stop!")
    map_viewer = MapViewer()

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  15)
    pipeline.start(cfg)
    align = rs.align(rs.stream.color)

    if not args.no_display:
        fig3d, ax3d = setup_3d_plot()

    print("[avoid] running — press Q to quit")

    backing_until = 0.0
    frame_count   = 0

    try:
        while True:
            frames  = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_f = aligned.get_color_frame()
            depth_f = aligned.get_depth_frame()
            if not color_f or not depth_f:
                continue

            frame = np.asanyarray(color_f.get_data())
            fh, fw = frame.shape[:2]
            frame_count += 1

            # ── YOLO detection ────────────────────────────────────────────────
            results        = model(frame, verbose=False)
            obstacle_ahead = False
            center_boxes   = []    # (x1,y1,x2,y2) of obstacles in center zone
            detections_3d  = []    # dicts for 3-D plot

            for box in results[0].boxes:
                conf = float(box.conf[0])
                if conf < CONF_THRESH:
                    continue
                label = model.names[int(box.cls[0])]
                if OBSTACLE_CLASSES and label not in OBSTACLE_CLASSES:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy   = (x1 + x2) // 2, (y1 + y2) // 2
                cx_ratio = cx / fw
                distance = depth_f.get_distance(
                    np.clip(cx, 0, fw - 1), np.clip(cy, 0, fh - 1)
                )
                in_center = LEFT_END <= cx_ratio <= RIGHT_START
                is_close  = 0.6 < distance < STOP_DISTANCE_M

                box_color = (0, 0, 255) if (in_center and is_close) else (0, 200, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
                cv2.putText(frame,
                            f"{label} {conf:.2f} {distance:.2f}m",
                            (x1, max(y1 - 8, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2)

                if in_center and is_close:
                    obstacle_ahead = True
                    center_boxes.append((x1, y1, x2, y2))

                # 3-D position
                if 0.6 < distance < 8.0:
                    ox, oy, oz = pixel_to_3d(cx, cy, distance, fw, fh)
                    box_w_m = ((x2 - x1) / fw)  * distance
                    box_h_m = ((y2 - y1) / fh)  * distance
                    detections_3d.append({
                        "label":       label,
                        "x": ox, "y": oy, "z": oz,
                        "w_m": box_w_m / 2,
                        "h_m": box_h_m / 2,
                        "d_m": 0.15,
                        "is_obstacle": in_center and is_close,
                    })

            # ── Avoidance decision ────────────────────────────────────────────
            now    = time.monotonic()
            action = "FORWARD"

            if now < backing_until:
                robot.backward()
                action = "BACK UP"
            elif obstacle_ahead:
                warner.warn()
                left_dist  = zone_distance(depth_f, 0.0,        LEFT_END)
                right_dist = zone_distance(depth_f, RIGHT_START, 1.0)
                if left_dist >= right_dist:
                    robot.arc_left()
                    action = f"ARC LEFT  (L:{left_dist:.1f}m R:{right_dist:.1f}m)"
                elif right_dist > left_dist:
                    robot.arc_right()
                    action = f"ARC RIGHT (L:{left_dist:.1f}m R:{right_dist:.1f}m)"
                else:
                    robot.backward()
                    backing_until = now + BACK_DURATION_S
                    action = "BACK UP"
            else:
                robot.forward()

            # ── 2-D display ───────────────────────────────────────────────────
            if not args.no_display:
                # Zone lines
                cv2.line(frame, (int(fw * LEFT_END),    0),
                                (int(fw * LEFT_END),    fh), (180, 180, 0), 1)
                cv2.line(frame, (int(fw * RIGHT_START), 0),
                                (int(fw * RIGHT_START), fh), (180, 180, 0), 1)

                # A* path overlay
                draw_path_overlay(frame, action, center_boxes)

                # Action label
                txt_color = (0, 165, 255) if "ARC"  in action else \
                            (0,   0, 255) if "BACK" in action else \
                            (0, 220,  80)
                cv2.putText(frame, action, (10, fh - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, txt_color, 2)

                cv2.imshow("LeadMe Avoid", frame)

                map_frame = map_viewer.get_frame()
                if map_frame is not None:
                    cv2.imshow("LeadMe Map", map_frame)

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

                # ── 3-D plot (throttled) ──────────────────────────────────────
                if frame_count % PLOT_3D_EVERY == 0:
                    update_3d_plot(fig3d, ax3d, detections_3d, action)

    except KeyboardInterrupt:
        pass
    finally:
        robot.close()
        pipeline.stop()
        cv2.destroyAllWindows()
        plt.close("all")
        print("[avoid] stopped")


if __name__ == "__main__":
    main()
