"""
LeadMe — Map Viewer
Enter a start location and destination in the terminal.
The route is fetched from OSRM and rendered on an OpenStreetMap tile map.

Usage:
    python map.py

Requires:
    pip install staticmap Pillow requests opencv-python numpy
"""

import queue
import threading
import sys

import cv2
import numpy as np
import requests
from staticmap import CircleMarker, Line, StaticMap

# ── Map config ────────────────────────────────────────────────────────────────

MAP_W = 900
MAP_H = 700
OSM_TILES = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
OSRM_URL  = "https://router.project-osrm.org/route/v1/foot/{lon1},{lat1};{lon2},{lat2}?overview=full&geometries=geojson"
NOMINATIM = "https://nominatim.openstreetmap.org/search"


# ── Helpers ───────────────────────────────────────────────────────────────────

def geocode(place: str):
    """Returns (lat, lon) or None."""
    try:
        resp = requests.get(
            NOMINATIM,
            params={"q": place, "format": "json", "limit": 1},
            headers={"User-Agent": "leadme-cane-nav"},
            timeout=10,
        )
        results = resp.json()
        if not results:
            return None
        return float(results[0]["lat"]), float(results[0]["lon"])
    except Exception as e:
        print(f"[map] geocode error: {e}")
        return None


def get_route(start, end):
    """Returns list of (lat, lon) waypoints or None."""
    try:
        url = OSRM_URL.format(
            lon1=start[1], lat1=start[0],
            lon2=end[1],   lat2=end[0],
        )
        resp = requests.get(url, timeout=15)
        data = resp.json()
        if data.get("code") != "Ok":
            print(f"[map] OSRM error: {data.get('message', data.get('code'))}")
            return None
        coords = data["routes"][0]["geometry"]["coordinates"]  # [[lon, lat], ...]
        return [(c[1], c[0]) for c in coords]                  # → (lat, lon)
    except Exception as e:
        print(f"[map] route error: {e}")
        return None


def render_map(route, start, end) -> np.ndarray:
    """Render route to a BGR numpy image."""
    m = StaticMap(MAP_W, MAP_H, url_template=OSM_TILES)

    # Route line
    line_coords = [(lon, lat) for lat, lon in route]
    m.add_line(Line(line_coords, "#3388ff", 5))

    # Start marker (green) and end marker (red)
    m.add_marker(CircleMarker((start[1], start[0]), "#00cc44", 14))
    m.add_marker(CircleMarker((end[1],   end[0]),   "#ff3333", 14))

    img_pil = m.render()
    img_rgb = np.array(img_pil)
    return cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)


# ── Input thread ──────────────────────────────────────────────────────────────

_request_q: queue.Queue = queue.Queue()


def _input_loop():
    print("\n=== LeadMe Map Viewer ===")
    print("Type start and destination to show a route.")
    print("Press Q in the map window to quit.\n")
    while True:
        try:
            start_str = input("Start location : ").strip()
            if not start_str:
                continue
            dest_str = input("Destination    : ").strip()
            if not dest_str:
                continue
            _request_q.put((start_str, dest_str))
        except EOFError:
            break


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    threading.Thread(target=_input_loop, daemon=True).start()

    placeholder = np.zeros((MAP_H, MAP_W, 3), dtype=np.uint8)
    cv2.putText(placeholder, "Enter start + destination in terminal",
                (MAP_W // 2 - 260, MAP_H // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (180, 180, 180), 2)
    cv2.imshow("LeadMe Map", placeholder)

    current_img = placeholder

    while True:
        # Check for a new route request
        try:
            start_str, dest_str = _request_q.get_nowait()

            print(f"\n[map] geocoding '{start_str}' …")
            start = geocode(start_str)
            if not start:
                print(f"[map] could not find: {start_str}")
                continue

            print(f"[map] geocoding '{dest_str}' …")
            end = geocode(dest_str)
            if not end:
                print(f"[map] could not find: {dest_str}")
                continue

            print(f"[map] start → {start[0]:.5f}, {start[1]:.5f}")
            print(f"[map] end   → {end[0]:.5f}, {end[1]:.5f}")
            print("[map] fetching route …")

            route = get_route(start, end)
            if not route:
                print("[map] no route found")
                continue

            print("[map] rendering …")
            current_img = render_map(route, start, end)

            # Overlay distance label
            dist_km = requests.get(
                OSRM_URL.format(
                    lon1=start[1], lat1=start[0],
                    lon2=end[1],   lat2=end[0],
                ),
                timeout=10,
            ).json()["routes"][0]["distance"] / 1000
            cv2.putText(current_img,
                        f"{start_str}  ->  {dest_str}   ({dist_km:.1f} km)",
                        (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
            cv2.putText(current_img,
                        f"{start_str}  ->  {dest_str}   ({dist_km:.1f} km)",
                        (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (30, 30, 30), 1)

            cv2.imshow("LeadMe Map", current_img)
            print("[map] done — enter new locations or press Q in the window to quit\n")

        except queue.Empty:
            pass

        key = cv2.waitKey(50) & 0xFF
        if key == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
