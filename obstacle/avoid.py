"""
Maps zone clearance → motor command string for a 2-wheel differential drive robot.

Only the front-left (m0) and front-right (m1) motors are populated.
Strafing doesn't work on differential drive — use forward + rotation instead:
  - Turn left:  low rot > 0  (left wheel slower, right faster → curves left)
  - Turn right: low rot < 0  (right wheel slower, left faster → curves right)

Protocol: angle|velocity|rot  (miniauto_firmware)
  angle   : always 0 (forward) — strafing unavailable on 2-wheel drive
  velocity: 0–100
  rot     : -100..100 (positive=CCW/left, negative=CW/right)
"""

from .detector import ZoneResult

FORWARD_SPEED = 60
TURN_SPEED    = 45   # slow down while turning so the user can feel the curve
TURN_ROT      = 55   # rotation strength (higher = tighter turn)


def zone_to_cmd(zones: ZoneResult) -> str:
    """
    Decision table (differential drive):
      center clear                → go straight forward
      center blocked, left clear  → curve left  (forward + CCW rotation)
      center blocked, right clear → curve right (forward + CW rotation)
      center blocked, both clear  → default curve right
      all blocked                 → stop
    """
    if zones.center_clear:
        return f"0|{FORWARD_SPEED}|0"

    if zones.left_clear and not zones.right_clear:
        return f"0|{TURN_SPEED}|{TURN_ROT}"     # curve left

    if zones.right_clear and not zones.left_clear:
        return f"0|{TURN_SPEED}|-{TURN_ROT}"    # curve right

    if zones.left_clear and zones.right_clear:
        return f"0|{TURN_SPEED}|-{TURN_ROT}"    # both open → default right

    return "S"   # all zones blocked → stop
