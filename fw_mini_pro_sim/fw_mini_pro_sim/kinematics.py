"""Planar rigid-body wheel kinematics; dimensions from the vendor manual."""
import math

WHEEL_NAMES = ('front_left', 'front_right', 'rear_left', 'rear_right')
WHEEL_POSITIONS = ((0.18, 0.14), (0.18, -0.14), (-0.18, 0.14), (-0.18, -0.14))
WHEEL_RADIUS = 0.07
MAX_WHEEL_SPEED = 1.5  # m/s; manual maximum chassis speed, conservative wheel cap


def filter_command(enable, gear, vx, vy, wz, age, timeout=0.5):
    """Return an allowed, uniformly scaled chassis twist or zero on rejection."""
    if (not enable or gear not in (6, 8) or
            not all(math.isfinite(v) for v in (vx, vy, wz, age)) or
            age < 0 or age > timeout):
        return (0.0, 0.0, 0.0)
    vy, wz = (0.0, wz) if gear == 6 else (vy, 0.0)
    peak = max(math.hypot(vx-wz*y, vy+wz*x) for x,y in WHEEL_POSITIONS)
    scale = min(1.0, MAX_WHEEL_SPEED/max(peak, 1e-12), 3.0/max(abs(wz), 1e-12))
    return vx*scale, vy*scale, wz*scale


def wheel_targets(vx, vy, wz, current_angles):
    """Return (continuous steering angle, wheel rad/s), FL/FR/RL/RR.

    Reverse wheel rotation instead of rotating steering by more than pi/2.
    Keep current steering at zero velocity so stop does not scrub the tires.
    """
    targets = []
    for (x,y), current in zip(WHEEL_POSITIONS, current_angles):
        wx, wy = vx-wz*y, vy+wz*x
        speed = math.hypot(wx, wy)
        if speed < 1e-9:
            targets.append((current, 0.0))
            continue
        delta = math.remainder(math.atan2(wy, wx)-current, 2*math.pi)
        if abs(delta) > math.pi/2:
            delta -= math.copysign(math.pi, delta)
            speed = -speed
        targets.append((current+delta, speed/WHEEL_RADIUS))
    return targets
