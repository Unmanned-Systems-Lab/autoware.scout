import math


def command_twist(velocity, steering, wheelbase, max_speed, max_yaw_rate, max_steering):
    if not all(math.isfinite(v) for v in (velocity, steering)):
        return 0.0, 0.0
    velocity = max(-max_speed, min(max_speed, velocity))
    steering = max(-max_steering, min(max_steering, steering))
    yaw_rate = velocity * math.tan(steering) / wheelbase
    return velocity, max(-max_yaw_rate, min(max_yaw_rate, yaw_rate))


def equivalent_steering(velocity, yaw_rate, wheelbase):
    if abs(velocity) < 0.01:
        return 0.0
    return math.atan(wheelbase * yaw_rate / velocity)
