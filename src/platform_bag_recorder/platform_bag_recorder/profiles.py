"""Which topics to record on each platform.

Each entry is a *slot* - a role like "2d scan" or "camera" - holding one or
more candidate topics. The first candidate that is actually being published
wins, which is how "compressed version if published" and "3D scan if
applicable" are handled without the caller having to know the sensor fit.

Slots marked required=False are skipped entirely when nothing publishes them,
so the same profile covers an a200 carrying a camera and 3D lidar and one
carrying only the 2D lidar - a real difference between the sensor fits in use
here, not a hypothetical one.
"""


class Slot:
    """One recorded role.

    required   the recorder waits for this before it starts recording.
    probe      the chosen candidate must actually deliver a message, not just
               be advertised. True for sensors, where an advertised-but-dead
               transport is a real hazard (the go1 advertises a compressed
               camera transport that emits nothing). False for command topics
               like cmd_vel, which are legitimately silent while the robot is
               idle - those are selected on advertisement so they appear in the
               bag, correctly typed and empty, rather than being dropped.
    """

    def __init__(self, name, candidates, required=True, probe=True):
        self.name = name
        self.candidates = candidates
        self.required = required
        self.probe = probe


def a200(ns='a200_0000'):
    """Clearpath a200 (Husky). Sensor topics live under <ns>/sensors/."""
    return [
        Slot('scan_2d', [f'/{ns}/sensors/lidar2d_0/scan']),
        Slot('scan_3d', [f'/{ns}/sensors/lidar3d_0/scan'], required=False),
        Slot('points_3d', [f'/{ns}/sensors/lidar3d_0/points'], required=False),
        # Prefer the compressed transport; fall back to raw.
        #
        # image_transport hangs its transports off the *base* topic name, so
        # for a base of "color/image" the compressed topic is "color/compressed"
        # - a sibling, not a child. Guessing "color/image/compressed" finds
        # nothing and quietly falls back to raw, which costs about 30x the bag
        # size for the same pictures.
        Slot('camera', [f'/{ns}/sensors/camera_0/color/compressed',
                        f'/{ns}/sensors/camera_0/color/image'], required=False),
        # No IMU is published by the a200 in simulation on either config in
        # use here; the slot stays so a robot that does have one is recorded.
        Slot('imu', [f'/{ns}/sensors/imu_0/data',
                     f'/{ns}/sensors/imu_0/data_raw',
                     f'/{ns}/platform/sensors/imu_0/data'], required=False),
        # platform/odom is the wheel odometry; odom/filtered is the EKF output.
        Slot('odom', [f'/{ns}/platform/odom/filtered',
                      f'/{ns}/platform/odom']),
        Slot('cmd_vel', [f'/{ns}/cmd_vel'], required=False, probe=False),
        Slot('tf', [f'/{ns}/tf']),
        Slot('tf_static', [f'/{ns}/tf_static'], required=False),
    ]


def go1(ns='robot1'):
    """Unitree go1. Flat namespace, and no 3D lidar on this model."""
    return [
        Slot('scan_2d', [f'/{ns}/scan']),
        Slot('camera', [f'/{ns}/color/image_raw/compressed',
                        f'/{ns}/color/image_raw']),
        Slot('imu', [f'/{ns}/imu_plugin/out', f'/{ns}/imu'], required=False),
        Slot('odom', [f'/{ns}/odometry/filtered', f'/{ns}/odom']),
        Slot('cmd_vel', [f'/{ns}/cmd_vel'], required=False, probe=False),
        Slot('tf', [f'/{ns}/tf']),
        Slot('tf_static', [f'/{ns}/tf_static'], required=False),
    ]


PROFILES = {'a200': a200, 'go1': go1}


def build(platform, namespace=''):
    if platform not in PROFILES:
        raise ValueError(f'unknown platform {platform!r}; '
                         f'expected one of {sorted(PROFILES)}')
    factory = PROFILES[platform]
    return factory(namespace) if namespace else factory()
