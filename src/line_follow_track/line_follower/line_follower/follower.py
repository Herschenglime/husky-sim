"""Camera line follower with lidar obstacle avoidance, for the painted warehouse track.

Two behaviours, kept separate:

* **Follow** - threshold the yellow paint in a band near the bottom of the frame,
  take the centroid of the largest blob, steer proportionally to how far off
  centre it is. No PID, no path fitting, no lookahead: the track is 0.15 m of
  high-contrast yellow on grey concrete and the camera is aimed straight at it.
* **Avoid** - when the 2D lidar sees something standing in the robot's own width
  ahead, stop steering to the line and run a three-phase detour around it, then
  hand back to Follow once the camera has the line again.

Structure worth knowing:
- The image and scan callbacks only *record* what they saw. A fixed-rate timer
  does all the commanding. That way a camera that stops publishing still gets
  handled by the lost-line branch instead of silently freezing the last command.
- Nothing is commanded until the first image arrives, so the robot never drives
  blind on startup.

Why a corridor test rather than an angular cone for detection: the track passes
within 0.87 m of warehouse props in places, so any test that trips on "something
within 1.6 m off the bow" would detour around the scenery for half the lap. What
matters is whether an obstacle stands inside the robot's own swept width, so the
scan is projected into forward/lateral components and only points with
|lateral| < corridor_half count. Shelving alongside the track is then ignored no
matter how close, while a box on the line is caught every time.
"""


import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        self.declare_parameter('image_topic', '/a200_0000/sensors/camera_0/color/image')
        self.declare_parameter('scan_topic', '/a200_0000/sensors/lidar2d_0/scan')
        self.declare_parameter('odom_topic', '/a200_0000/platform/odom')
        self.declare_parameter('cmd_topic', '/a200_0000/cmd_vel')
        self.declare_parameter('control_rate', 20.0)

        # Detection. The paint renders around hue 26 (OpenCV's 0-179 scale) under
        # warehouse lighting; the band is wide enough to survive the shading
        # across the frame but excludes the concrete, which is unsaturated.
        self.declare_parameter('hue_min', 18)
        self.declare_parameter('hue_max', 38)
        self.declare_parameter('sat_min', 120)
        self.declare_parameter('val_min', 60)
        # Only look at the bottom of the frame. The far half of the image sees
        # the warehouse's own yellow hazard tape and yellow shelving, which are
        # not the track; the near band sees floor and nothing else.
        self.declare_parameter('roi_top', 0.55)
        # Crop the very bottom of the frame too. That strip is the ground right
        # in front of the bumper (~0.8 m), where a given sideways offset
        # subtends the largest angle: stepping 0.6-0.8 m aside to clear a box
        # puts the line at an image error of 1.2-1.5 there, i.e. off the edge of
        # the picture. The follower then loses the line precisely when it is
        # avoiding, hunts for it, and can pick it up facing the wrong way.
        # Ignoring the nearest strip keeps the paint in view throughout.
        self.declare_parameter('roi_bottom', 0.85)
        self.declare_parameter('min_area_px', 300)

        # Control.
        self.declare_parameter('forward_speed', 0.6)
        self.declare_parameter('k_steer', 1.5)
        self.declare_parameter('max_angular', 1.2)
        # How much to slow down on a hard turn: speed is scaled by
        # (1 - curve_slowdown * |error|), so a full-lock error costs this much.
        self.declare_parameter('curve_slowdown', 0.6)

        # Lost-line behaviour: coast on the last command briefly (a corner taken
        # slightly wide drops the line for a few frames), then rotate in place
        # toward whichever side it was last seen until it comes back.
        # Must exceed the camera's worst frame gap, not its nominal period.
        # Measured in sim time: 19.6 Hz against a nominal 30, median gap 33 ms
        # but worst 363 ms. At 0.3 s the follower kept declaring the line lost
        # while it was in plain view, which reset the detour's confirmation
        # streak and made 7 of 9 detours time out.
        self.declare_parameter('detection_timeout', 0.7)
        self.declare_parameter('lost_coast_sec', 0.5)
        self.declare_parameter('search_angular', 0.5)
        # Hard bound on how far the lost-line search may swing from the heading
        # the robot was last travelling on. The camera cannot tell which way
        # along the line it is pointing, so an unbounded spin will happily
        # reacquire the track backwards - measured at 117 and 129 degrees
        # off-tangent. Generous enough for any corner on this track.
        self.declare_parameter('search_limit_deg', 100.0)
        # A reacquired line is only accepted if the robot is still pointing
        # roughly the way it was travelling. This is the one check that makes
        # the follower direction-aware at all: the camera sees a stripe, not an
        # arrow, so a detour that swings the robot far enough round (out can
        # turn 55 deg one way and back sweeps 75 deg the other) will otherwise
        # rejoin the track reversed and follow it perfectly happily - measured
        # at 133 deg off-tangent with a cross-track error of 0.01 m.
        self.declare_parameter('align_limit_deg', 90.0)
        # How long the remembered travel heading stays trustworthy. Past this it
        # describes track the robot has already left, so the reversal check
        # stops applying rather than holding the robot to a stale direction.
        self.declare_parameter('travel_yaw_max_age', 15.0)

        # Obstacle avoidance.
        # corridor_half is the half-width of the swept corridor the robot checks
        # ahead: the a200 is 0.67 m wide, so 0.45 m leaves ~0.12 m of margin per
        # side while still ignoring props alongside the track.
        # --- obstacle avoidance: bug0 boundary following --------------------
        # Ported from bug0_a300 in this workspace, whose constants are already
        # tuned for this chassis (0.99 x 0.67 m, TwistStamped cmd_vel).
        #
        # This replaced two earlier attempts - an open-loop three-phase detour,
        # then a sideways bias on the line error - both of which steered by
        # dead reckoning and drifted metres off the track. Boundary following
        # closes the loop on the obstacle itself: the robot holds a set distance
        # off the box and rounds it, so the excursion is bounded by the box
        # rather than by how well a fixed manoeuvre was guessed. Losing the wall
        # past the far corner turns the robot back toward the line for free.
        self.declare_parameter('wall_distance', 0.70)
        self.declare_parameter('wall_dist_kp', 1.6)
        self.declare_parameter('wall_align_kp', 1.1)
        self.declare_parameter('wall_beam_half_deg', 8.0)
        self.declare_parameter('wall_lost_distance', 2.4)
        self.declare_parameter('wall_speed', 0.35)
        self.declare_parameter('wall_reacquire_turn', 0.50)
        self.declare_parameter('front_stop_distance', 0.85)
        self.declare_parameter('front_sector_deg', 35.0)
        self.declare_parameter('front_clear_distance', 2.0)
        # Do not quit the wall the instant the line flickers into view - the
        # robot is still beside the box then.
        self.declare_parameter('min_distance_from_hit', 0.60)
        # Leave the wall when the robot is pointing back along its original
        # direction of travel and the way ahead is clear - bug0's "goal heading
        # is free", with the direction of travel standing in for the goal.
        #
        # The first port used "the camera can see the line" instead, and the
        # robot circled the box for the whole run without ever leaving: while
        # hugging an obstacle it sits ~0.95 m off the paint with the box in the
        # way, so that condition is unsatisfiable exactly when it is being
        # tested. A leave test has to be checkable from where the robot
        # actually is.
        self.declare_parameter('wall_follow_timeout', 25.0)
        # Settle time after leaving an obstacle before another can be latched.
        # Without it the robot leaves the wall, turns back toward the line,
        # sweeps the box it just rounded into its corridor and latches the same
        # obstacle again: 11 of 19 triggers in one run fired at 0.0-0.9 m, all
        # of them re-detections of a box already dealt with.
        self.declare_parameter('corridor_half', 0.45)
        self.declare_parameter('obstacle_trigger', 1.6)   # m: start the detour
        # How far to run on past an obstacle before curving back. Sector-based
        # exits were tried first and are unreliable: driving parallel to a box
        # holds its lateral range roughly constant, so the phase only ever ended
        # on its timeout. A travelled distance is predictable and enough here -
        # 2.0 m clears a 0.5 m box plus the robot's own length.
        # How far to angle back toward the line during the return, measured
        # from the heading the robot held when the detour began. Bounding the
        # return this way is what stops a detour from turning the robot around:
        # the camera cannot tell which way along the line it is pointing, so
        # without this a wide return would rejoin heading back the way it came
        # and the robot would ping-pong over one stretch of track for ever.
        # Hard bound on how far the swing-out may turn the robot from the
        # heading it held when the detour began. Without it 'out' turns until
        # the corridor clears, which against a wall means turning at
        # avoid_angular for the whole phase timeout - most of a full circle,
        # leaving the robot pointing anywhere.
        # Abort the straight run if anything comes within this - the wall case.
        # Consecutive good frames needed before a detour is called finished.
        # One frame is not enough: the warehouse has its own yellow hazard tape
        # and yellow shelving, and a robot that is off the track pointing at
        # scenery can catch a single frame of it, declare itself back on the
        # line, and drive away steering at something that is not the track.
        # Proximity guard for manoeuvring. Distinct from the corridor test on
        # purpose: the corridor is +-0.45 m and is the right shape for spotting
        # a box standing in the lane ahead, but it is the wrong shape for not
        # hitting one. The moment the robot turns, the box leaves the corridor,
        # forward_distance() goes infinite, the creep guard lets go and the
        # robot drives into it diagonally - measured at 0.12 m. This is a plain
        # nearest-return-in-front check over a wide arc, used to gate forward
        # motion during a detour.
        self.declare_parameter('proximity_arc_deg', 70.0)
        self.declare_parameter('proximity_stop', 0.8)
        self.declare_parameter('k_heading', 1.5)
        # Ignore fresh triggers briefly after finishing a detour, so a box still
        # sliding out of the corridor cannot immediately restart one.
        self.declare_parameter('rearm_sec', 1.0)
        self.declare_parameter('scan_timeout', 1.0)

        p = self.get_parameter
        self.rate = p('control_rate').value
        self.forward_speed = p('forward_speed').value
        self.k_steer = p('k_steer').value
        self.max_angular = p('max_angular').value
        self.curve_slowdown = p('curve_slowdown').value
        self.detection_timeout = p('detection_timeout').value
        self.lost_coast_sec = p('lost_coast_sec').value
        self.search_angular = p('search_angular').value
        self.search_limit = np.radians(p('search_limit_deg').value)
        self.align_limit = np.radians(p('align_limit_deg').value)
        self.travel_yaw_max_age = p('travel_yaw_max_age').value
        self.roi_top = p('roi_top').value
        self.roi_bottom = p('roi_bottom').value
        self.min_area_px = p('min_area_px').value
        self.corridor_half = p('corridor_half').value
        self.obstacle_trigger = p('obstacle_trigger').value
        self.proximity_arc = np.radians(p('proximity_arc_deg').value)
        self.proximity_stop = p('proximity_stop').value
        self.k_heading = p('k_heading').value
        self.wall_distance = p('wall_distance').value
        self.wall_dist_kp = p('wall_dist_kp').value
        self.wall_align_kp = p('wall_align_kp').value
        self.wall_beam_half = np.radians(p('wall_beam_half_deg').value)
        self.wall_lost_distance = p('wall_lost_distance').value
        self.wall_speed = p('wall_speed').value
        self.wall_reacquire_turn = p('wall_reacquire_turn').value
        self.front_stop_distance = p('front_stop_distance').value
        self.front_sector = np.radians(p('front_sector_deg').value)
        self.front_clear_distance = p('front_clear_distance').value
        self.min_distance_from_hit = p('min_distance_from_hit').value
        self.wall_follow_timeout = p('wall_follow_timeout').value
        self.rearm_sec = p('rearm_sec').value
        self.scan_timeout = p('scan_timeout').value
        self.lo = np.array([p('hue_min').value, p('sat_min').value, p('val_min').value], np.uint8)
        self.hi = np.array([p('hue_max').value, 255, 255], np.uint8)

        # Detection state, written by the image callback, read by the timer.
        self.have_image = False
        self.had_line = False
        self.error = None           # -1 (line far left) .. +1 (line far right)
        self.last_error = 0.0
        self.last_seen = None       # ROS time of the last successful detection
        self.last_cmd = (0.0, 0.0)

        # Scan state, written by the scan callback, read by the timer.
        self.scan_fwd = None        # forward component of each beam, metres
        self.scan_lat = None        # lateral component (+ve to the left)
        self.scan_ang = None
        self.scan_rng = None
        self.scan_stamp = None

        # 'follow' = camera on the line; 'wall' = rounding an obstacle.
        self.mode = 'follow'
        self.wall_side = 1.0        # +1: obstacle on our left, we pass right
        self.hit_xy = None          # where we met it, for the leave test
        self.wall_since = None      # when we started rounding it
        self.left_wall_at = None    # when we last finished, for the re-arm
        self.detours = 0

        # Odometry, for the reversal check only.
        self.odom_xy = None
        self.yaw = None
        self.travel_yaw = None      # heading while actually following the line
        self.travel_yaw_stamp = None

        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, p('image_topic').value, self.on_image, sensor_qos)
        self.create_subscription(LaserScan, p('scan_topic').value, self.on_scan, sensor_qos)
        self.create_subscription(Odometry, p('odom_topic').value, self.on_odom, 10)
        self.pub = self.create_publisher(TwistStamped, p('cmd_topic').value, 10)
        self.create_timer(1.0 / self.rate, self.on_timer)

        self.get_logger().info(
            f"following {p('image_topic').value} -> {p('cmd_topic').value} "
            f'at {self.forward_speed} m/s, avoiding via {p("scan_topic").value}')

    # -- perception ---------------------------------------------------------

    def on_image(self, msg):
        self.have_image = True
        try:
            frame = to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warn(str(exc), throttle_duration_sec=5.0)
            return

        h, w = frame.shape[:2]
        roi = frame[int(self.roi_top * h):int(self.roi_bottom * h), :]
        mask = cv2.inRange(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV), self.lo, self.hi)
        # Close small holes so a speckle of concrete showing through the paint
        # does not split one blob into two.
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.error = None
            return
        # Largest blob only: if a stray yellow object does creep into the ROI,
        # the track is the bigger of the two at this range.
        blob = max(contours, key=cv2.contourArea)
        if cv2.contourArea(blob) < self.min_area_px:
            self.error = None
            return
        m = cv2.moments(blob)
        if m['m00'] <= 0.0:
            self.error = None
            return

        cx = m['m10'] / m['m00']
        self.error = (cx - w / 2.0) / (w / 2.0)
        self.last_error = self.error
        self.last_seen = self.get_clock().now()

    def on_scan(self, msg):
        r = np.asarray(msg.ranges, dtype=np.float64)
        if self.scan_ang is None or len(self.scan_ang) != len(r):
            self.scan_ang = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        # Drop the invalid returns and anything inside the robot's own footprint,
        # which the lidar sees as the sensor mast and bumpers.
        good = np.isfinite(r) & (r > max(msg.range_min, 0.25)) & (r < msg.range_max)
        r = np.where(good, r, np.inf)
        self.scan_rng = r
        self.scan_fwd = r * np.cos(self.scan_ang)
        self.scan_lat = r * np.sin(self.scan_ang)
        self.scan_stamp = self.get_clock().now()

    def on_odom(self, msg):
        pos = msg.pose.pose.position
        self.odom_xy = (pos.x, pos.y)
        q = msg.pose.pose.orientation
        self.yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    # -- lidar queries ------------------------------------------------------

    def scan_is_fresh(self):
        if self.scan_stamp is None:
            return False
        age = (self.get_clock().now() - self.scan_stamp).nanoseconds * 1e-9
        return age <= self.scan_timeout

    def forward_distance(self):
        """Distance to the nearest thing standing inside the robot's own width."""
        if not self.scan_is_fresh():
            return float('inf')     # no scan is not the same as blocked
        m = (self.scan_fwd > 0.0) & (np.abs(self.scan_lat) < self.corridor_half)
        return float(self.scan_fwd[m].min()) if m.any() else float('inf')

    def near_distance(self):
        """Nearest return anywhere in a wide arc ahead, whatever its bearing."""
        return self.sector_min(-self.proximity_arc, self.proximity_arc)

    def too_close(self):
        return self.near_distance() < self.proximity_stop

    def sector_min(self, lo, hi):
        """Nearest return with bearing in [lo, hi] radians (+ve to the left)."""
        if not self.scan_is_fresh():
            return float('inf')
        m = (self.scan_ang >= lo) & (self.scan_ang <= hi)
        return float(self.scan_rng[m].min()) if m.any() else float('inf')

    # -- control ------------------------------------------------------------

    def on_timer(self):
        if not self.have_image:
            return                                  # never drive blind

        # A detection counts only while it is fresh. Without this age check the
        # `self.error is not None` branch would hold forever if frames simply
        # stopped arriving - self.error is only ever cleared by on_image, so a
        # dead camera would leave the robot driving on its last good frame
        # indefinitely, which is the exact failure this timer exists to avoid.
        age = self.age_of_last_detection()
        fresh = self.error is not None and age is not None and age <= self.detection_timeout
        if fresh != self.had_line:
            self.get_logger().info(
                f'line {"REACQUIRED" if fresh else "LOST"} (mode {self.mode})')
            self.had_line = fresh

        linear, angular = self.follow_command(age, fresh)

        # Last-resort collision stop, applied to whatever either controller
        # asked for. This existed but was never actually called - the check is
        # cheap and is the only thing standing between a control bug and the
        # robot pushing into something.
        if linear > 0.0 and self.too_close():
            linear = 0.0

        self.last_cmd = (linear, angular)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear)
        msg.twist.angular.z = float(angular)
        self.pub.publish(msg)

    def aligned(self):
        """Still pointing roughly the way we were going when last on the line.

        Only meaningful while the remembered heading is recent. After a long
        detour it describes a stretch of track the robot has already left, and
        holding the robot to it makes it spin on the spot trying to match a
        direction that no longer applies.
        """
        if self.yaw is None or self.travel_yaw is None:
            return True
        if self.travel_yaw_stamp is None:
            return True
        age = (self.get_clock().now() - self.travel_yaw_stamp).nanoseconds * 1e-9
        if age > self.travel_yaw_max_age:
            return True
        return abs(wrap_angle(self.yaw - self.travel_yaw)) < self.align_limit

    def follow_command(self, age, fresh):
        """Line following, handing off to boundary following around obstacles."""
        if self.mode == 'wall':
            return self.wall_follow_command(fresh)

        if self.forward_distance() < self.obstacle_trigger and self.rearmed():
            self.enter_wall_follow()
            return self.wall_follow_command(fresh)

        if fresh and not self.aligned():
            # On the line but facing back down it. Do not steer to it - that
            # just drives the lap in reverse.
            self.get_logger().warn('line reacquired facing backwards - turning around',
                                   throttle_duration_sec=3.0)
            return 0.0, clamp(self.k_heading * wrap_angle(self.travel_yaw - self.yaw),
                              self.max_angular)

        if fresh:
            if self.yaw is not None:
                self.travel_yaw = self.yaw
                self.travel_yaw_stamp = self.get_clock().now()
            err = self.error
            angular = clamp(-self.k_steer * err, self.max_angular)
            linear = self.forward_speed * (1.0 - self.curve_slowdown * min(1.0, abs(err)))
        elif age is not None and age < self.lost_coast_sec:
            linear, angular = self.last_cmd              # coast
        else:
            direction = -1.0 if self.last_error > 0.0 else 1.0
            # Sweep, do not spin: turn back at the bound rather than going round
            # far enough to pick the line up reversed.
            if self.yaw is not None and self.travel_yaw is not None:
                dev = wrap_angle(self.yaw - self.travel_yaw)
                if dev > self.search_limit:
                    direction = -1.0
                elif dev < -self.search_limit:
                    direction = 1.0
            linear, angular = 0.0, direction * self.search_angular
        return linear, angular

    # -- bug0 boundary following ---------------------------------------------

    def enter_wall_follow(self):
        """Start rounding an obstacle, on whichever side has more room."""
        left = self.sector_min(np.radians(20), np.radians(100))
        right = self.sector_min(np.radians(-100), np.radians(-20))
        # Pass on the roomier side; the obstacle is then on the other flank,
        # and that flank is the wall we follow.
        self.wall_side = -1.0 if left >= right else 1.0
        self.hit_xy = self.odom_xy
        self.wall_since = self.get_clock().now()
        self.mode = 'wall'
        self.detours += 1
        self.get_logger().info(
            f'obstacle at {self.forward_distance():.2f} m - rounding it on the '
            f'{"left" if self.wall_side < 0 else "right"} '
            f'(left {left:.2f} m, right {right:.2f} m)')

    def rearmed(self):
        if self.left_wall_at is None:
            return True
        return (self.get_clock().now() - self.left_wall_at).nanoseconds * 1e-9 > self.rearm_sec

    def wall_measurements(self):
        """Distance to the wall and its angle, from a perpendicular and a
        diagonal beam. Two beams give the wall's slope, so the robot can run
        parallel to it instead of sawing in and out."""
        side = self.wall_side
        d_perp = self.sector_min(side * np.pi / 2 - self.wall_beam_half,
                                 side * np.pi / 2 + self.wall_beam_half)
        d_diag = self.sector_min(side * np.pi / 4 - self.wall_beam_half,
                                 side * np.pi / 4 + self.wall_beam_half)
        if not np.isfinite(d_perp):
            return float('inf'), None
        if not np.isfinite(d_diag):
            return d_perp, None
        c45 = s45 = np.sqrt(0.5)
        phi = np.arctan2(d_diag * s45 - d_perp, d_diag * c45)
        return d_perp * np.cos(phi), phi

    def moved_from_hit(self):
        if self.hit_xy is None or self.odom_xy is None:
            return 0.0
        return float(np.hypot(self.odom_xy[0] - self.hit_xy[0],
                              self.odom_xy[1] - self.hit_xy[1]))

    def wall_follow_command(self, fresh):
        side = self.wall_side
        front = self.sector_min(-self.front_sector, self.front_sector)

        wall_d, wall_a = self.wall_measurements()

        # Leave once the obstacle is genuinely behind: its side has dropped out
        # of range (we are past the far corner), the way ahead is clear, and we
        # have actually travelled.
        #
        # This used to also require pointing within 30 deg of travel_yaw - the
        # heading held when the obstacle was first seen. That is frozen for the
        # whole detour, so wherever the track curves across a box the correct
        # heading afterwards differs from it by more than the tolerance and the
        # test can never pass: the robot circled until the timeout instead of
        # leaving. It explains both the timeouts (13 of 19 encounters in one
        # run) and the swing between runs, since it depended on whether a box
        # happened to sit on a straight or a bend. Every term below is measured
        # here and now, so nothing goes stale.
        wall_gone = wall_d > self.wall_lost_distance
        # Leave for the mirror image of the reason we entered: the lane ahead
        # is clear again, and we have travelled far enough to be past the
        # obstacle rather than merely pointing away from it.
        #
        # An earlier version also demanded the side beam read beyond
        # wall_lost_distance. That is not a fact about the obstacle at all -
        # once past the box that beam sees warehouse walls and shelving, which
        # are frequently nearer than 2.4 m, so the condition never came true and
        # 6 of 9 encounters ran to the timeout. Both terms below are about the
        # robot's own lane, which is what actually decides whether it can drive.
        lane_clear = self.forward_distance() > self.obstacle_trigger
        gone_far_enough = self.moved_from_hit() >= self.min_distance_from_hit
        stuck = (self.wall_since is not None and
                 (self.get_clock().now() - self.wall_since).nanoseconds * 1e-9
                 > self.wall_follow_timeout)
        if (lane_clear and gone_far_enough) or stuck:
            self.get_logger().info(
                f'obstacle #{self.detours} '
                f'{"rounded" if not stuck else "ABANDONED (timeout)"} after '
                f'{self.moved_from_hit():.2f} m - back to the line')
            # Refresh the direction-of-travel reference from the detour itself.
            # The straight line from where we met the obstacle to where we are
            # now IS the direction we travelled getting past it, so it is both
            # current and correct - unlike the heading remembered from before
            # the detour, which goes stale and had to be expired, switching the
            # reversal check off exactly when it was needed. Measured on the
            # go1: 3 reversals in 200 s with the check expiring, and it is
            # reversals, not slow progress, that hold coverage down.
            if self.hit_xy is not None and self.odom_xy is not None:
                dx = self.odom_xy[0] - self.hit_xy[0]
                dy = self.odom_xy[1] - self.hit_xy[1]
                if np.hypot(dx, dy) >= 0.5:
                    self.travel_yaw = float(np.arctan2(dy, dx))
                    self.travel_yaw_stamp = self.get_clock().now()
            self.mode = 'follow'
            self.hit_xy = None
            self.wall_since = None
            self.left_wall_at = self.get_clock().now()
            return self.last_cmd

        if front < self.front_stop_distance:
            # Nose into it: turn away on the spot rather than pushing.
            return 0.0, -side * self.max_angular

        if wall_gone:
            # Past the far corner. Curving back toward where the wall was is
            # what carries the robot round the box and onto the line again.
            return self.wall_speed * 0.7, side * self.wall_reacquire_turn

        angular = side * self.wall_dist_kp * (wall_d - self.wall_distance)
        if wall_a is not None:
            angular += side * self.wall_align_kp * wall_a
        if front < self.front_clear_distance:
            angular += 0.6 * -side * self.max_angular
        angular = clamp(angular, self.max_angular)
        speed = self.wall_speed * (1.0 - 0.5 * min(abs(angular) / self.max_angular, 1.0))
        return speed, angular

    def age_of_last_detection(self):
        if self.last_seen is None:
            return None
        return (self.get_clock().now() - self.last_seen).nanoseconds * 1e-9

    def stop(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def wrap_angle(a):
    """Signed angle difference folded into [-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def clamp(value, limit):
    return max(-limit, min(limit, value))


def to_bgr(msg):
    """sensor_msgs/Image -> BGR ndarray, without pulling in cv_bridge."""
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'unsupported image encoding {msg.encoding!r}')
    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    return frame[:, :, ::-1] if msg.encoding == 'rgb8' else frame


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
