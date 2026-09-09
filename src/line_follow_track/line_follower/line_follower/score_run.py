"""Score a line-following run against the track's ground-truth centreline.

Ground truth comes from Gazebo's own pose stream, not from odometry: the a200's
skid-steer odometry drifts on the order of a metre over a run this long, which
would be scored as tracking error the follower never made.

It is read by streaming `gz topic -e` and parsing it, rather than through
ros_gz_bridge, because the Pose_V -> TFMessage conversion drops the entity name -
every transform arrives with an empty child_frame_id, so there is no way to pick
the robot out of the message.

The centreline comes from line_follow_sim/config/track_centerline.csv, which
gen_track_texture.py writes from the same drawing the decal is painted from, so
it is the actual painted line rather than an approximation of it.
"""

import argparse
import re
import subprocess
import sys
import threading
import time

import numpy as np


NAME_RE = re.compile(r'name:\s*"([^"]*)"')
FLOAT_RE = re.compile(r'([xyz]):\s*(-?[\d.eE+-]+)')


def stream_poses(proc, model, samples, period, stop):
    """Append (t, x, y) for `model` to `samples` until `stop` is set.

    Parses the text `gz topic -e` prints. Blocks are shaped:

        pose {
          name: "a200_0000/robot"
          ...
          position {
            x: 5.0
            y: 13.26
    """
    in_model = False
    in_position = False
    pending = {}
    for line in proc.stdout:
        if stop.is_set():
            break
        stripped = line.strip()
        name = NAME_RE.match(stripped)
        if name:
            in_model = name.group(1) == model
            in_position = False
            pending = {}
            continue
        if not in_model:
            continue
        if stripped.startswith('position'):
            in_position = True
            continue
        if stripped.startswith('orientation'):
            in_position = False
            continue
        if in_position:
            m = FLOAT_RE.match(stripped)
            if m:
                pending[m.group(1)] = float(m.group(2))
                if 'x' in pending and 'y' in pending:
                    now = time.monotonic()
                    if not samples or now - samples[-1][0] >= period:
                        samples.append((now, pending['x'], pending['y']))
                    in_model = in_position = False
                    pending = {}


def report(samples, centerline, args):
    """Print the run's numbers; return True if it passes."""
    if len(samples) < 2:
        print('FAIL: %d pose samples for model %r on %s'
              % (len(samples), args.robot_model, args.pose_topic))
        print('      is the sim running, and is the robot moving? A stationary')
        print('      model publishes nothing on dynamic_pose/info.')
        return False

    s = np.array(samples)
    xy = s[:, 1:3]

    # Cross-track error: distance to the nearest point on the painted centreline.
    d = np.hypot(xy[:, None, 0] - centerline[None, :, 0],
                 xy[:, None, 1] - centerline[None, :, 1])
    xte = d.min(axis=1)
    nearest = d.argmin(axis=1)

    path_len = float(np.hypot(*np.diff(xy, axis=0).T).sum())
    duration = float(s[-1, 0] - s[0, 0])
    # Loop coverage in one-metre bins, so dawdling in one spot earns nothing.
    line_len = float(np.hypot(*np.diff(centerline, axis=0).T).sum())
    bins = max(1, int(round(line_len)))
    visited = np.unique((nearest * bins) // len(centerline))
    coverage = len(visited) / bins

    print('--- line following run ---------------------------------------')
    print('duration            %.1f s over %d samples' % (duration, len(s)))
    print('path length         %.1f m  (track loop is %.1f m)' % (path_len, line_len))
    print('loop coverage       %.0f%% (%d of %d one-metre bins)'
          % (100 * coverage, len(visited), bins))
    print('cross-track error   mean %.3f m | median %.3f m | p95 %.3f m | max %.3f m'
          % (xte.mean(), np.median(xte), np.percentile(xte, 95), xte.max()))
    print('                    within 0.15 m: %.0f%%   within 0.30 m: %.0f%%'
          % (100 * (xte < 0.15).mean(), 100 * (xte < 0.30).mean()))
    worst = xy[xte.argmax()]
    print('worst point         (%.2f, %.2f), %.3f m off the line'
          % (worst[0], worst[1], xte.max()))

    checks = [
        ('max cross-track error <= %.2f m' % args.max_xte, xte.max() <= args.max_xte),
        ('path length >= %.1f m' % args.min_path, path_len >= args.min_path),
        ('loop coverage >= %.0f%%' % (100 * args.min_coverage), coverage >= args.min_coverage),
    ]
    print('--- checks ---------------------------------------------------')
    for name, ok in checks:
        print('  [%s] %s' % ('PASS' if ok else 'FAIL', name))
    passed = all(ok for _, ok in checks)
    print('=== %s ===' % ('PASS' if passed else 'FAIL'))

    if args.out:
        np.savetxt(args.out, np.column_stack([s, xte]), fmt='%.4f', delimiter=',',
                   header='t,x,y,cross_track_error')
        print('wrote %s' % args.out)
    return passed


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--centerline', required=True)
    ap.add_argument('--duration', type=float, default=120.0,
                    help='seconds of driving to score (default 120)')
    ap.add_argument('--pose-topic', default='/world/warehouse_line/dynamic_pose/info')
    ap.add_argument('--robot-model', default='a200_0000/robot')
    ap.add_argument('--sample-rate', type=float, default=10.0)
    ap.add_argument('--max-xte', type=float, default=0.50,
                    help='fail if the robot ever gets this far off the line')
    ap.add_argument('--min-path', type=float, default=10.0)
    ap.add_argument('--min-coverage', type=float, default=0.0)
    ap.add_argument('--out', default=None, help='write per-sample CSV here')
    args = ap.parse_args(argv if argv is not None else sys.argv[1:])

    centerline = np.loadtxt(args.centerline, delimiter=',')
    samples = []
    stop = threading.Event()
    proc = subprocess.Popen(['gz', 'topic', '-e', '-t', args.pose_topic],
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
    reader = threading.Thread(target=stream_poses, daemon=True, args=(
        proc, args.robot_model, samples, 1.0 / args.sample_rate, stop))
    reader.start()

    deadline = time.monotonic() + args.duration
    try:
        while time.monotonic() < deadline:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        # The reader is a daemon blocked on proc.stdout: if the stream has gone
        # quiet it will never notice `stop` and never clean up after itself, so
        # the subprocess has to be killed from here or it outlives us.
        stop.set()
        proc.terminate()
        proc.wait(timeout=5)

    sys.exit(0 if report(list(samples), centerline, args) else 1)


if __name__ == '__main__':
    main()
