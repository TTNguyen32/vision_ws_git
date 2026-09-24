#!/usr/bin/env python3
"""
tf_displacement.py - aggregate one TF edge into displacement statistics.

Two modes:

  live:  subscribe to /tf, filter one parent->child edge, summarise on Ctrl-C
             ./tf_displacement.py --parent odom --child base_link
             ./tf_displacement.py --parent world --child Tin_LIDAR

  file:  parse text captured earlier from `ros2 topic echo /tf`
             ros2 topic echo /tf > tf_run.txt        # during the run
             ./tf_displacement.py --from-file tf_run.txt --parent odom --child base_link

Run it with /usr/bin/python3 so conda's `base` env is bypassed: live mode
needs rclpy and file mode needs PyYAML, and ROS installs both there.

Reads only the direct parent->child edge, not a composed chain. With the
LiDAR extrinsic at identity that is the same thing; if you later set a
real extrinsic, the body edge is still the one you want for displacement.
"""

import argparse
import math
import sys


# ---------------------------------------------------------------- helpers

def stamp_to_sec(stamp):
    """ROS stamp (dict or msg) -> float seconds."""
    if isinstance(stamp, dict):
        return stamp["sec"] + stamp["nanosec"] * 1e-9
    return stamp.sec + stamp.nanosec * 1e-9


def quat_to_yaw(x, y, z, w):
    """Yaw about Z, in radians. Standard quaternion -> Euler Z component."""
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


def quat_angle_between(a, b):
    """Absolute rotation angle between two quaternions, radians.

    dot = cos(theta/2) for unit quaternions; abs() folds q and -q, which
    represent the same rotation.
    """
    dot = abs(sum(ai * bi for ai, bi in zip(a, b)))
    return 2.0 * math.acos(min(1.0, dot))


class Accumulator:
    """Collects (t, x, y, z, quat) samples and reports displacement stats."""

    def __init__(self, min_step):
        self.min_step = min_step
        self.samples = []      # (t, x, y, z, (qx,qy,qz,qw))
        self.path_len = 0.0    # sum of accepted step lengths
        self.rejected = 0      # steps below min_step (jitter while hovering)
        self.max_speed = 0.0

    def add(self, t, x, y, z, quat):
        if self.samples:
            t0, x0, y0, z0, _ = self.samples[-1]
            step = math.dist((x, y, z), (x0, y0, z0))

            # Below-threshold steps are almost all mocap jitter. Summing them
            # would inflate path length without bound while the drone hovers.
            if step < self.min_step:
                self.rejected += 1
                return

            dt = t - t0
            if dt > 0:
                self.max_speed = max(self.max_speed, step / dt)
            self.path_len += step

        self.samples.append((t, x, y, z, quat))

    def report(self, parent, child, out=sys.stdout):
        n = len(self.samples)
        if n < 2:
            print(f"Only {n} sample(s) for {parent} -> {child}. "
                  f"Nothing to measure.", file=out)
            return

        t0, x0, y0, z0, q0 = self.samples[0]
        t1, x1, y1, z1, q1 = self.samples[-1]

        duration = t1 - t0
        net = math.dist((x1, y1, z1), (x0, y0, z0))
        far = max(math.dist((s[1], s[2], s[3]), (x0, y0, z0))
                  for s in self.samples)

        xs = [s[1] for s in self.samples]
        ys = [s[2] for s in self.samples]
        zs = [s[3] for s in self.samples]

        yaws = [quat_to_yaw(*s[4]) for s in self.samples]
        # Unwrap so a pass through +/-pi is not counted as a 2*pi jump.
        yaw_total = 0.0
        for a, b in zip(yaws, yaws[1:]):
            d = (b - a + math.pi) % (2.0 * math.pi) - math.pi
            yaw_total += abs(d)

        print(f"\n=== {parent} -> {child} ===", file=out)
        print(f"Samples kept      : {n}  ({self.rejected} below "
              f"{self.min_step * 1000:.1f} mm, treated as jitter)", file=out)
        if duration > 0:
            print(f"Duration          : {duration:.2f} s   "
                  f"({n / duration:.1f} Hz effective)", file=out)
        else:
            print(f"Duration          : {duration:.2f} s", file=out)
        print(f"", file=out)
        print(f"Net displacement  : {net:.4f} m   "
              f"(start -> end, straight line)", file=out)
        print(f"Path length       : {self.path_len:.4f} m   "
              f"(distance actually travelled)", file=out)
        print(f"Max from start    : {far:.4f} m", file=out)
        print(f"", file=out)
        print(f"Start             : ({x0:+.4f}, {y0:+.4f}, {z0:+.4f})", file=out)
        print(f"End               : ({x1:+.4f}, {y1:+.4f}, {z1:+.4f})", file=out)
        print(f"Extent  x         : {min(xs):+.4f} .. {max(xs):+.4f}  "
              f"({max(xs) - min(xs):.4f} m)", file=out)
        print(f"        y         : {min(ys):+.4f} .. {max(ys):+.4f}  "
              f"({max(ys) - min(ys):.4f} m)", file=out)
        print(f"        z         : {min(zs):+.4f} .. {max(zs):+.4f}  "
              f"({max(zs) - min(zs):.4f} m)", file=out)
        print(f"", file=out)
        if duration > 0:
            print(f"Mean speed        : {self.path_len / duration:.3f} m/s",
                  file=out)
        print(f"Max step speed    : {self.max_speed:.3f} m/s", file=out)
        print(f"Yaw swept         : {math.degrees(yaw_total):.1f} deg "
              f"(cumulative |dyaw|)", file=out)
        print(f"Net rotation      : "
              f"{math.degrees(quat_angle_between(q0, q1)):.1f} deg "
              f"(start -> end)", file=out)

    def write_csv(self, path):
        import csv
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "x", "y", "z", "qx", "qy", "qz", "qw"])
            for t, x, y, z, q in self.samples:
                w.writerow([f"{t:.9f}", x, y, z, *q])
        print(f"\nWrote {len(self.samples)} samples to {path}")


# ---------------------------------------------------------------- file mode

def clean_echo_text(text):
    """Strip rmw loss notices from `ros2 topic echo` output.

    When the middleware drops samples, echo interleaves non-YAML lines:

        A message was lost!!!
        \ttotal count change:707
        \ttotal count: 707---        <-- note the glued document separator

    Tabs cannot start a YAML token, so the parser dies. Returns the
    cleaned text and the number of messages reported lost.
    """
    kept, lost = [], 0

    for line in text.splitlines():
        if line.startswith("A message was lost"):
            continue
        if line.startswith("\t") or line.startswith("    total count"):
            if "total count change:" in line:
                try:
                    lost += int(line.split("total count change:")[1]
                                .split("-")[0].strip())
                except ValueError:
                    pass
            # The separator for the next document can be glued onto the end
            # of a notice line; keep it or every later document merges.
            if line.rstrip().endswith("---"):
                kept.append("---")
            continue
        kept.append(line)

    return "\n".join(kept), lost


def run_file(args, acc):
    try:
        import yaml
    except ImportError:
        sys.exit("PyYAML not found. Run this with /usr/bin/python3, or "
                 "conda run -n lidar python " + " ".join(sys.argv))

    with open(args.from_file) as f:
        text, lost = clean_echo_text(f.read())

    if lost:
        print(f"WARNING: capture reports {lost} lost message(s). "
              f"Path length below is an UNDER-estimate.", file=sys.stderr)

    # `ros2 topic echo` emits YAML documents separated by '---'.
    for doc in yaml.safe_load_all(text):
        if not doc:
            continue
        for tf in doc.get("transforms", []):
            if (tf["header"]["frame_id"].lstrip("/") != args.parent
                    or tf["child_frame_id"].lstrip("/") != args.child):
                continue
            tr = tf["transform"]["translation"]
            ro = tf["transform"]["rotation"]
            acc.add(stamp_to_sec(tf["header"]["stamp"]),
                    tr["x"], tr["y"], tr["z"],
                    (ro["x"], ro["y"], ro["z"], ro["w"]))

    acc.report(args.parent, args.child)
    if args.csv:
        acc.write_csv(args.csv)


# ---------------------------------------------------------------- live mode

def run_live(args, acc):
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from tf2_msgs.msg import TFMessage
    except ImportError as e:
        sys.exit(f"rclpy/tf2_msgs import failed ({e}).\n"
                 f"Use /usr/bin/python3 and source your ROS workspace first.")

    class TFWatcher(Node):
        def __init__(self):
            super().__init__("tf_displacement")
            # /tf is reliable with a deep queue: matching it avoids dropping
            # samples, which would understate path length. This is why live
            # mode loses far less than `ros2 topic echo`.
            qos = QoSProfile(depth=500,
                             reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
            self.create_subscription(TFMessage, "/tf", self.cb, qos)
            self.get_logger().info(
                f"Watching {args.parent} -> {args.child}. Ctrl-C to finish.")
            self.create_timer(2.0, self.progress)

        def cb(self, msg):
            for tf in msg.transforms:
                if (tf.header.frame_id.lstrip("/") != args.parent
                        or tf.child_frame_id.lstrip("/") != args.child):
                    continue
                t = tf.transform.translation
                r = tf.transform.rotation
                acc.add(stamp_to_sec(tf.header.stamp),
                        t.x, t.y, t.z, (r.x, r.y, r.z, r.w))

        def progress(self):
            self.get_logger().info(
                f"{len(acc.samples)} samples, path {acc.path_len:.3f} m")

    rclpy.init()
    node = TFWatcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    acc.report(args.parent, args.child)
    if args.csv:
        acc.write_csv(args.csv)


# ---------------------------------------------------------------- main

def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--parent", default="odom",
                   help="header.frame_id of the edge (default: odom)")
    p.add_argument("--child", default="base_link",
                   help="child_frame_id of the edge (default: base_link)")
    p.add_argument("--min-step", type=float, default=0.001,
                   help="ignore steps shorter than this, metres (default: 0.001)")
    p.add_argument("--from-file", metavar="TXT",
                   help="parse saved `ros2 topic echo /tf` output instead of subscribing")
    p.add_argument("--csv", metavar="OUT.csv",
                   help="also write the raw samples")
    args = p.parse_args()

    acc = Accumulator(args.min_step)
    (run_file if args.from_file else run_live)(args, acc)


if __name__ == "__main__":
    main()
