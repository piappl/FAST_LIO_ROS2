#!/usr/bin/env python3
"""
fake_livox_pub.py -- synthetic Livox Mid-360 / HAP publisher.

Purpose: rehearse and validate the whole perf harness (stream_monitor,
resource_monitor, analyze.py) on ANY machine, with no sensor and no bag, and
with the ability to *inject on purpose* the exact faults we suspect on the
Jetson. If the harness catches the injected fault here, you can trust its
verdict on the target.

Publishes:
  <ns>/lidar  sensor_msgs/PointCloud2  with the Livox field layout
              (x,y,z FLOAT32, intensity FLOAT32, tag/line UINT8,
               timestamp FLOAT64 = ns since epoch)  -- what mid360_handler parses
  <ns>/imu    sensor_msgs/Imu

Fault injection:
  --drop-imu-frac 0.02      randomly drop 2% of IMU samples (queue-overflow look)
  --imu-stall-every 30      every 30s, stop IMU for --imu-stall-ms
  --imu-stall-ms 300
  --zero-point-ts           publish per-point timestamp = 0 (driver misconfig)
  --ts-span-ms 400          force an absurd per-point time span (unsynced merge)
  --second-imu-publisher    ALSO publish a 2nd, time-offset IMU stream on the
                            same topic -> reproduces "2x Mid-360 on /livox/imu"
  --imu-still               stationary IMU: gravity + Gaussian noise instead of
                            the default sinusoid. This is the mode the IMU-init
                            and pose-stability checks are about -- the default
                            motion makes gravity/bias init meaningless by design.
  --acc-noise-std 0.01      per-axis accelerometer noise, m/s^2
  --gyr-noise-std 0.0007    per-axis gyro noise, rad/s
  --acc-spike-count 1       inject N single-sample accelerometer outliers inside
  --acc-spike-mag 1.0       --acc-spike-window seconds of startup, on
  --acc-spike-axis y        --acc-spike-axis. Reproduces an init whose per-axis
  --acc-spike-window 2.0    acc std is lopsided (one axis 20x the others)
                            because of a couple of bad samples rather than
                            vibration -- see the IMU init warnings.
  --range-noise-std 0.03    per-point lidar range noise, metres. REQUIRED for any
                            pose-stability or IMU-covariance experiment: with the
                            default exact geometry the plane fits have a zero
                            residual, the lidar pins the pose outright, and the
                            IMU covariances cannot make any difference.

Examples:
  # clean reference
  ./fake_livox_pub.py
  # reproduce the two-IMU-on-one-topic fault
  ./fake_livox_pub.py --second-imu-publisher
  # reproduce dropped IMU + collapsed point timestamps
  ./fake_livox_pub.py --drop-imu-frac 0.05 --zero-point-ts
  # stationary, quiet IMU -- the reference for judging IMU init and pose drift
  ./fake_livox_pub.py --imu-still
  # stationary but with two bad accelerometer samples during init
  ./fake_livox_pub.py --imu-still --acc-spike-count 2 --acc-spike-mag 1.0
"""

import argparse
import math
import random
import struct
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu, PointCloud2, PointField

# Livox PointCloud2 layout used by livox_ros_driver2 (LivoxPointXyzitl):
#   float x,y,z, float intensity, uint8 tag, uint8 line, float64 timestamp
POINT_STEP = 26
FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="tag", offset=16, datatype=PointField.UINT8, count=1),
    PointField(name="line", offset=17, datatype=PointField.UINT8, count=1),
    PointField(name="timestamp", offset=18, datatype=PointField.FLOAT64, count=1),
]

NP_DTYPE = np.dtype({
    "names": ["x", "y", "z", "intensity", "tag", "line", "timestamp"],
    "formats": [np.float32, np.float32, np.float32, np.float32,
                np.uint8, np.uint8, np.float64],
    "offsets": [0, 4, 8, 12, 16, 17, 18],
    "itemsize": POINT_STEP,
})


def sec_to_stamp(t: float):
    from builtin_interfaces.msg import Time
    s = int(math.floor(t))
    return Time(sec=s, nanosec=int(round((t - s) * 1e9)))


def _triple(v, what):
    """One value -> all three axes; "x,y,z" -> per axis."""
    parts = [p.strip() for p in str(v).split(",") if p.strip() != ""]
    try:
        vals = [float(p) for p in parts]
    except ValueError:
        raise SystemExit(f"{what}: expected a number or x,y,z, got {v!r}")
    if len(vals) == 1:
        return vals * 3
    if len(vals) == 3:
        return vals
    raise SystemExit(f"{what}: expected 1 or 3 values, got {len(vals)}")


class FakeLivox(Node):
    def __init__(self, a):
        super().__init__("fake_livox_pub")
        self.a = a
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.pub_cloud = self.create_publisher(PointCloud2, f"{a.ns}/lidar", qos)
        self.pub_imu = self.create_publisher(Imu, f"{a.ns}/imu", qos)

        self.t0 = time.time()
        self.imu_n = 0
        self.cloud_n = 0
        self.dropped = 0
        self.rng = random.Random(a.seed)
        self.acc_spikes_left = a.acc_spike_count
        self.cloud_rng = np.random.default_rng(a.seed + 1)
        self.acc_sd = _triple(a.acc_noise_std, "--acc-noise-std")
        self.gyr_sd = _triple(a.gyr_noise_std, "--gyr-noise-std")

        self.create_timer(1.0 / a.cloud_rate, self.tick_cloud)
        self.create_timer(1.0 / a.imu_rate, self.tick_imu)
        self.create_timer(5.0, self.status)

        self.get_logger().info(
            f"fake_livox: {a.ns}/lidar @{a.cloud_rate}Hz ({a.points} pts), "
            f"{a.ns}/imu @{a.imu_rate}Hz | faults: "
            f"drop_imu={a.drop_imu_frac} stall_every={a.imu_stall_every}s "
            f"zero_ts={a.zero_point_ts} ts_span={a.ts_span_ms}ms "
            f"second_imu={a.second_imu_publisher}"
        )

    # ---------------------------------------------------------------- cloud
    def tick_cloud(self):
        a = self.a
        now = time.time()
        n = a.points
        arr = np.zeros(n, dtype=NP_DTYPE)
        ang = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        r = 5.0 + 0.5 * np.sin(4.0 * ang)
        z = np.linspace(-1.0, 1.0, n)
        if a.range_noise_std > 0.0:
            # Per-point range noise. WHY IT MATTERS: without it this scene is
            # geometrically exact, so the plane fits in h_share_model() have a
            # zero residual and the lidar pins the pose no matter what the IMU
            # covariances say. That makes the whole pose-stability fault class --
            # and any A/B of acc_cov / gyr_cov / b_acc_cov -- unreproducible
            # here, because those only matter when the lidar constraint is weak.
            # Set it to the mean residual the perf report shows for your real
            # scene (section 4, "fit: mean residual") to get a comparable test.
            r = r + self.cloud_rng.normal(0.0, a.range_noise_std, n)
            z = z + self.cloud_rng.normal(0.0, a.range_noise_std, n)
        arr["x"] = (r * np.cos(ang)).astype(np.float32)
        arr["y"] = (r * np.sin(ang)).astype(np.float32)
        arr["z"] = z.astype(np.float32)
        arr["intensity"] = 100.0
        arr["line"] = (np.arange(n) % 4).astype(np.uint8)

        header_ns = now * 1e9
        if a.zero_point_ts:
            arr["timestamp"] = 0.0
        else:
            span_ms = a.ts_span_ms if a.ts_span_ms > 0 else (1000.0 / a.cloud_rate)
            arr["timestamp"] = header_ns + np.linspace(0.0, span_ms * 1e6, n)

        msg = PointCloud2()
        msg.header.stamp = sec_to_stamp(now)
        msg.header.frame_id = a.frame
        msg.height = 1
        msg.width = n
        msg.fields = FIELDS
        msg.is_bigendian = False
        msg.point_step = POINT_STEP
        msg.row_step = POINT_STEP * n
        msg.is_dense = True
        msg.data = arr.tobytes()
        self.pub_cloud.publish(msg)
        self.cloud_n += 1

    # ------------------------------------------------------------------ imu
    def _imu_msg(self, t, phase=0.0):
        a = self.a
        m = Imu()
        m.header.stamp = sec_to_stamp(t)
        m.header.frame_id = a.frame
        if a.imu_still:
            # Stationary: gravity on z plus white noise. Nothing to average out
            # except the noise, so this is the case where a gravity/bias init is
            # meaningful and where "pose wanders while still" means something.
            ax = self.rng.gauss(0.0, self.acc_sd[0])
            ay = self.rng.gauss(0.0, self.acc_sd[1])
            az = 9.81 + self.rng.gauss(0.0, self.acc_sd[2])
            # A couple of bad samples early on, on ONE axis: that is what makes a
            # per-axis acc std lopsided without any real vibration.
            # Spread the remaining budget over the remaining window, so
            # --acc-spike-count is actually delivered whatever the window size.
            # That makes one mechanism cover both faults: a few large spikes is
            # "bad samples", many small ones is "this axis is vibrating".
            left_s = a.acc_spike_window - (t - self.t0)
            left_n = max(1.0, left_s * a.imu_rate)
            if (self.acc_spikes_left > 0 and left_s > 0.0
                    and self.rng.random() < self.acc_spikes_left / left_n):
                self.acc_spikes_left -= 1
                if a.acc_spike_axis == "x":
                    ax += a.acc_spike_mag
                elif a.acc_spike_axis == "y":
                    ay += a.acc_spike_mag
                else:
                    az += a.acc_spike_mag
            m.linear_acceleration.x = ax
            m.linear_acceleration.y = ay
            m.linear_acceleration.z = az
            m.angular_velocity.x = self.rng.gauss(0.0, self.gyr_sd[0])
            m.angular_velocity.y = self.rng.gauss(0.0, self.gyr_sd[1])
            m.angular_velocity.z = self.rng.gauss(0.0, self.gyr_sd[2])
            return m
        w = 2.0 * math.pi * 0.2 * t + phase
        m.linear_acceleration.x = 0.20 * math.sin(w)
        m.linear_acceleration.y = 0.20 * math.cos(w)
        m.linear_acceleration.z = 9.81
        m.angular_velocity.x = 0.02 * math.sin(w)
        m.angular_velocity.y = 0.02 * math.cos(w)
        m.angular_velocity.z = 0.05
        return m

    def tick_imu(self):
        a = self.a
        now = time.time()
        el = now - self.t0

        if a.imu_stall_every > 0:
            # stall for imu_stall_ms at the top of each imu_stall_every window
            if (el % a.imu_stall_every) < (a.imu_stall_ms / 1000.0):
                self.dropped += 1
                return
        if a.drop_imu_frac > 0.0 and self.rng.random() < a.drop_imu_frac:
            self.dropped += 1
            return

        self.pub_imu.publish(self._imu_msg(now))
        self.imu_n += 1

        if a.second_imu_publisher:
            # A 2nd sensor on the SAME topic, clock-offset by --second-imu-offset-ms.
            # Interleaved, out-of-order stamps -> imu_cbk() sees regressions and
            # calls imu_buffer.clear() over and over.
            self.pub_imu.publish(
                self._imu_msg(now - a.second_imu_offset_ms / 1000.0, phase=1.7)
            )
            self.imu_n += 1

    def status(self):
        self.get_logger().info(
            f"published clouds={self.cloud_n} imu={self.imu_n} "
            f"imu_dropped_on_purpose={self.dropped}"
        )


def main():
    p = argparse.ArgumentParser(
        description="Synthetic Livox publisher with fault injection",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    p.add_argument("--ns", default="/livox")
    p.add_argument("--frame", default="livox_frame")
    p.add_argument("--cloud-rate", type=float, default=10.0)
    p.add_argument("--imu-rate", type=float, default=200.0)
    p.add_argument("--points", type=int, default=20000,
                   help="points per cloud (Mid-360 ~20k @10Hz; use 40000 for 2x)")
    p.add_argument("--seed", type=int, default=0)
    # fault injection
    p.add_argument("--drop-imu-frac", type=float, default=0.0)
    p.add_argument("--imu-stall-every", type=float, default=0.0)
    p.add_argument("--imu-stall-ms", type=float, default=300.0)
    p.add_argument("--zero-point-ts", action="store_true")
    p.add_argument("--ts-span-ms", type=float, default=0.0)
    p.add_argument("--second-imu-publisher", action="store_true")
    p.add_argument("--second-imu-offset-ms", type=float, default=7.0)
    p.add_argument("--imu-still", action="store_true",
                   help="stationary IMU (gravity + noise) instead of the sinusoid")
    p.add_argument("--acc-noise-std", default="0.01",
                   help="accelerometer noise std, m/s^2 (--imu-still). One value "
                        "for all axes, or x,y,z -- an uneven triple is how a "
                        "single resonating axis looks, as opposed to broadband "
                        "vibration or a few bad samples")
    p.add_argument("--gyr-noise-std", default="0.0007",
                   help="gyro noise std, rad/s (--imu-still). One value or x,y,z")
    p.add_argument("--acc-spike-count", type=int, default=0)
    p.add_argument("--acc-spike-mag", type=float, default=1.0)
    p.add_argument("--acc-spike-axis", choices=["x", "y", "z"], default="y")
    p.add_argument("--acc-spike-window", type=float, default=2.0)
    p.add_argument("--range-noise-std", type=float, default=0.0,
                   help="per-point lidar range noise, metres. 0 = geometrically "
                        "exact scene (the default, and useless for judging pose "
                        "stability -- see the note in tick_cloud)")
    p.add_argument("--duration", type=float, default=0.0)
    a = p.parse_args()

    rclpy.init()
    node = FakeLivox(a)
    if a.duration > 0:
        node.create_timer(a.duration, lambda: (_ for _ in ()).throw(KeyboardInterrupt))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
