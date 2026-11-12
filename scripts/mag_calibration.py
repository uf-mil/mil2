#!/usr/bin/env python3
"""Hard and soft iron magnetometer calibration from /imu/mag.

Start it, then turn the boat through as many orientations as you can (roll it,
pitch it, spin it) until the capture ends. Prints the imu_filter_madgwick block
for prop_bringup/launch/infix_sensors.launch.py.

    python3 scripts/mag_calibration.py [--duration 30] [--save samples.csv]
    python3 scripts/mag_calibration.py --load samples.csv
"""

import argparse
import sys
import time

import numpy as np

MIN_DEPTH = 0.2
MAX_BIAS_SIGMA = 1e-6
MAX_SOFT_IRON_SIGMA = 0.02


def _fibonacci_sphere(n):
    i = np.arange(n) + 0.5
    phi = np.arccos(1 - 2 * i / n)
    theta = np.pi * (1 + 5**0.5) * i
    return np.column_stack(
        [np.cos(theta) * np.sin(phi), np.sin(theta) * np.sin(phi), np.cos(phi)],
    )


BINS = _fibonacci_sphere(100)


def coverage(p, center, A):
    """Fraction of the sphere of field directions the samples touched."""
    u = (p - center) @ A.T
    u /= np.linalg.norm(u, axis=1, keepdims=True)
    return len(np.unique(np.argmax(u @ BINS.T, axis=1))) / len(BINS)


def sphere_fit(p):
    A = np.hstack([2 * p, np.ones((len(p), 1))])
    x, *_ = np.linalg.lstsq(A, (p**2).sum(axis=1), rcond=None)
    return x[:3]


def ellipsoid_fit(p):
    """Center c and det-1 symmetric A so |A (p - c)| is constant, or None."""
    mu = p.mean(axis=0)
    sigma = np.linalg.norm(p - mu, axis=1).mean()
    x, y, z = ((p - mu) / sigma).T
    D = np.column_stack(
        [x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, -2 * x, -2 * y, -2 * z],
    )
    s, *_ = np.linalg.lstsq(D, np.ones(len(x)), rcond=None)
    Q = np.array([[s[0], s[3], s[4]], [s[3], s[1], s[5]], [s[4], s[5], s[2]]])
    w, V = np.linalg.eigh(Q)
    if w.min() <= 0:
        return None
    c = np.linalg.solve(Q, s[6:])
    A = V @ np.diag(np.sqrt(w)) @ V.T
    return mu + sigma * c, A / np.cbrt(np.linalg.det(A))


def depth(p):
    """Thinnest over widest axis of the sample cloud. A flat spin gives ~0.04."""
    w = np.linalg.eigvalsh(np.cov(p.T))
    return np.sqrt(max(w[0], 0) / w[-1])


def drop_outliers(p):
    p = p[np.isfinite(p).all(axis=1) & (np.linalg.norm(p, axis=1) > 0)]
    center, A = ellipsoid_fit(p) or (sphere_fit(p), np.eye(3))
    r = np.linalg.norm((p - center) @ A.T, axis=1)
    dev = np.abs(r - np.median(r))
    mad = 1.4826 * np.median(dev)
    return p[dev < 5 * mad] if mad > 0 else p


def bootstrap(p, n=20):
    """Spread of the fit over resamples: (bias sigma, worst soft iron entry sigma).

    A pure yaw spin fits the same wrong ellipsoid every time, so the bias barely
    moves; the soft iron matrix is what flags it.
    """
    rng = np.random.default_rng(0)
    fits = [ellipsoid_fit(p[rng.integers(0, len(p), len(p))]) for _ in range(n)]
    fits = [f for f in fits if f is not None]
    if len(fits) < 0.9 * n:
        return np.inf, np.inf
    bias_sigma = np.linalg.norm(np.std([f[0] for f in fits], axis=0))
    return bias_sigma, np.std([f[1] for f in fits], axis=0).max()


def magnitude_spread(p, center, A):
    r = np.linalg.norm((p - center) @ A.T, axis=1)
    return r.std() / r.mean(), r.mean()


def capture(topic, duration):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import MagneticField

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("mag_calibration")
    samples = []
    node.create_subscription(
        MagneticField,
        topic,
        lambda m: samples.append(
            (m.magnetic_field.x, m.magnetic_field.y, m.magnetic_field.z),
        ),
        qos_profile_sensor_data,
    )

    start = None
    try:
        print(f"waiting for {topic} ...")
        while not samples:
            rclpy.spin_once(node, timeout_sec=0.5)
        print(f"rotate the boat through every orientation for {duration:.0f} s")
        print("(ctrl-c stops early and fits what it has)")
        start = last = time.monotonic()
        while time.monotonic() - start < duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last >= 1.0 and len(samples) > 20:
                last = now
                p = np.array(samples)
                cov = coverage(p, sphere_fit(p), np.eye(3))
                d = depth(p)
                print(
                    f"\r  {now - start:4.0f} s  {len(p):6d} samples"
                    f"  coverage {100 * cov:3.0f}%  depth {d:.2f}"
                    + ("  ok        " if d >= MIN_DEPTH else "  tilt more!"),
                    end="",
                    flush=True,
                )
    except KeyboardInterrupt:
        pass
    print()
    elapsed = time.monotonic() - start if start else 0.0
    node.destroy_node()
    rclpy.try_shutdown()
    return np.array(samples), elapsed


def sci(v):
    m, e = f"{v:.4e}".split("e")
    return f"{m}e{int(e)}"


def report(p):
    uT = 1e6
    ell = ellipsoid_fit(p)
    if ell is None:
        bias, A = sphere_fit(p), np.eye(3)
        bias_sigma = soft_sigma = np.inf
    else:
        bias, A = ell
        bias_sigma, soft_sigma = bootstrap(p)
    d = depth(p)
    trusted = (
        d >= MIN_DEPTH
        and bias_sigma <= MAX_BIAS_SIGMA
        and soft_sigma <= MAX_SOFT_IRON_SIGMA
    )

    raw_spread, _ = magnitude_spread(p, np.zeros(3), np.eye(3))
    hard_spread, _ = magnitude_spread(p, bias, np.eye(3))
    soft_spread, field = magnitude_spread(p, bias, A)
    print(f"coverage        {100 * coverage(p, bias, A):.0f}% of the sphere")
    print(f"depth           {d:.2f} (want >= {MIN_DEPTH})")
    print(f"field strength  {field * uT:.1f} uT")
    print(
        f"magnitude spread  raw {100 * raw_spread:.1f}%"
        f"   bias removed {100 * hard_spread:.1f}%"
        f"   bias + soft iron {100 * soft_spread:.1f}%",
    )
    print(
        f"bias            {np.array2string(bias * uT, precision=2)} uT"
        f"  (+/- {bias_sigma * uT:.2f})",
    )

    print("\nimu_filter_madgwick parameters:\n")
    print(" " * 20 + "{")
    print(" " * 24 + '"publish_tf": False,')
    for axis, v in zip("xyz", bias):
        print(" " * 24 + f'"mag_bias_{axis}": {sci(v)},')
    print(" " * 20 + "},")

    if ell is not None:
        print(
            f"\nsoft iron (+/- {soft_sigma:.4f}), corrected = A @ (raw - bias)."
            " Madgwick only takes the bias.",
        )
        for row in A:
            print("    [" + ", ".join(f"{v: .5f}" for v in row) + "]")
        print("\nmagnetic_compensation hardsoft_compensator parameters:")
        print(f"    scale: {[round(v, 6) for v in np.linalg.inv(A).flatten()]}")
        print(f"    shift: {[float(sci(v)) for v in bias]}")

    if not trusted:
        print(
            "\nWARNING: this fit is not trustworthy. The boat probably only spun"
            " flat. Run it again and tilt it well over on every side as you turn"
            " it.",
        )
    return trusted


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--topic", default="/imu/mag")
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--save", help="write the raw samples to this CSV")
    parser.add_argument("--load", help="fit samples from a CSV instead of ROS")
    args = parser.parse_args()

    if args.load:
        p = np.loadtxt(args.load, delimiter=",", ndmin=2)
        print(f"{len(p)} samples from {args.load}")
    else:
        p, elapsed = capture(args.topic, args.duration)
        if len(p):
            print(f"{len(p)} samples over {elapsed:.1f} s ({len(p) / elapsed:.0f} Hz)")
        if args.save and len(p):
            np.savetxt(args.save, p, delimiter=",", header="x,y,z tesla")
            print(f"saved to {args.save}")

    if len(p) < 50:
        sys.exit(f"only {len(p)} samples, need at least 50")

    kept = drop_outliers(p)
    if len(kept) < len(p):
        print(f"dropped {len(p) - len(kept)} outliers")
    sys.exit(0 if report(kept) else 1)


if __name__ == "__main__":
    main()
