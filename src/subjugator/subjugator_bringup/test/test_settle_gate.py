"""Tests for the convergence-gated settle in `pool_tests/sim_bringup.py`.

`sim_bringup.py` is an installed script, not an importable module, so it is
loaded by path. Its module body must stay side-effect free (functions plus
constants; `main()` guarded by `__name__ == "__main__"`) or these tests would
run a real bring-up.
"""

import argparse
import importlib.util
import re
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
from nav_msgs.msg import Odometry

SCRIPT = Path(__file__).resolve().parents[1] / "pool_tests" / "sim_bringup.py"

# The exact regex run_task uses to parse the settle line out of the log.
SETTLE_RE = re.compile(r"settle: (converged|capped) after ([\d.]+) sim-s")


def load_module():
    """Import sim_bringup.py by path; it is an installed script, not a module."""
    spec = importlib.util.spec_from_file_location("sim_bringup", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def m():
    return load_module()


# --------------------------------------------------------------------------
# settled()
# --------------------------------------------------------------------------


def test_settled_requires_enough_samples(m):
    assert m.settled([0.001, 0.001], tol=0.02, need=5) is False


def test_settled_when_recent_samples_are_all_small(m):
    assert m.settled([0.5, 0.3, 0.001, 0.002, 0.001], tol=0.02, need=3) is True


def test_not_settled_when_a_recent_sample_is_large(m):
    assert m.settled([0.001, 0.5, 0.001], tol=0.02, need=3) is False


def test_only_the_most_recent_window_matters(m):
    assert m.settled([9.0, 0.001, 0.001], tol=0.02, need=2) is True


def test_settled_is_false_on_no_samples(m):
    assert m.settled([], tol=0.02, need=10) is False


def test_settled_exactly_enough_samples(m):
    """`need` samples is enough; `need - 1` is not."""
    assert m.settled([0.001] * 3, tol=0.02, need=3) is True
    assert m.settled([0.001] * 2, tol=0.02, need=3) is False


def test_settled_boundary_is_strict(m):
    """A sample exactly at the tolerance is not below it."""
    assert m.settled([0.02, 0.02], tol=0.02, need=2) is False
    assert m.settled([0.0199, 0.0199], tol=0.02, need=2) is True


def test_settled_rejects_a_large_sample_anywhere_in_the_window(m):
    """Every position in the window is checked, not just the ends."""
    for i in range(4):
        window = [0.001] * 4
        window[i] = 0.9
        assert m.settled(window, tol=0.02, need=4) is False, f"missed index {i}"


def test_settled_ignores_old_large_samples(m):
    assert m.settled([5.0, 5.0, 5.0, 0.001, 0.001], tol=0.02, need=2) is True


def test_settled_with_need_one(m):
    assert m.settled([9.0, 0.001], tol=0.02, need=1) is True
    assert m.settled([0.001, 9.0], tol=0.02, need=1) is False


def test_settled_returns_a_bool(m):
    assert isinstance(m.settled([0.001, 0.001], tol=0.02, need=2), bool)
    assert isinstance(m.settled([0.001], tol=0.02, need=2), bool)


def test_settled_does_not_mutate_its_input(m):
    mags = [0.5, 0.001, 0.001]
    m.settled(mags, tol=0.02, need=2)
    assert mags == [0.5, 0.001, 0.001]


def test_settled_honours_tolerance(m):
    """The same samples settle under a loose tol and not under a tight one."""
    mags = [0.05, 0.05, 0.05]
    assert m.settled(mags, tol=0.1, need=3) is True
    assert m.settled(mags, tol=0.01, need=3) is False


# --------------------------------------------------------------------------
# Bringup.twist_magnitude() / Bringup.sim_now()
# --------------------------------------------------------------------------


def odom_with(**components):
    o = Odometry()
    for name, value in components.items():
        group, axis = name.split("_")
        setattr(getattr(o.twist.twist, group), axis, value)
    return o


def test_twist_magnitude_none_without_odometry(m):
    assert m.Bringup.twist_magnitude(SimpleNamespace(odom=None)) is None


def test_twist_magnitude_zero_when_still(m):
    node = SimpleNamespace(odom=Odometry())
    assert m.Bringup.twist_magnitude(node) == 0.0


@pytest.mark.parametrize(
    "component",
    [
        "linear_x",
        "linear_y",
        "linear_z",
        "angular_x",
        "angular_y",
        "angular_z",
    ],
)
def test_twist_magnitude_sees_every_component(m, component):
    """Each of the six velocity components can dominate the magnitude."""
    node = SimpleNamespace(odom=odom_with(**{component: 0.75}))
    assert m.Bringup.twist_magnitude(node) == 0.75


@pytest.mark.parametrize(
    "component",
    [
        "linear_x",
        "linear_y",
        "linear_z",
        "angular_x",
        "angular_y",
        "angular_z",
    ],
)
def test_twist_magnitude_uses_absolute_value(m, component):
    node = SimpleNamespace(odom=odom_with(**{component: -0.75}))
    assert m.Bringup.twist_magnitude(node) == 0.75


def test_twist_magnitude_takes_the_largest_component(m):
    node = SimpleNamespace(
        odom=odom_with(linear_x=0.1, linear_y=-0.4, angular_z=0.3),
    )
    assert m.Bringup.twist_magnitude(node) == pytest.approx(0.4)


def test_odom_stamp_none_without_odometry(m):
    assert m.Bringup.odom_stamp(SimpleNamespace(odom=None)) is None


def test_odom_stamp_reads_the_message_header(m):
    o = Odometry()
    o.header.stamp.sec = 12
    o.header.stamp.nanosec = 340_000_000
    assert m.Bringup.odom_stamp(SimpleNamespace(odom=o)) == (12, 340_000_000)


def test_odom_stamp_distinguishes_sub_second_messages(m):
    """Odometry runs well above 1 Hz, so nanosec must be part of the identity."""
    first, second = Odometry(), Odometry()
    first.header.stamp.sec = second.header.stamp.sec = 3
    first.header.stamp.nanosec = 100_000_000
    second.header.stamp.nanosec = 133_000_000
    assert m.Bringup.odom_stamp(SimpleNamespace(odom=first)) != m.Bringup.odom_stamp(
        SimpleNamespace(odom=second),
    )


def test_sim_now_converts_nanoseconds_to_seconds(m):
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_500_000_000))
    node = SimpleNamespace(get_clock=lambda: clock)
    assert m.Bringup.sim_now(node) == pytest.approx(1.5)


# --------------------------------------------------------------------------
# main(): the settle loop
# --------------------------------------------------------------------------


def fake_bringup(
    made,
    mags,
    sim_step=0.5,
    odom=None,
    odom_period=0.01,
    stamp_always=False,
):
    """A stand-in for `Bringup` whose sim clock only moves when spun.

    `mags` is a list of twist magnitudes handed out one per loop iteration; the
    last entry repeats once exhausted. A `None` entry means "no odometry yet".

    `sim_step` is the real-time factor: `spin_for(s)` advances the sim clock by
    `sim_step * s`. `odom_period` is how far the sim clock must advance before a
    NEW odometry message exists, so a low `sim_step` reproduces the real failure
    mode -- polling faster than odometry is published and reading one message
    many times over.

    `stamp_always` decouples the stamp from the magnitude, so a test can hand
    back a fresh stamp with no usable twist -- the case the loop's
    `mag is not None` guard exists for.
    """

    class FakeBringup:
        def __init__(self):
            self.sim_t = 0.0
            self.sim_step = sim_step
            self.pending = list(mags)
            self.odom = odom
            self.goals = []
            self.set_poses = []
            self.calls = []
            self.spins = []
            self.stamps = []
            self.destroyed = False
            made.append(self)

        def odom_stamp(self):
            if not stamp_always and (not self.pending or self.pending[0] is None):
                self.stamps.append(None)
                return None
            stamp = (int(self.sim_t / odom_period), 0)
            self.stamps.append(stamp)
            return stamp

        def spin_for(self, s):
            self.spins.append(s)
            self.sim_t += self.sim_step * s

        def call(self, T, name, req, t=10.0):
            self.calls.append(name)

        def set_pose(self, x, y, z):
            self.set_poses.append((x, y, z))

        def goal(self, x, y, z):
            self.goals.append((x, y, z))

        def twist_magnitude(self):
            if len(self.pending) > 1:
                return self.pending.pop(0)
            return self.pending[0] if self.pending else None

        def sim_now(self):
            return self.sim_t

        def destroy_node(self):
            self.destroyed = True

    return FakeBringup


def run_main(
    m,
    monkeypatch,
    argv,
    mags,
    sim_step=0.5,
    odom=None,
    odom_period=0.01,
    stamp_always=False,
):
    """Run `main()` against a fake node and return (rc, node, stdout)."""
    made = []
    monkeypatch.setattr(
        m,
        "Bringup",
        fake_bringup(made, mags, sim_step, odom, odom_period, stamp_always),
    )
    monkeypatch.setattr(
        m,
        "rclpy",
        SimpleNamespace(init=lambda: None, shutdown=lambda: None),
    )
    monkeypatch.setattr(m, "gz_unpause", lambda: None)
    monkeypatch.setattr(m, "gz_teleport", lambda x, y, z: None)
    monkeypatch.setattr(m, "gz_truth_xyz", lambda: (1.0, 2.0, -0.35))
    monkeypatch.setattr(sys, "argv", ["sim_bringup.py", *argv])
    rc = m.main()
    return rc, made[0], made


BASE_ARGS = ["--x", "1.0", "--y", "2.0"]


def parse_settle(out):
    lines = [ln for ln in out.splitlines() if ln.startswith("settle:")]
    assert len(lines) == 1, f"expected exactly one settle line, got {lines}"
    match = SETTLE_RE.fullmatch(lines[0])
    assert match, f"line does not match run_task's regex: {lines[0]!r}"
    return match.group(1), float(match.group(2))


def test_settle_gate_converges(m, monkeypatch, capsys):
    rc, node, _ = run_main(
        m,
        monkeypatch,
        [*BASE_ARGS, "--settle-until-still", "--stable-samples", "4"],
        mags=[0.001],
    )
    out = capsys.readouterr().out
    assert rc == 0
    state, secs = parse_settle(out)
    assert state == "converged"
    # 4 iterations of spin_for(0.5) at 0.5x sim rate = 1.0 sim-s.
    assert secs == pytest.approx(1.0, abs=0.01)
    assert node.destroyed is True


def test_settle_gate_waits_out_early_motion(m, monkeypatch, capsys):
    """Large magnitudes up front delay convergence but do not prevent it."""
    run_main(
        m,
        monkeypatch,
        [*BASE_ARGS, "--settle-until-still", "--stable-samples", "3"],
        mags=[0.9, 0.9, 0.9, 0.001],
    )
    state, secs = parse_settle(capsys.readouterr().out)
    assert state == "converged"
    # 3 noisy samples then 3 quiet ones = 6 iterations x 0.25 sim-s.
    assert secs == pytest.approx(1.5, abs=0.01)


def test_settle_gate_caps_when_never_still(m, monkeypatch, capsys):
    run_main(
        m,
        monkeypatch,
        [
            *BASE_ARGS,
            "--settle-until-still",
            "--stable-samples",
            "3",
            "--settle-cap-sim",
            "2.0",
        ],
        mags=[5.0],
    )
    state, secs = parse_settle(capsys.readouterr().out)
    assert state == "capped"
    assert secs >= 2.0


def test_settle_gate_cap_is_inclusive(m, monkeypatch, capsys):
    """Hitting the cap exactly stops the loop rather than overshooting."""
    run_main(
        m,
        monkeypatch,
        [
            *BASE_ARGS,
            "--settle-until-still",
            "--stable-samples",
            "3",
            "--settle-cap-sim",
            "2.0",
        ],
        mags=[5.0],
        sim_step=1.0,  # 0.5 sim-s per iteration
    )
    state, secs = parse_settle(capsys.readouterr().out)
    assert state == "capped"
    assert secs == pytest.approx(2.0, abs=0.01)


def test_settle_gate_caps_in_sim_seconds_not_wall_seconds(m, monkeypatch, capsys):
    """A slow sim gets the same amount of *sim* settling, just more wall time."""
    for rtf in (0.1, 1.0):
        run_main(
            m,
            monkeypatch,
            [
                *BASE_ARGS,
                "--settle-until-still",
                "--stable-samples",
                "3",
                "--settle-cap-sim",
                "3.0",
            ],
            mags=[5.0],
            sim_step=rtf,
        )
        state, secs = parse_settle(capsys.readouterr().out)
        assert state == "capped"
        assert secs == pytest.approx(3.0, abs=0.3), f"rtf={rtf}"


def test_settle_gate_survives_missing_odometry(m, monkeypatch, capsys):
    """`twist_magnitude()` returning None must not crash or falsely converge."""
    rc, _, _ = run_main(
        m,
        monkeypatch,
        [
            *BASE_ARGS,
            "--settle-until-still",
            "--stable-samples",
            "2",
            "--settle-cap-sim",
            "2.0",
        ],
        mags=[None],
    )
    out = capsys.readouterr().out
    state, _secs = parse_settle(out)
    assert rc == 0
    assert state == "capped"
    # The sim itself is healthy -- only odometry stopped -- so this is a cap,
    # not a stall.
    assert "not advancing" not in out


def test_a_missing_twist_is_never_recorded_as_a_still_sample(m, monkeypatch, capsys):
    """A fresh stamp with no usable twist must not count as "still".

    Treating an absent measurement as zero velocity would converge the gate on
    a sub nobody has measured -- the same failure the whole ordering guards
    against, arrived at from the other side.
    """
    _rc, node, _ = run_main(
        m,
        monkeypatch,
        [
            *BASE_ARGS,
            "--settle-until-still",
            "--stable-samples",
            "2",
            "--settle-cap-sim",
            "2.0",
        ],
        mags=[None],
        stamp_always=True,  # stamps keep advancing; the twist never arrives
    )
    state, _secs = parse_settle(capsys.readouterr().out)
    assert state == "capped"
    assert len(consecutive_distinct(node.stamps)) > 2, "stamps were not advancing"


def test_stall_guard_fires_when_odometry_and_the_clock_both_stop(
    m,
    monkeypatch,
    capsys,
):
    """A dead sim is caught by the wall guard, not by riding out the sim cap.

    With no odometry the sample list never grows, and with a frozen clock the
    sim-second cap is unreachable, so without the guard this loop never ends.
    """
    monkeypatch.setattr(m, "SETTLE_STALL_WALL_S", 0.0)
    rc, node, _ = run_main(
        m,
        monkeypatch,
        [*BASE_ARGS, "--settle-until-still", "--settle-cap-sim", "40.0"],
        mags=[None],
        sim_step=0.0,
    )
    out = capsys.readouterr().out
    state, secs = parse_settle(out)
    assert rc == 0
    assert state == "capped"
    assert secs == pytest.approx(0.0)
    assert "not advancing" in out
    # It gave up promptly rather than polling out a 40 sim-second cap.
    assert len(node.stamps) == 1


def consecutive_distinct(seq):
    """Collapse runs of equal values -- how many distinct messages were seen."""
    out = []
    for s in seq:
        if not out or s != out[-1]:
            out.append(s)
    return out


def test_repeated_odometry_stamps_do_not_count_toward_convergence(
    m,
    monkeypatch,
    capsys,
):
    """Ten reads of one message are one sample, not ten.

    `spin_for` waits in WALL time while odometry is stamped in SIM time, so a
    slow sim hands the same message back on every poll. Counting those would let
    the gate converge on a single instant -- the exact RTF-dependence this
    feature removes.
    """
    _rc, node, _ = run_main(
        m,
        monkeypatch,
        [
            *BASE_ARGS,
            "--settle-until-still",
            "--stable-samples",
            "3",
            "--settle-cap-sim",
            "5.0",
        ],
        mags=[0.001],  # dead still, but only ever one message
        sim_step=0.5,
        odom_period=10.0,  # one message per 10 sim-s: none of them are new
    )
    state, _secs = parse_settle(capsys.readouterr().out)
    assert state == "capped"
    assert len(node.stamps) > 3, "the gate polled fewer times than it sampled"
    assert len(consecutive_distinct(node.stamps)) == 1


def test_convergence_needs_the_same_messages_at_any_rtf(m, monkeypatch, capsys):
    """The gate is RTF-independent: 4 distinct messages at 0.05x and at 1.0x."""
    distinct = {}
    polls = {}
    for rtf in (0.05, 1.0):
        _rc, node, _ = run_main(
            m,
            monkeypatch,
            [
                *BASE_ARGS,
                "--settle-until-still",
                "--stable-samples",
                "4",
                "--settle-cap-sim",
                "100.0",
            ],
            mags=[0.001],
            sim_step=rtf,
            odom_period=0.1,
        )
        state, _secs = parse_settle(capsys.readouterr().out)
        assert state == "converged", f"rtf={rtf}"
        distinct[rtf] = len(consecutive_distinct(node.stamps))
        polls[rtf] = len(node.stamps)
    assert distinct[0.05] == distinct[1.0] == 4
    # The slow run needed many more polls to see those same 4 messages -- which
    # is precisely what a poll-counting gate would have got wrong.
    assert polls[0.05] > polls[1.0]


def test_convergence_wins_a_tie_with_the_cap(m, monkeypatch, capsys):
    """When both fire on the same iteration, a settled sub is not called capped."""
    run_main(
        m,
        monkeypatch,
        [
            *BASE_ARGS,
            "--settle-until-still",
            "--stable-samples",
            "2",
            "--settle-cap-sim",
            "0.5",  # exactly 2 iterations of 0.25 sim-s
        ],
        mags=[0.001],
    )
    state, secs = parse_settle(capsys.readouterr().out)
    assert state == "converged"
    assert secs == pytest.approx(0.5, abs=0.01)


def test_stall_guard_ignores_a_slow_but_advancing_clock(m, monkeypatch, capsys):
    """A crawling sim is not a dead sim: the guard must check clock progress."""
    monkeypatch.setattr(m, "SETTLE_STALL_WALL_S", 0.0)
    run_main(
        m,
        monkeypatch,
        [
            *BASE_ARGS,
            "--settle-until-still",
            "--stable-samples",
            "3",
            "--settle-cap-sim",
            "0.1",
        ],
        mags=[5.0],
        sim_step=0.02,  # 0.01 sim-s per iteration -- slow, but never frozen
    )
    out = capsys.readouterr().out
    assert "not advancing" not in out, "stall guard fired on a live clock"
    state, secs = parse_settle(out)
    assert state == "capped"
    assert secs == pytest.approx(0.1, abs=0.01)


def test_stall_guard_wall_timeout_is_finite_and_sane(m):
    """The guard is a deadlock backstop, not a wall cap: minutes, not never."""
    assert 10.0 <= m.SETTLE_STALL_WALL_S <= 300.0


def test_settle_gate_gives_up_on_a_dead_sim_clock(m, monkeypatch, capsys):
    """A /clock that never advances must not hang the loop forever."""
    monkeypatch.setattr(m, "SETTLE_STALL_WALL_S", 0.0)
    rc, _, _ = run_main(
        m,
        monkeypatch,
        [
            *BASE_ARGS,
            "--settle-until-still",
            "--stable-samples",
            "5",
            "--settle-cap-sim",
            "40.0",
        ],
        mags=[5.0],
        sim_step=0.0,  # clock frozen
    )
    out = capsys.readouterr().out
    state, secs = parse_settle(out)
    assert rc == 0
    assert state == "capped"
    assert secs == pytest.approx(0.0)
    assert "not advancing" in out


def test_default_twist_tol_is_two_centimetres_per_second(m, monkeypatch, capsys):
    """Defaults must match `task_runner.tasks.Settle`, which documents them."""
    run_main(m, monkeypatch, [*BASE_ARGS, "--settle-until-still"], mags=[0.019])
    assert parse_settle(capsys.readouterr().out)[0] == "converged"
    # The cap must leave room for the default 10 samples to accumulate, or this
    # would cap on time regardless of the tolerance and prove nothing.
    run_main(
        m,
        monkeypatch,
        [*BASE_ARGS, "--settle-until-still", "--settle-cap-sim", "10.0"],
        mags=[0.021],
    )
    assert parse_settle(capsys.readouterr().out)[0] == "capped"


def test_default_stable_samples_is_ten(m, monkeypatch, capsys):
    run_main(m, monkeypatch, [*BASE_ARGS, "--settle-until-still"], mags=[0.001])
    state, secs = parse_settle(capsys.readouterr().out)
    assert state == "converged"
    assert secs == pytest.approx(10 * 0.25, abs=0.01)


def test_default_cap_is_forty_sim_seconds(m, monkeypatch, capsys):
    run_main(
        m,
        monkeypatch,
        [*BASE_ARGS, "--settle-until-still"],
        mags=[5.0],
        sim_step=8.0,  # 4 sim-s per iteration, so the cap lands exactly on 40
    )
    state, secs = parse_settle(capsys.readouterr().out)
    assert state == "capped"
    assert secs == pytest.approx(40.0, abs=0.01)


def test_settle_gate_holds_the_goal_every_iteration(m, monkeypatch):
    _rc, node, _ = run_main(
        m,
        monkeypatch,
        [*BASE_ARGS, "--settle-until-still", "--stable-samples", "4"],
        mags=[0.001],
    )
    # step 4 publishes the hold goal once, then every loop iteration republishes.
    assert node.goals.count((1.0, 2.0, -0.35)) >= 5


# --------------------------------------------------------------------------
# main(): the default (unflagged) path must be byte-for-byte the old behaviour
# --------------------------------------------------------------------------


def test_default_path_does_not_print_a_settle_line(m, monkeypatch, capsys):
    run_main(m, monkeypatch, [*BASE_ARGS, "--settle", "3"], mags=[5.0])
    out = capsys.readouterr().out
    assert "settle:" not in out
    assert "hold 3s so the sub goes dead still" in out


@pytest.mark.parametrize("settle", [2, 4, 7])
def test_default_path_spins_one_second_per_settle_step(m, monkeypatch, settle):
    _rc, node, _ = run_main(
        m,
        monkeypatch,
        [*BASE_ARGS, "--settle", str(settle)],
        mags=[5.0],
    )
    # Two 1-second spins precede the loop (post-set_pose, post-goal), so the
    # 1-second spin count scales as 2 + settle.
    assert node.spins.count(1.0) == settle + 2
    assert node.goals.count((1.0, 2.0, -0.35)) == settle + 1  # 1 pre-goal + loop


def test_default_settle_is_25_seconds(m, monkeypatch, capsys):
    _rc, node, _ = run_main(m, monkeypatch, BASE_ARGS, mags=[5.0])
    assert "hold 25s so the sub goes dead still" in capsys.readouterr().out
    assert node.spins.count(1.0) == 27


def test_default_path_ignores_gate_tuning_flags(m, monkeypatch, capsys):
    """Setting the gate knobs without --settle-until-still changes nothing."""
    _rc, node, _ = run_main(
        m,
        monkeypatch,
        [*BASE_ARGS, "--settle", "2", "--twist-tol", "9.0", "--stable-samples", "1"],
        mags=[5.0],
    )
    assert "settle:" not in capsys.readouterr().out
    assert node.spins.count(1.0) == 4


# --------------------------------------------------------------------------
# CLI surface
# --------------------------------------------------------------------------


def test_help_lists_the_new_flags_and_keeps_the_settle_default(m, monkeypatch, capsys):
    monkeypatch.setattr(sys, "argv", ["sim_bringup.py", "--help"])
    with pytest.raises(SystemExit) as exc:
        m.main()
    assert exc.value.code == 0
    # argparse wraps help text at the terminal width, which varies between a
    # tty and CI; collapse whitespace so the assertions do not depend on it.
    out = " ".join(capsys.readouterr().out.split())
    for flag in (
        "--settle-until-still",
        "--twist-tol",
        "--stable-samples",
        "--settle-cap-sim",
    ):
        assert flag in out, f"{flag} missing from --help"
    assert "--settle SETTLE" in out, "--settle disappeared from --help"
    assert "default: 25" in out, "--settle default of 25 not shown in --help"
    # --stable-samples counts odometry messages, not polls; say so, or a reader
    # will size it as a wall-cadence poll count.
    assert "MESSAGES" in out, "--stable-samples help does not say messages"


@pytest.mark.parametrize("bad", ["0", "-1", "-10"])
def test_stable_samples_below_one_is_rejected(m, monkeypatch, capsys, bad):
    """0 would make `settled()` ask if all of an empty window is small: True."""
    monkeypatch.setattr(
        sys,
        "argv",
        ["sim_bringup.py", *BASE_ARGS, "--settle-until-still", "--stable-samples", bad],
    )
    with pytest.raises(SystemExit) as exc:
        m.main()
    assert exc.value.code == 2
    assert "at least 1" in capsys.readouterr().err


def test_stable_samples_of_one_is_still_allowed(m, monkeypatch, capsys):
    run_main(
        m,
        monkeypatch,
        [*BASE_ARGS, "--settle-until-still", "--stable-samples", "1"],
        mags=[0.001],
    )
    assert parse_settle(capsys.readouterr().out)[0] == "converged"


@pytest.mark.parametrize("value", ["1", "10", "999"])
def test_positive_int_accepts_counts_of_one_or_more(m, value):
    assert m.positive_int(value) == int(value)


@pytest.mark.parametrize("value", ["0", "-1"])
def test_positive_int_rejects_zero_and_negatives(m, value):
    with pytest.raises(argparse.ArgumentTypeError, match="at least 1"):
        m.positive_int(value)


def test_positive_int_rejects_non_integers(m):
    with pytest.raises(ValueError, match="invalid literal"):
        m.positive_int("banana")


def test_module_body_has_no_side_effects(m):
    """Importing the script must not run a bring-up."""
    assert m.WORLD == "robosub_2025"
    assert callable(m.main)
