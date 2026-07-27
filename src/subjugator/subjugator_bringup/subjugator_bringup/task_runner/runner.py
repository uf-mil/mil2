"""The live half of the run: collect while orchestrating.

The orchestrator is itself a ROS node. Everything it collects is appended as
plain tuples and handed to the pure functions in collectors.py / probes.py at
the end, which is what keeps the analysis testable without a sim.

Two timing choices are load-bearing:

* **Collectors start before the mission does.** They are subscribed from the
  moment the readiness gates pass, so the settle and the whole mission window
  land in one continuous sample stream.
* **Ground truth is polled sparsely (2 s).** ``collectors.drift`` matches every
  odometry sample against its nearest-in-time truth sample, which is O(n*m):
  measured 0.63 s at 12000 odom x 600 truth, but 15 s at 12000 x 12000. A
  faster truth loop needs ``bisect`` in ``collectors._nearest`` first.
"""

from __future__ import annotations

import contextlib
import importlib
import os
import re
import signal
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

from . import btbudget, btlog, collectors, lifecycle, probes

# The gz world name is the launch argument minus its .world suffix.
DEFAULT_GZ_WORLD = "robosub_2025"

# The vehicle's model name in the world file. Tracked alongside the task's
# props because odom-vs-truth drift is what proves the EKF anchor held.
SUB_MODEL = "sub9"

TRUTH_POLL_S = 2.0

# A budget of zero would be exhausted before the first tick: budget_exhausted is
# correctly >=, and btbudget.declared_bound() legitimately returns 0.0 for a
# tree with no statically resolvable timeout.
MIN_BUDGET_S = 1.0


def gz_world_name(launch_args: dict) -> str:
    """'robosub_2025.world' -> 'robosub_2025'."""
    world = launch_args.get("world", "")
    if not world:
        return DEFAULT_GZ_WORLD
    return os.path.splitext(os.path.basename(world))[0]


class Collector(Node):
    """Subscribes for the whole run; holds raw samples, computes nothing."""

    def __init__(self, detection_topic: str, models: tuple, world: str):
        super().__init__("run_task_collector")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.world = world
        self.clock: list = []  # (wall_s, sim_s)
        self.odom: list = []  # (sim_s, x, y, z)
        self.detections: list = []  # (sim_s, [(label, conf), ...])
        self.models: dict = {m: [] for m in models}
        self.models.setdefault(SUB_MODEL, [])
        self.last_goal = None
        self.truth_polls = 0
        self.truth_hits = 0

        self.create_subscription(Clock, "/clock", self._on_clock, 50)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_odom, 20)
        self.create_subscription(Pose, "/goal_pose", self._on_goal, 10)
        self._sub_detections(detection_topic)

    def _sub_detections(self, topic: str) -> None:
        if not topic:
            return
        try:
            from yolo_msgs.msg import DetectionArray
        except ImportError:
            self.get_logger().warn("yolo_msgs unavailable; detections not collected")
            return
        self.create_subscription(DetectionArray, topic, self._on_detections, 20)

    def sim_now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_clock(self, msg) -> None:
        sim = msg.clock.sec + msg.clock.nanosec / 1e9
        self.clock.append((time.time(), sim))

    def _on_odom(self, msg) -> None:
        p = msg.pose.pose.position
        self.odom.append((self.sim_now(), p.x, p.y, p.z))

    def _on_goal(self, msg) -> None:
        self.last_goal = (msg.position.x, msg.position.y, msg.position.z)

    def _on_detections(self, msg) -> None:
        found = [(det.class_name, float(det.score)) for det in msg.detections]
        self.detections.append((self.sim_now(), found))

    def poll_truth(self) -> int:
        """One gz snapshot of every tracked model. Returns models matched.

        `pose/info`, not `dynamic_pose/info`: the latter carries only entities
        that moved this iteration, so a prop that was never touched would be
        reported absent rather than untouched -- which is precisely the case the
        ground-truth scorer exists to catch.
        """
        sim = self.sim_now()
        self.truth_polls += 1
        matched = select_poses(
            parse_gz_poses(gz_pose_snapshot(self.world)),
            self.models,
        )
        for name, point in matched.items():
            self.models[name].append((sim, *point))
        if matched:
            self.truth_hits += 1
        return len(matched)


def gz_control_services(world: str) -> int:
    """How many of the world's control services gz is advertising.

    This is the one thing observable before the settle. The sim launches
    PAUSED, so /clock does not advance, the cameras do not render, and
    robot_localization's ekf_node sits in "Waiting for clock to start..."
    without advertising /odometry/filtered at all. sim_bringup.py is what
    unpauses it, so any topic-level gate has to come after that.
    """
    out = subprocess.run(
        ["gz", "service", "-l"],
        capture_output=True,
        text=True,
        timeout=20,
        check=False,
    ).stdout
    return sum(1 for line in out.splitlines() if f"/world/{world}/control" in line)


def clock_advancing(clock: list) -> bool:
    """True once /clock has reported two different sim times.

    Message count is not enough: a paused gz republishes the same stamp, so a
    naive "have we seen two messages" gate passes against a frozen sim.
    """
    return len({sim for _wall, sim in clock}) > 1


def gz_pose_snapshot(world: str) -> str:
    """One message off the world's pose topic, as gz's text format."""
    return subprocess.run(
        ["gz", "topic", "-e", "-t", f"/world/{world}/pose/info", "-n", "1"],
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    ).stdout


def parse_gz_poses(text: str) -> list:
    """[(name, (x, y, z)), ...] from a gz Pose_V text dump.

    Same shape sim_bringup.py has been reading successfully; kept as a pure
    function so it can be exercised against a captured dump without a sim.
    """
    poses = []
    for block in re.findall(r"pose\s*\{(.*?)\n\}", text, re.S):
        name = re.search(r'name:\s*"([^"]+)"', block)
        position = re.search(r"position\s*\{([^}]*)\}", block)
        if not name or not position:
            continue
        values = dict(re.findall(r"(\w+):\s*(-?[\d.eE+-]+)", position.group(1)))
        poses.append(
            (name.group(1), tuple(float(values.get(k, 0.0)) for k in "xyz")),
        )
    return poses


def select_poses(poses: list, wanted) -> dict:
    """The first pose per wanted name in one snapshot.

    `pose/info` carries links as well as models, and a link often shares its
    model's name -- taking the first match keeps one sample per model per poll
    instead of silently double-weighting whichever models happen to have a
    same-named link.
    """
    picked: dict = {}
    for name, point in poses:
        if name in wanted and name not in picked:
            picked[name] = point
    return picked


def _spawn(cmd: list, log_path: str):
    """Run in its own session so a terminal Ctrl-C never reaches it."""
    handle = open(log_path, "w")
    return subprocess.Popen(
        cmd,
        stdout=handle,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def start_bag(run_dir: str, topics: list):
    """Optional raw capture, so a run stays re-analysable offline.

    SIGTERM, never SIGINT: a process backgrounded from a non-interactive parent
    inherits SIGINT=SIG_IGN, and `ros2 bag record` under rmw_zenoh ignores it
    anyway -- which is what made pooltest.sh appear to hang forever.
    """
    if not topics:
        return None
    return _spawn(
        ["ros2", "bag", "record", "-o", os.path.join(run_dir, "bag"), *topics],
        os.path.join(run_dir, "bag.log"),
    )


def _kill(proc) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except OSError:
        return
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        with contextlib.suppress(OSError):
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)


def wait_for(predicate, timeout_s: float, spin, period: float = 0.2) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        spin()
        if predicate():
            return True
        time.sleep(period)
    return False


def gate_hint(topic: str) -> str:
    if "detections" in topic:
        return (
            "  The down-cam YOLO node publishes nothing. On this box it needs the "
            "numpy-1 PYTHONPATH shadow; check bringup.log for an import error."
        )
    if "odometry" in topic:
        return (
            "  The EKF is not publishing. sim_bringup.py enables it -- "
            "see sim_bringup.log."
        )
    return ""


def parse_settle_log(path: str) -> dict:
    """Pull the machine-readable settle line out of sim_bringup's output."""
    result = {"converged": None, "sim_s": None, "offset": None}
    try:
        with open(path, errors="replace") as handle:
            text = handle.read()
    except OSError:
        return result
    match = re.search(r"settle: (converged|capped) after ([\d.]+) sim-s", text)
    if match:
        result["converged"] = match.group(1) == "converged"
        result["sim_s"] = float(match.group(2))
    offset = re.search(r"offset=([\d.]+) m", text)
    if offset:
        result["offset"] = float(offset.group(1))
    return result


def scan_health(path: str) -> dict:
    try:
        with open(path, errors="replace") as handle:
            text = handle.read()
    except OSError:
        return {"crashes": 0, "fatal": 0, "error": 0}
    return {
        "crashes": len(re.findall(r"terminate called|what\(\):", text)),
        "fatal": len(re.findall(r"\[FATAL\]", text)),
        "error": len(re.findall(r"\[ERROR\]", text)),
    }


def write_trace(path: str, node: Collector) -> None:
    with open(path, "w") as handle:
        handle.write("sim_s,source,x,y,z\n")
        for sim, x, y, z in node.odom:
            handle.write(f"{sim:.3f},odom,{x:.4f},{y:.4f},{z:.4f}\n")
        for name, samples in node.models.items():
            for sim, x, y, z in samples:
                handle.write(f"{sim:.3f},{name},{x:.4f},{y:.4f},{z:.4f}\n")


def mission_tree_dir() -> str:
    """Where the installed mission XML lives, for tree-ID lookup."""
    prefix = subprocess.run(
        ["ros2", "pkg", "prefix", "mission_planner"],
        capture_output=True,
        text=True,
        check=False,
    ).stdout.strip()
    return os.path.join(prefix, "share", "mission_planner", "bt")


def resolve_budget(args, stage) -> float:
    """The run's sim-second cap, floored so a zero can never end a run instantly."""
    return max(args.sim_budget or stage.sim_budget, MIN_BUDGET_S)


def execute(spec, stage_name, stage, start_name, args, run_dir: str) -> dict:
    """Run one mission end to end and return the assembled run record."""
    os.makedirs(run_dir, exist_ok=True)
    launch_cmd, settle_cmd, mission_cmd = lifecycle.plan_commands(
        spec,
        stage_name,
        stage,
        start_name,
        args,
    )
    budget = resolve_budget(args, stage)

    detection_topic = next(
        (p.topic for p in spec.probes if p.kind == "detections"),
        "",
    )
    models = next((p.models for p in spec.probes if p.kind == "model_pose"), ())

    rclpy.init()
    node = Collector(detection_topic, models, gz_world_name(spec.launch_args))

    def spin():
        rclpy.spin_once(node, timeout_sec=0.05)

    run = {
        "task": args.task,
        "task_name": spec.name,
        "stage": stage_name,
        "mission": stage.mission,
        "role": args.role,
        "sim_budget": budget,
        "run_dir": run_dir,
        "harness_error": None,
        "outcome": None,
        "stop_reason": None,
        "config": {
            "stage": stage_name,
            "start": start_name,
            "start_world": spec.world_start(start_name),
            "device": args.device,
            "role": args.role,
            "score_level": args.score_level,
            "sim_budget": budget,
            "stall_timeout": args.stall_timeout,
            "params": dict(stage.params),
            "commands": [launch_cmd, settle_cmd, mission_cmd],
            "git_sha": subprocess.run(
                ["git", "rev-parse", "--short", "HEAD"],
                capture_output=True,
                text=True,
                check=False,
            ).stdout.strip(),
        },
    }

    launch_proc = mission_proc = bag_proc = None
    truth_stop = threading.Event()

    def truth_loop():
        while not truth_stop.is_set():
            # A transient gz hiccup must never take the run down with it.
            with contextlib.suppress(Exception):
                node.poll_truth()
            truth_stop.wait(TRUTH_POLL_S)

    try:
        launch_proc = _spawn(launch_cmd, os.path.join(run_dir, "bringup.log"))

        # Pre-settle gate: the gz world exists. Nothing topic-level can be
        # gated here -- see gz_control_services on why a paused sim is silent.
        world = gz_world_name(spec.launch_args)
        if not wait_for(
            lambda: gz_control_services(world) > 0,
            180.0,
            spin,
            period=3.0,
        ):
            run["harness_error"] = (
                f"the gz world never came up: no /world/{world}/control service.\n"
                "  Check bringup.log -- gz sim failed to load the world."
            )
            return run

        settle_log = os.path.join(run_dir, "sim_bringup.log")
        settle_proc = _spawn(settle_cmd, settle_log)
        while settle_proc.poll() is None:
            spin()
            time.sleep(0.1)
        settle = parse_settle_log(settle_log)

        # Data gates: messages actually arriving. Deliberately AFTER the settle,
        # because sim_bringup.py is what unpauses gz and enables the EKF --
        # before it runs there is no sim time, no rendered camera frame and no
        # /odometry/filtered publisher, so gating on any of them can never pass.
        if not wait_for(lambda: clock_advancing(node.clock), 120.0, spin):
            run["harness_error"] = (
                "/clock never advanced: gz is still paused or wedged.\n"
                "  sim_bringup.log should show step [1] unpausing it."
            )
            return run

        collected = {"/odometry/filtered": node.odom, detection_topic: node.detections}
        for topic in spec.ready_topics:
            samples = collected.get(topic)
            if samples is None:
                continue
            if not wait_for(lambda s=samples: bool(s), 120.0, spin):
                run["harness_error"] = (
                    f"readiness gate failed: {topic} is advertised but sent no data\n"
                    f"{gate_hint(topic)}"
                )
                return run

        threading.Thread(target=truth_loop, daemon=True).start()

        if args.bag:
            bag_topics = ["/odometry/filtered", "/goal_pose"]
            if detection_topic:
                bag_topics.append(detection_topic)
            bag_proc = start_bag(run_dir, bag_topics)

        console = os.path.join(run_dir, "console.log")
        start_sim = node.clock[-1][1]
        start_wall = time.time()
        mission_proc = _spawn(mission_cmd, console)

        while True:
            spin()
            if mission_proc.poll() is not None:
                run["stop_reason"] = "mission exited normally"
                break
            if collectors.stalled(node.clock, time.time(), args.stall_timeout):
                run["harness_error"] = (
                    f"/clock stalled for over {args.stall_timeout:.0f} wall-s: gz is "
                    "wedged or paused"
                )
                run["stop_reason"] = "clock stalled"
                break
            if collectors.budget_exhausted(node.clock, start_sim, budget):
                run["stop_reason"] = f"sim budget exhausted ({budget:.0f} sim-s)"
                break
            time.sleep(0.05)

        truth_stop.set()
        node.poll_truth()

        trees = btbudget.load_trees(mission_tree_dir())
        parsed = btlog.parse_file(console, set(trees))
        run["outcome"] = parsed.outcome
        run["last_node"] = parsed.last_node
        run["innermost_node"] = parsed.innermost_node

        clock = node.clock
        run["rtf"] = collectors.rtf(clock)
        run["sim_duration"] = collectors.consumed_sim(clock, start_sim)
        run["wall_duration"] = time.time() - start_wall
        run["stages"] = [
            {
                "name": s.name,
                "occurrence": s.occurrence,
                "result": s.result,
                "sim_duration": round(
                    (collectors.wall_to_sim(clock, s.end_wall) or 0.0)
                    - (collectors.wall_to_sim(clock, s.start_wall) or 0.0),
                    1,
                ),
            }
            for s in parsed.stages
        ]

        # Vehicle stats cover the MISSION window only. Collection starts before
        # the settle, and robot_localization's estimate diverges to ~1e24 m
        # until sim_bringup re-anchors it -- one such sample makes path length
        # and drift meaningless. The full stream still goes to trace.csv.
        mission_odom = [s for s in node.odom if s[0] >= start_sim]
        truth = [s for s in node.models.get(SUB_MODEL, []) if s[0] >= start_sim]
        run["vehicle"] = {
            "path_length": collectors.path_length(mission_odom),
            "depth_range": collectors.depth_range(mission_odom),
            "goal_error": collectors.goal_error(node.last_goal, mission_odom),
            "anchor_offset": settle.get("offset"),
            "settle": settle,
            "drift": collectors.drift(mission_odom, truth),
            "odom_samples": len(mission_odom),
            "odom_samples_all": len(node.odom),
            "truth_samples": len(truth),
        }

        detections = probes.summarize_detections(node.detections)
        detections["topic"] = detection_topic
        run["perception"] = detections

        model_summary = probes.summarize_model_poses(
            {m: node.models.get(m, []) for m in models},
        )
        run["models"] = model_summary
        if spec.scorer:
            scorer = importlib.import_module(spec.scorer)
            verdict = scorer.score(args.role, model_summary, run["outcome"])
            if not stage.places:
                # A stage that never attempts a place cannot contradict the tree
                # by not placing anything. Keeping the flag on would fire the
                # report's loudest warning on every calib run and train people
                # to ignore it -- which is the one thing it must never become.
                verdict["disagrees_with_bt"] = False
                verdict["note"] = (
                    f"stage '{stage_name}' attempts no place; "
                    "object rows are informational"
                )
            run["ground_truth"] = verdict

        run["health"] = scan_health(console)
        write_trace(os.path.join(run_dir, "trace.csv"), node)
        return run

    finally:
        truth_stop.set()
        _kill(mission_proc)
        _kill(bag_proc)
        if args.keep_alive:
            print("--keep-alive: the sim stack is still running:")
            for stray in lifecycle.find_stale():
                print(f"  {stray.pid} {stray.name}")
        else:
            _kill(launch_proc)
            time.sleep(2.0)
            survivors = lifecycle.find_stale()
            if survivors:
                lifecycle.kill_stale(survivors)
                time.sleep(2.0)
                still = lifecycle.find_stale()
                if still:
                    # A wedged gz server ignores SIGTERM, and an observed
                    # teardown left four of them behind. Escalate rather than
                    # leave a stack running to poison the next run's numbers.
                    lifecycle.kill_stale(still, signal.SIGKILL)
                    time.sleep(2.0)
                    still = lifecycle.find_stale()
                if still:
                    print("WARNING: sim processes survived teardown:")
                    for stray in still:
                        print(f"  {stray.pid} {stray.name}")
        node.destroy_node()
        rclpy.shutdown()
