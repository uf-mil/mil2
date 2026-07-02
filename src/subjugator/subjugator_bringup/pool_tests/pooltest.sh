#!/usr/bin/env bash
# Guided poolside test runner for the Task 5 down-cam stack (S2 + S3).
# See docs/superpowers/specs/2026-06-25-pool-test-protocol-design.md
set -euo pipefail

# ---- config / globals -------------------------------------------------------
DEFAULT_DELAY=30
DELAY="$DEFAULT_DELAY"
SIM=0
DRYRUN=0
ROLE=""
STAGE=""
SCORE_LEVEL="" # optional override for the capstone dial (empty -> mission default 6)
DO_PINGER=""   # optional override: 0 skips S1 pinger homing (empty -> mission default 1)
RUN_ROOT="${POOLTEST_RUNS:-$PWD/pooltest_runs}"
SIM_IMAGE_TOPIC="/down_cam/image_raw"
BAG_TOPICS=(/odometry/filtered /goal_pose /yolo_down/detections /yolo/detections)
ORDERED_STAGES=(preflight calib hone select combined deadreckon full_s2 loop full)
BAG_PID="" # set by start_bag; cleared by stop_bag

# ---- cleanup trap -----------------------------------------------------------
_cleanup() { stop_bag; }
_on_signal() {
	stop_bag
	exit 130
}
trap _cleanup EXIT
trap _on_signal INT TERM

# ---- small ui helpers -------------------------------------------------------
hr() { printf '%s\n' "------------------------------------------------------------"; }
say() { printf '%s\n' "$*"; }
ask() {
	local p="$1" v
	read -r -p "$p" v
	printf '%s' "$v"
}

usage() {
	cat <<'EOF'
pooltest.sh — guided poolside test runner for the Task 5 down-cam stack (S2+S3)

USAGE:
  ./pooltest.sh [STAGE] [FLAGS]      run one stage (or the guided sequence if omitted)

STAGES (smallest whole unit first):
  preflight   topics/camera/YOLO/odom are alive (no motion)
  calib       CenterCamera sign/gain calibration  (mission: CenterCameraTest)
  hone        hold center over the table          (mission: HoneOverTableOnly)
  select      lock the role target                (mission: SelectOnly)
  combined    center then select, one process     (mission: HoneOverTableSelect)  [S2+S3 milestone]
  deadreckon  (Tier B) measure the 4 dead-reckon constants (manual teleop)
  full_s2     (Tier B) full surface->descend->dead-reckon->hone (mission: OctagonTableMission)
  loop        S6: grab+place up to TWO objects     (mission: OctagonLoopMission)
  full        WHOLE Task 5 run S1..S7              (mission: OctagonMission)

FLAGS:
  --role R        R = survey_repair (nut_cylinder,electric_box) | search_rescue (pill_cylinder,bandaid_box)
  --sim           rehearse the harness in Gazebo (adds down_image_topic override)
  --delay N       autonomous-start countdown seconds after GO (default 30)
  --score-level N ('full' only) how far up the ladder to run (default 6 = everything):
                    0 pinger->table | 1 +center | 2 +grab 1 | 3 +grab 2 | 5 +face image | 6 +rotation bonus
  --no-pinger     ('full' only) skip S1 acoustic homing; start at S2 (sub is hand-placed over the octagon)
  --dry-run       print directions and prompts but do NOT launch missions or record
  -h|--help       this help

WHAT YOU NEED before running a real test:
  * sub powered and in the water
  * the down-cam YOLO node running with the required model(s) loaded
      - 'table' model for calib/hone/combined/full_s2
      - role object model (nut_cylinder/electric_box or pill_cylinder/bandaid_box) for select/combined
  * the ROLE for this run (survey_repair or search_rescue)
  * a diver/teleop to position the sub before each motion stage
  * clear pool space

OUTPUT: each run writes to  pooltest_runs/<stage>_<UTC>/  (bag/, console.log, results.txt[, constants.txt])

AFTER YOU TYPE GO: you have <delay> seconds to disconnect and clear the sub; it
then runs on its own. Run this script inside tmux so it survives your disconnect:
    tmux new -s pooltest   then   ./pooltest.sh
EOF
}

# ---- arg parsing ------------------------------------------------------------
parse_args() {
	while [ $# -gt 0 ]; do
		case "$1" in
		-h | --help)
			usage
			exit 0
			;;
		--sim) SIM=1 ;;
		--dry-run) DRYRUN=1 ;;
		--delay)
			[ $# -ge 2 ] || {
				say "Error: --delay requires a value"
				exit 2
			}
			DELAY="$2"
			shift
			;;
		--role)
			[ $# -ge 2 ] || {
				say "Error: --role requires a value"
				exit 2
			}
			ROLE="$2"
			shift
			;;
		--score-level)
			[ $# -ge 2 ] || {
				say "Error: --score-level requires a value"
				exit 2
			}
			SCORE_LEVEL="$2"
			shift
			;;
		--no-pinger) DO_PINGER=0 ;;
		preflight | calib | hone | select | combined | deadreckon | full_s2 | loop | full) STAGE="$1" ;;
		*)
			say "Unknown argument: $1"
			usage
			exit 2
			;;
		esac
		shift
	done
	case "$DELAY" in
	'' | *[!0-9]*)
		say "Error: --delay requires a non-negative integer"
		exit 2
		;;
	esac
	if [ -n "$SCORE_LEVEL" ]; then
		case "$SCORE_LEVEL" in
		'' | *[!0-9]*)
			say "Error: --score-level requires a non-negative integer (0..6)"
			exit 2
			;;
		esac
	fi
}

# ---- environment guards -----------------------------------------------------
warn_if_not_detachable() {
	# Real runs must survive operator disconnect; tmux/screen is how we do that.
	if [ "$DRYRUN" -eq 0 ] && [ -z "${TMUX:-}" ] && [ -z "${STY:-}" ]; then
		hr
		say "WARNING: not running inside tmux/screen."
		say "If you disconnect after GO, the test will DIE. Start tmux first:"
		say "    tmux new -s pooltest   then re-run this script."
		local a
		a="$(ask 'Continue anyway? [y/N] ')"
		[ "$a" = "y" ] || {
			say "Aborting — start tmux and retry."
			exit 1
		}
	fi
}

# ---- run dir / bag / mission launch ----------------------------------------
make_run_dir() { # $1 = stage name -> echoes the path
	local ts
	ts="$(date -u +%Y%m%dT%H%M%SZ)"
	local d="$RUN_ROOT/${1}_${ts}"
	mkdir -p "$d"
	printf '%s' "$d"
}

start_bag() { # $1 = run dir; sets BAG_PID (empty if recording failed to start). Always returns 0.
	if [ "$DRYRUN" -eq 1 ]; then
		say "DRY RUN: would record bag of: ${BAG_TOPICS[*]}"
		BAG_PID=""
		return 0
	fi
	setsid ros2 bag record -o "$1/bag" "${BAG_TOPICS[@]}" >"$1/bag.log" 2>&1 &
	BAG_PID=$!
	sleep 1
	if ! kill -0 "$BAG_PID" 2>/dev/null; then
		say "WARNING: ros2 bag record did not start (see $1/bag.log)."
		say "  This run will NOT be recorded. Ctrl-C to abort and fix, or continue unrecorded."
		: >"$1/BAG_FAILED"
		BAG_PID=""
	fi
	return 0
}

stop_bag() { # uses global BAG_PID set by start_bag
	[ -n "${BAG_PID:-}" ] || return 0
	kill -INT "$BAG_PID" 2>/dev/null || true
	wait "$BAG_PID" 2>/dev/null || true
	BAG_PID=""
}

run_mission() { # $1 = run dir, $2 = mission name. Launches + waits. Always returns 0 (a normal mission FAILURE must not abort the stage); a startup/fatal crash is detected and surfaced. Detail is in console.log.
	local dir="$1" mission="$2"
	local args=(--ros-args -p mission:="$mission")
	[ -n "$ROLE" ] && args+=(-p role:="$ROLE")
	[ "$SIM" -eq 1 ] && args+=(-p down_image_topic:="$SIM_IMAGE_TOPIC")
	# Capstone dials (harmless for missions that don't read them; the node always declares them).
	[ -n "$SCORE_LEVEL" ] && args+=(-p score_level:="$SCORE_LEVEL")
	[ -n "$DO_PINGER" ] && args+=(-p do_pinger:="$DO_PINGER")
	if [ "$DRYRUN" -eq 1 ]; then
		say "DRY RUN: would launch mission:=$mission ${ROLE:+role:=$ROLE}${SCORE_LEVEL:+ score_level:=$SCORE_LEVEL}${DO_PINGER:+ do_pinger:=$DO_PINGER}$([ "$SIM" -eq 1 ] && printf ' (sim)')"
		return 0
	fi
	# Mission runs in its own session (setsid) so a terminal Ctrl-C never reaches it.
	setsid ros2 run mission_planner mission_planner_node "${args[@]}" >"$dir/console.log" 2>&1 &
	local mpid=$!
	say "Mission running (pid $mpid). Ctrl-C here stops ONLY this live view; the mission and bag keep running."
	# Live view in foreground. Ctrl-C must kill only this tail, not stop_bag: point
	# INT at killing the tail. The bag and mission are setsid'd away from the
	# terminal, so the terminal SIGINT never reaches them anyway.
	tail -f --pid="$mpid" "$dir/console.log" &
	local tpid=$!
	trap 'kill "$tpid" 2>/dev/null || true' INT
	wait "$tpid" 2>/dev/null || true
	trap _on_signal INT
	wait "$mpid" 2>/dev/null || true
	# Catch a node that crashed/aborted at startup (e.g. workspace not rebuilt after
	# adding missions -> "Unknown mission"). A normal mission FAILURE is fine here.
	if grep -qiE "Unknown mission|terminate called|what\(\):|\[FATAL\]" "$dir/console.log" 2>/dev/null; then
		say "WARNING: the mission node reported a startup/fatal error (see $dir/console.log)."
		say "  If you just added missions, rebuild + re-source the workspace — this stage likely did NOT run."
	fi
	return 0
}

# ---- gates / briefing -------------------------------------------------------
go_gate() { # waits for GO
	local a
	a="$(ask 'Type GO when the sub is positioned and you are ready: ')"
	[ "$a" = "GO" ] || {
		say "Did not receive GO (got '$a'); skipping stage."
		return 1
	}
}

countdown() { # honors --delay; skipped in dry-run or when delay=0
	local n="$DELAY"
	if [ "$DRYRUN" -eq 1 ] || [ "$n" -le 0 ]; then return 0; fi
	hr
	say "AUTONOMOUS START in ${n}s — DISCONNECT AND CLEAR THE SUB NOW."
	while [ "$n" -gt 0 ]; do
		printf '\r  %2ds...' "$n"
		sleep 1
		n=$((n - 1))
	done
	printf '\r        \r'
}

briefing() {
	hr
	say "POOL TEST — Task 5 down-cam stack (S2 + S3)"
	hr
	usage | sed -n '/WHAT YOU NEED/,/tmux new/p'
	hr
}

# ---- helpers for stage preconditions / data collection ----------------------
ensure_role() {
	if [ "$DRYRUN" -eq 1 ] && [ -z "$ROLE" ]; then
		say "DRY RUN: would prompt for role; defaulting to survey_repair"
		ROLE="survey_repair"
		return 0
	fi
	while [ "$ROLE" != "survey_repair" ] && [ "$ROLE" != "search_rescue" ]; do
		say "This stage needs a ROLE. Valid values:"
		say "  survey_repair  -> objects: nut_cylinder, electric_box"
		say "  search_rescue  -> objects: pill_cylinder, bandaid_box"
		read -r -p 'Enter role: ' ROLE || {
			say "No role provided; aborting."
			exit 2
		}
	done
}

# Fail loud if the down-cam detection stream is not alive before a vision stage.
require_down_detections() {
	if [ "$DRYRUN" -eq 1 ]; then
		say "DRY RUN: would verify /yolo_down/detections is publishing"
		return 0
	fi
	if ! timeout 5 ros2 topic hz /yolo_down/detections >/dev/null 2>&1; then
		hr
		say "PRECONDITION FAILED: no detections on /yolo_down/detections."
		say "Likely cause: the down-cam YOLO node (yolo_down) is not running, or no model is loaded."
		say "Fix that, then re-run this stage. NOT launching a motion stage blind."
		return 1
	fi
}

# Append a labeled results form to results.txt. Args: run dir, then "Label|format e.g." specs.
collect_form() {
	local dir="$1"
	shift
	local f="$dir/results.txt"
	{
		echo "=== RESULTS ($(date -u +%FT%TZ)) ==="
		echo "stage_dir: $dir"
	} >>"$f"
	local spec label val
	for spec in "$@"; do
		label="${spec%%|*}"
		val="$(ask "$label (${spec#*|}): ")" || val=""
		printf '%s: %s\n' "$label" "$val" >>"$f"
	done
	say "Saved results -> $f"
}

# ---- stage implementations --------------------------------------------------
check_topic_alive() { # $1 = label, $2 = topic; prints rate or MISSING; returns 0 if publishing, 1 if not
	local out
	out="$(timeout 5 ros2 topic hz "$2" 2>/dev/null | head -n 2 || true)"
	say "-- $1 ($2):"
	if [ -n "$out" ]; then
		say "$out"
		return 0
	fi
	say "  MISSING"
	return 1
}

stage_preflight() {
	hr
	say "STAGE 0 — PREFLIGHT (no motion). Confirms the rig before we move the sub."
	say "Required model(s): 'table' (calib/hone/combined/full_s2), role object model (select/combined)."
	local img="/down_camera/rgb/image_raw"
	[ "$SIM" -eq 1 ] && img="$SIM_IMAGE_TOPIC"
	if [ "$DRYRUN" -eq 1 ]; then
		say "DRY RUN: would check: $img publishing (+WxH), /yolo_down/detections rate, /odometry/filtered, classes seen."
		return 0
	fi
	local ok=1
	check_topic_alive "down image" "$img" || ok=0
	check_topic_alive "detections" "/yolo_down/detections" || ok=0
	check_topic_alive "odometry" "/odometry/filtered" || ok=0
	say "-- classes currently detected:"
	local classes
	classes="$(timeout 5 ros2 topic echo --once /yolo_down/detections 2>/dev/null | grep -i "class_name" || true)"
	if [ -n "$classes" ]; then say "$classes"; else say "  (none in this sample)"; fi
	hr
	if [ "$ok" -eq 1 ]; then say "PREFLIGHT OK."; else
		say "PREFLIGHT FAILED — fix MISSING items before any motion stage."
		return 1
	fi
}

stage_calib() {
	hr
	say "STAGE 1 — CALIBRATE (mission: CenterCameraTest). Smallest whole unit."
	say "DO THIS:"
	say "  1. POSITION: diver holds the sub level, ~1 m above the table, table centered beneath it."
	say "  2. TYPE: GO (after the countdown the sub will move ON ITS OWN to center over the table, ~30s)."
	say "  3. WHAT WILL HAPPEN: the sub drives to reduce the table's offset from image center and holds."
	say "  4. SUCCESS LOOKS LIKE: the sub settles centered over the table and stops drifting."
	say "IF IT DROVE AWAY FROM THE TABLE last run, edit the literals and rebuild:"
	say "  file: src/subjugator/mission_planner/subjugator_missions/xml/center_camera_test_mission.xml"
	say '  try:  set swap_axes="true", or flip map_x_sign / map_y_sign between 1.0 and -1.0'
	say "  then: source scripts/setup.bash && colcon build --packages-select mission_planner && source install/setup.bash"
	go_gate || return 0
	local dir
	dir="$(make_run_dir calib)"
	start_bag "$dir"
	countdown
	run_mission "$dir" CenterCameraTest
	stop_bag
	collect_form "$dir" \
		"Converged toward target|y/n e.g. y" \
		"Signs used (swap_axes/map_x_sign/map_y_sign)|text e.g. false/1.0/-1.0" \
		"kp / max_step used|text e.g. 0.5/0.25" \
		"Final behavior|tight/oscillated/drifted e.g. tight" \
		"Approx settle time|seconds e.g. 12" \
		"Notes|free text"
	say "SUMMARY (report back): the converging sign triple + gains recorded above."
}

stage_hone() {
	hr
	say "STAGE 2 — HONE (mission: HoneOverTableOnly). The real centering subtree."
	say "DO THIS:"
	say "  1. POSITION: same as calibrate — sub ~1 m above the table, table centered beneath it."
	say "  2. TYPE: GO (sub will autonomously center over the table after the countdown)."
	say "  3. WHAT WILL HAPPEN: HoneOverTarget centers; if it never sees the table it gives up gracefully."
	say "  4. SUCCESS LOOKS LIKE: sub holds centered over the table using the calibrated signs."
	require_down_detections || return 0
	go_gate || return 0
	local dir
	dir="$(make_run_dir hone)"
	start_bag "$dir"
	countdown
	run_mission "$dir" HoneOverTableOnly
	stop_bag
	collect_form "$dir" \
		"Held center|y/n e.g. y" \
		"Table detection confidence range|text e.g. 0.7-0.9" \
		"Approx altitude over table|meters e.g. 1.0" \
		"Detection stability|stable/flickery e.g. stable" \
		"Fell back to AlwaysSuccess (lost table)|y/n e.g. n" \
		"Notes|free text"
}

stage_select() {
	hr
	say "STAGE 3 — SELECT (mission: SelectOnly). Holds station, then decides (no travel)."
	ensure_role
	say "DO THIS:"
	say "  1. POSITION: sub so the TASK OBJECTS are in the down-cam frame (over the table)."
	say "  2. TYPE: GO (the sub holds position and just decides which object to grab)."
	say "  3. WHAT WILL HAPPEN: SelectTarget locks the highest-confidence role object across frames."
	say "  4. SUCCESS LOOKS LIKE: console prints  SelectTarget: locked '<class>' (N frames)."
	say "  ROLE this run: $ROLE"
	require_down_detections || return 0
	go_gate || return 0
	local dir
	dir="$(make_run_dir select)"
	start_bag "$dir"
	countdown
	run_mission "$dir" SelectOnly
	stop_bag
	collect_form "$dir" \
		"Locked label|text e.g. nut_cylinder" \
		"Was it correct|y/n e.g. y" \
		"Lock time / frames|text e.g. 2s/3" \
		"Per-class confidence seen|text e.g. nut_cylinder 0.8, electric_box 0.6" \
		"Any false picks|y/n e.g. n" \
		"min_conf / consecutive_frames used|text e.g. 0.60/3" \
		"Notes|free text"
}

stage_combined() {
	hr
	say "STAGE 4 — COMBINED (mission: HoneOverTableSelect). *** S2+S3 MILESTONE ***"
	ensure_role
	say "DO THIS:"
	say "  1. POSITION: sub ~1 m above the table with the task objects in view."
	say "  2. TYPE: GO (sub autonomously centers over the table, THEN locks the target — one run)."
	say "  3. WHAT WILL HAPPEN: hone phase centers; then select phase prints the locked class."
	say "  4. SUCCESS LOOKS LIKE: it centers, then console prints  locked '<class>'."
	say "  ROLE this run: $ROLE"
	require_down_detections || return 0
	go_gate || return 0
	local dir
	dir="$(make_run_dir combined)"
	start_bag "$dir"
	countdown
	run_mission "$dir" HoneOverTableSelect
	stop_bag
	collect_form "$dir" \
		"Centered then locked|y/n e.g. y" \
		"Locked label|text e.g. nut_cylinder" \
		"Total time|seconds e.g. 35" \
		"Handoff worked (target_label populated)|y/n e.g. y" \
		"Notes|free text"
}

# Echo "x y z" of one /odometry/filtered pose, or nothing on failure. Dry-run returns zeros.
snap_pose() {
	if [ "$DRYRUN" -eq 1 ]; then
		printf '0 0 0'
		return 0
	fi
	local out
	out="$(ros2 topic echo --once /odometry/filtered --field pose.pose.position 2>/dev/null |
		awk '/^x:/{x=$2} /^y:/{y=$2} /^z:/{z=$2} END{if(x!=""&&y!=""&&z!="") printf "%s %s %s", x, y, z}' || true)"
	printf '%s' "$out"
}

stage_deadreckon() {
	hr
	say "STAGE 5 — MEASURE DEAD-RECKON CONSTANTS (Tier B). Manual teleop — NO autonomous delay."
	say "You drive the sub; type SNAP at each checkpoint to record its pose. Order matters."
	say "  CP1 = S1-arrival start pose (a pool marker)"
	say "  CP2 = at the surface"
	say "  CP3 = at down-cam hover depth (table fills the frame)"
	say "  CP4 = centered over the table"
	if [ "$DRYRUN" -eq 1 ]; then
		say "DRY RUN: would snapshot CP1..CP4 and compute the 4 constants."
		return 0
	fi
	local dir
	dir="$(make_run_dir deadreckon)"
	local cp cp1 cp2 cp3 cp4 a
	for cp in CP1 CP2 CP3 CP4; do
		a="$(ask "Drive to $cp, then type SNAP: ")" || {
			say "Aborted (no input)."
			return 1
		}
		[ "$a" = "SNAP" ] || {
			say "Expected SNAP; aborting."
			return 1
		}
		local pose
		pose="$(snap_pose)"
		if [ -z "$pose" ]; then
			say "ERROR: no data from /odometry/filtered at $cp — is odom publishing? Aborting."
			return 1
		fi
		case "$cp" in
		CP1) cp1="$pose" ;;
		CP2) cp2="$pose" ;;
		CP3) cp3="$pose" ;;
		CP4) cp4="$pose" ;;
		esac
		say "  $cp = $pose"
	done
	# deltas: surface_dz=z2-z1, hover_dz=z3-z2, octagon_dx=x4-x1, octagon_dy=y4-y1
	local x1 y1 z1 z2 z3 x4 y4
	read -r x1 y1 z1 <<<"$cp1"
	read -r _ _ z2 <<<"$cp2"
	read -r _ _ z3 <<<"$cp3"
	read -r x4 y4 _ <<<"$cp4"
	{
		echo "# Dead-reckon constants (measured $(date -u +%FT%TZ))"
		echo "# VALID ONLY for this start pose; RE-MEASURE at competition vs the real S1 arrival."
		awk -v z1="$z1" -v z2="$z2" -v z3="$z3" -v x1="$x1" -v y1="$y1" -v x4="$x4" -v y4="$y4" 'BEGIN{
			printf "surface_dz = %.3f\n", z2 - z1
			printf "hover_dz   = %.3f\n", z3 - z2
			printf "octagon_dx = %.3f\n", x4 - x1
			printf "octagon_dy = %.3f\n", y4 - y1
		}'
		echo "# Edit these into octagon_table_mission.xml (steps 1-3) before running 'full_s2'."
	} | tee "$dir/constants.txt"
	say "Saved -> $dir/constants.txt"
}

stage_full_s2() {
	hr
	say "STAGE 6 — FULL S2 (mission: OctagonTableMission). Tier B."
	say "DO THIS:"
	say "  1. PRECONDITION: the four dead-reckon constants in octagon_table_mission.xml are set"
	say "     (run the 'deadreckon' stage first and edit them in)."
	say "  2. POSITION: place the sub at the designated S1-arrival start pose."
	say "  3. TYPE: GO (sub will surface, descend, dead-reckon to center, then hone — autonomous)."
	say "  4. SUCCESS LOOKS LIKE: sub ends centered over the table."
	go_gate || return 0
	local dir
	dir="$(make_run_dir full_s2)"
	start_bag "$dir"
	countdown
	run_mission "$dir" OctagonTableMission
	stop_bag
	collect_form "$dir" \
		"Reached the table area|y/n e.g. y" \
		"Hone centered at the end|y/n e.g. y" \
		"Which step failed (if any)|text e.g. none" \
		"Notes|free text"
}

stage_loop() {
	hr
	say "STAGE 7 — COLLECTION LOOP / S6 (mission: OctagonLoopMission)."
	say "Grabs and places up to TWO objects: S3 select -> S4 grasp -> S5 place, twice."
	say "The 2nd cycle skips the object already placed (grabbed_label exclusion); grabbing"
	say "just ONE object still counts as success."
	ensure_role
	say "DO THIS:"
	say "  1. PRECONDITION: sub already CENTERED over the table (run 'combined' first, or hand-hold it),"
	say "     the task objects AND the role's basket marker in the down-cam frame, gripper service up."
	say "  2. TYPE: GO (after the countdown the sub autonomously grabs, carries, and drops — twice)."
	say "  3. WHAT WILL HAPPEN: for each object it centers, descends, closes the gripper, lifts,"
	say "     moves to the basket, centers on the marker, descends, opens the gripper, lifts."
	say "  4. SUCCESS LOOKS LIKE: one or two objects end up in the role's basket."
	say "  ROLE this run: $ROLE  (survey_repair -> warning basket; search_rescue -> red_cross basket)"
	require_down_detections || return 0
	go_gate || return 0
	local dir
	dir="$(make_run_dir loop)"
	start_bag "$dir"
	countdown
	run_mission "$dir" OctagonLoopMission
	stop_bag
	collect_form "$dir" \
		"Objects placed (0/1/2)|number e.g. 2" \
		"Object 1 label grabbed|text e.g. nut_cylinder" \
		"Object 2 label grabbed|text e.g. electric_box" \
		"Dropped into correct basket|y/n e.g. y" \
		"Where it failed (if <2)|text e.g. 2nd grasp missed" \
		"Notes|free text"
}

stage_full() {
	hr
	say "STAGE 8 — WHOLE TASK 5 RUN / S1..S7 (mission: OctagonMission). *** FULL MISSION ***"
	say "DEFAULT runs EVERYTHING: pinger->table, center, grab+place 2 objects, face the wall image,"
	say "rotation bonus. No flags needed for the competition run."
	ensure_role
	say "DO THIS:"
	say "  1. PRECONDITION (default, autonomous): the Task-5 octagon PINGER is active and the sub is"
	say "     in the water within acoustic range. S1 will home onto the table on its own."
	say "     NO PINGER? add --no-pinger and hand-place the sub over the octagon (starts at S2)."
	say "     S7 needs the 'octagon_symbols' forward-cam model; without it S7 fails at CheckYoloModel"
	say "     AFTER the objects are already placed (they still score; only the final log says FAILURE)."
	say "  2. BRING-UP: dial how far it goes with --score-level N (default 6):"
	say "       0 pinger->table | 1 +center | 2 +grab 1 | 3 +grab 2 | 5 +face image | 6 +rotation bonus"
	say "  3. TYPE: GO (after the countdown the sub runs the whole mission ON ITS OWN)."
	say "  4. SUCCESS LOOKS LIKE: sub reaches the table, collects the objects into baskets, then"
	say "     faces the octagon-wall image (and spins if score_level 6)."
	say "  RUN: role=$ROLE  score_level=${SCORE_LEVEL:-6 (default)}  do_pinger=${DO_PINGER:-1 (default)}"
	go_gate || return 0
	local dir
	dir="$(make_run_dir full)"
	start_bag "$dir"
	countdown
	run_mission "$dir" OctagonMission
	stop_bag
	collect_form "$dir" \
		"Farthest stage reached (S1..S7)|text e.g. S5" \
		"Objects placed (0/1/2)|number e.g. 2" \
		"Faced the wall image (S7)|y/n/na e.g. na" \
		"Where it stopped/failed|text e.g. S7 no model" \
		"End-of-mission status|SUCCESS/FAILURE e.g. FAILURE" \
		"Notes|free text"
}

# ---- guided sequence --------------------------------------------------------
run_guided_sequence() {
	local s a
	for s in "${ORDERED_STAGES[@]}"; do
		hr
		a="$(ask "Run stage '$s' next? [y/N/q] ")"
		case "$a" in
		y | Y) "stage_$s" || say "Stage '$s' did not complete; continuing." ;;
		q | Q)
			say "Quitting guided sequence."
			return 0
			;;
		*) say "Skipping '$s'." ;;
		esac
	done
	say "Guided sequence complete. Outputs under $RUN_ROOT/"
}

main() {
	parse_args "$@"
	mkdir -p "$RUN_ROOT"
	briefing
	warn_if_not_detachable
	if [ -n "$STAGE" ]; then
		# Run one stage in an `if` so a stage's nonzero return is a clean exit code
		# (set -e would otherwise abort mid-handler); guided mode handles per-stage.
		if "stage_$STAGE"; then
			exit 0
		else
			say "Stage '$STAGE' reported failure."
			exit 1
		fi
	else
		run_guided_sequence
	fi
}

main "$@"
