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
RUN_ROOT="${POOLTEST_RUNS:-$PWD/pooltest_runs}"
SIM_IMAGE_TOPIC="/down_cam/image_raw"
BAG_TOPICS=(/odometry/filtered /goal_pose /yolo_down/detections)
# shellcheck disable=SC2034  # referenced by Tasks 3-4 stage stubs
ORDERED_STAGES=(preflight calib hone select combined deadreckon full_s2)
BAG_PID="" # set by start_bag; cleared by stop_bag

# ---- cleanup trap -----------------------------------------------------------
_cleanup() { stop_bag; }
trap _cleanup EXIT INT TERM

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

FLAGS:
  --role R    R = survey_repair (nut_bolt,plug) | search_rescue (pill,bandage)
  --sim       rehearse the harness in Gazebo (adds down_image_topic override)
  --delay N   autonomous-start countdown seconds after GO (default 30)
  --dry-run   print directions and prompts but do NOT launch missions or record
  -h|--help   this help

WHAT YOU NEED before running a real test:
  * sub powered and in the water
  * the down-cam YOLO node running with the required model(s) loaded
      - 'table' model for calib/hone/combined
      - role object model (nut_bolt/plug or pill/bandage) for select/combined
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
		preflight | calib | hone | select | combined | deadreckon | full_s2) STAGE="$1" ;;
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

start_bag() { # $1 = run dir; sets BAG_PID
	if [ "$DRYRUN" -eq 1 ]; then
		say "DRY RUN: would record bag of: ${BAG_TOPICS[*]}"
		BAG_PID=""
		return
	fi
	setsid ros2 bag record -o "$1/bag" "${BAG_TOPICS[@]}" >"$1/bag.log" 2>&1 &
	BAG_PID=$!
}

stop_bag() { # uses global BAG_PID set by start_bag
	[ -n "${BAG_PID:-}" ] || return 0
	kill -INT "$BAG_PID" 2>/dev/null || true
	wait "$BAG_PID" 2>/dev/null || true
	BAG_PID=""
}

run_mission() { # $1 = run dir, $2 = mission name. Launches + waits. Always returns 0; mission SUCCESS/FAILURE is in console.log (stages always proceed to the results form).
	local dir="$1" mission="$2"
	local args=(--ros-args -p mission:="$mission")
	[ -n "$ROLE" ] && args+=(-p role:="$ROLE")
	[ "$SIM" -eq 1 ] && args+=(-p down_image_topic:="$SIM_IMAGE_TOPIC")
	if [ "$DRYRUN" -eq 1 ]; then
		say "DRY RUN: would launch mission:=$mission ${ROLE:+role:=$ROLE}$([ "$SIM" -eq 1 ] && printf ' (sim)')"
		return 0
	fi
	# setsid -> own session so it survives if the live view (tail) is interrupted.
	setsid ros2 run mission_planner mission_planner_node "${args[@]}" >"$dir/console.log" 2>&1 &
	local mpid=$!
	say "Mission running (pid $mpid). Live log (Ctrl-C just stops the view, not the mission):"
	tail -f --pid="$mpid" "$dir/console.log" || true
	wait "$mpid" 2>/dev/null || true
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
	while [ "$ROLE" != "survey_repair" ] && [ "$ROLE" != "search_rescue" ]; do
		say "This stage needs a ROLE. Valid values:"
		say "  survey_repair  -> objects: nut_bolt, plug"
		say "  search_rescue  -> objects: pill, bandage"
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
stage_preflight() {
	hr
	say "STAGE 0 — PREFLIGHT (no motion). Confirms the rig before we move the sub."
	say "Required model(s): 'table' (calib/hone/combined), role object model (select/combined)."
	local img="/down_camera/rgb/image_raw"
	[ "$SIM" -eq 1 ] && img="$SIM_IMAGE_TOPIC"
	if [ "$DRYRUN" -eq 1 ]; then
		say "DRY RUN: would check: $img publishing (+WxH), /yolo_down/detections rate, /odometry/filtered, classes seen."
		return 0
	fi
	local ok=1
	local hz_out
	say "-- down image ($img):"
	hz_out="$(timeout 5 ros2 topic hz "$img" 2>/dev/null | head -n 2 || true)"
	if [ -n "$hz_out" ]; then say "$hz_out"; else
		say "  MISSING"
		ok=0
	fi
	say "-- detections (/yolo_down/detections):"
	hz_out="$(timeout 5 ros2 topic hz /yolo_down/detections 2>/dev/null | head -n 2 || true)"
	if [ -n "$hz_out" ]; then say "$hz_out"; else
		say "  MISSING"
		ok=0
	fi
	say "-- odometry (/odometry/filtered):"
	hz_out="$(timeout 5 ros2 topic hz /odometry/filtered 2>/dev/null | head -n 2 || true)"
	if [ -n "$hz_out" ]; then say "$hz_out"; else
		say "  MISSING"
		ok=0
	fi
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
	countdown
	local dir
	dir="$(make_run_dir calib)"
	start_bag "$dir"
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
	countdown
	local dir
	dir="$(make_run_dir hone)"
	start_bag "$dir"
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
	say "STAGE 3 — SELECT (mission: SelectOnly). Decision only, no motion."
	ensure_role
	say "DO THIS:"
	say "  1. POSITION: sub so the TASK OBJECTS are in the down-cam frame (over the table)."
	say "  2. TYPE: GO (the sub holds position and just decides which object to grab)."
	say "  3. WHAT WILL HAPPEN: SelectTarget locks the highest-confidence role object across frames."
	say "  4. SUCCESS LOOKS LIKE: console prints  SelectTarget: locked '<class>' (N frames)."
	say "  ROLE this run: $ROLE"
	require_down_detections || return 0
	go_gate || return 0
	countdown
	local dir
	dir="$(make_run_dir select)"
	start_bag "$dir"
	run_mission "$dir" SelectOnly
	stop_bag
	collect_form "$dir" \
		"Locked label|text e.g. nut_bolt" \
		"Was it correct|y/n e.g. y" \
		"Lock time / frames|text e.g. 2s/3" \
		"Per-class confidence seen|text e.g. nut_bolt 0.8, plug 0.6" \
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
	countdown
	local dir
	dir="$(make_run_dir combined)"
	start_bag "$dir"
	run_mission "$dir" HoneOverTableSelect
	stop_bag
	collect_form "$dir" \
		"Centered then locked|y/n e.g. y" \
		"Locked label|text e.g. nut_bolt" \
		"Total time|seconds e.g. 35" \
		"Handoff worked (target_label populated)|y/n e.g. y" \
		"Notes|free text"
}

# Task 4 owns this stub — do NOT modify.
stage_deadreckon() { say "[deadreckon] stub"; }

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
	countdown
	local dir
	dir="$(make_run_dir full_s2)"
	start_bag "$dir"
	run_mission "$dir" OctagonTableMission
	stop_bag
	collect_form "$dir" \
		"Reached the table area|y/n e.g. y" \
		"Hone centered at the end|y/n e.g. y" \
		"Which step failed (if any)|text e.g. none" \
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
		"stage_$STAGE"
	else
		run_guided_sequence
	fi
}

main "$@"
