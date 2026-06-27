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

# ---- TEMPORARY stubs (replaced in Tasks 3 and 4) ----------------------------
stage_preflight() { say "[preflight] stub"; }
stage_calib() {
	say "[calib] DO THIS (stub)"
	go_gate || return 0
	countdown
	# TODO Task 3: wrap run_mission with start_bag/stop_bag
	run_mission "$(make_run_dir calib)" CenterCameraTest
}
stage_hone() { say "[hone] stub"; }
stage_select() { say "[select] stub"; }
stage_combined() { say "[combined] stub"; }
stage_deadreckon() { say "[deadreckon] stub"; }
stage_full_s2() { say "[full_s2] stub"; }
run_guided_sequence() { say "[guided] stub"; }

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
