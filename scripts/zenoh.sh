#!/bin/sh

ROS2="$(which ros2)"

do_usage() {
	echo "Usage: zenoh.sh {start <ips>|restart <ips>|stop|status}" >&2
}

do_start() {
	# parse provided IP addresses
	if [ "$#" -ge 2 ]; then
		ENDPOINTS="tcp/$2:7447"
		shift 2
		for ip in "$@"; do
			ENDPOINTS="tcp/$ip:7447,$ENDPOINTS"
		done
		export ZENOH_CONFIG_OVERRIDE="connect/endpoints=[\"$ENDPOINTS\"]"
	fi

	start-stop-daemon -v --start -b --name rmw_zenohd -O /tmp/zenoh.log \
		--startas "$ROS2" run rmw_zenoh_cpp rmw_zenohd

	if [ "$?" = 0 ] && [ -n "$ENDPOINTS" ]; then
		echo "endpoints: $ENDPOINTS"
	fi
}

do_stop() {
	start-stop-daemon -v --stop --retry 1 --name rmw_zenohd
}

case "$1" in
start)
	do_start "$@"
	;;
stop)
	do_stop
	;;
status)
	start-stop-daemon -v --status --name rmw_zenohd
	if [ "$?" = 0 ]; then
		echo "running"
	else
		echo "not running"
	fi
	;;
restart)
	do_stop
	do_start "$@"
	;;
*)
	do_usage
	;;
esac
