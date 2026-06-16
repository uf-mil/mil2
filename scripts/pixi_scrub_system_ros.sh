# shellcheck shell=sh
# Runs as a pixi activation script. ~/.bashrc may source scripts/setup.bash,
# which sources the system /opt/ros/jazzy and leaks its Gazebo vendor libraries
# (built against libprotobuf.so.32) into PATH-like env vars. Pixi/robostack ships
# libprotobuf.so.31, so loading both into one process corrupts the heap and
# crashes Gazebo on launch (even a stock empty.sdf, headless). This was inherited
# from the parent shell, so it survives `pixi shell` / `pixi run` unless scrubbed.
# Here we strip every /opt/ros component from the inherited path-like vars and
# drop the system-ROS-only vars entirely. Pixi's own entries are untouched.
# See issue #454.

_mil_strip_opt_ros() {
	# Echo $1 (a colon-separated path list) with any component containing
	# /opt/ros removed.
	_old="$1"
	_new=""
	_IFS_save="$IFS"
	IFS=":"
	for _p in $_old; do
		case "$_p" in
		*/opt/ros/*) ;; # drop it
		"") ;;          # drop empties
		*) _new="${_new:+$_new:}$_p" ;;
		esac
	done
	IFS="$_IFS_save"
	printf '%s' "$_new"
}

# Scrub /opt/ros components out of every path-like var. We must export the
# result (even when empty): pixi captures activation via exported variables, so
# a plain `unset` here is silently ignored and the leaked value would survive.
for _var in LD_LIBRARY_PATH PATH CMAKE_PREFIX_PATH PYTHONPATH PKG_CONFIG_PATH \
	AMENT_PREFIX_PATH GZ_SIM_RESOURCE_PATH GZ_SIM_SYSTEM_PLUGIN_PATH \
	GZ_GUI_PLUGIN_PATH; do
	eval "_cur=\${$_var:-}"
	if [ -n "$_cur" ]; then
		_scrubbed=$(_mil_strip_opt_ros "$_cur")
		export "$_var=$_scrubbed"
	fi
done

# Pin gz to pixi's own config dir. A clean pixi env leaves this unset (gz then
# uses its built-in $CONDA_PREFIX/share/gz); setting it explicitly is equivalent
# and overrides any leaked system path that pixi would otherwise re-inject.
if [ -n "${CONDA_PREFIX:-}" ] && [ -d "$CONDA_PREFIX/share/gz" ]; then
	export GZ_CONFIG_PATH="$CONDA_PREFIX/share/gz"
fi

unset _mil_strip_opt_ros _cur _scrubbed _var _old _new _p _IFS_save 2>/dev/null || true
