#! /usr/bin/env bash
MIL_REPO="$HOME/mil2"

if [[ $(ps -p $$ | tail -n 1 | awk '{ print $4 }') == "zsh" ]]; then
	source /opt/ros/jazzy/setup.zsh
else
	source /opt/ros/jazzy/setup.bash
fi

_list_complete() {
	local THING
	THINGS=("$1")
	for THING in "${THINGS[@]}"; do
		if [[ -z $2 || -n "$(echo "${THING:0:${#2}}" | grep "$2")" ]]; then
			COMPREPLY+=("$THING")
		fi
	done
}

# Use Cyclone DDS by default (it's super fast and amazing!)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Setup colcon_cd
source "/usr/share/colcon_cd/function/colcon_cd.sh"
export _colcon_cd_root=$MIL_REPO
alias ccd="colcon_cd"

# Setup up Gazebo
export GZ_VERSION=harmonic

# Setup colcon autocomplete
source "/usr/share/colcon_cd/function/colcon_cd-argcomplete.bash"

alias mil='cd $MIL_REPO'

alias srcbrc="source ~/.bashrc"
alias search_root='sudo find / ... | grep -i'
alias search='find . -print | grep -i'
alias fd="fdfind"
alias imu-socat="sudo socat PTY,link=/dev/ttyV0,mode=777 TCP:192.168.37.61:10001"

# potentially borrowed from forrest
autopush() {
	git push origin +"${1:-HEAD}":refs/heads/autopush-"$USER"-"$(uuidgen --random | cut -c1-8)"-citmp

}

# ssd utils
mount_ssd() {
	sudo mkdir -p /mnt/ssd
	sudo mount -t exfat /dev/sda1 /mnt/ssd
}

unmount_ssd() {
	sudo umount /mnt/ssd
}

prettycp() {
	rsync --recursive --times --modify-window=2 --progress --verbose --itemize-changes --stats --human-readable "$1" "$2"
}

# uhhh maybe also borrowed from forrest
cw() {
	git add -u
	git commit -m "work"
}

dmb() {
	git diff "$(git merge-base --fork-point "$(git branch -l main master --format '%(refname:short)')" HEAD)"
}

subnet_ip() {
	ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){2}37\.[0-9]*' | grep -v '127.0.0.1'
}

# This will remove symlink directories (fixes issue moving from plain colcon build to --symlink-install)
rm_symlink_dirs() {
	find "$MIL_REPO/build" -type d -path "*/ament_cmake_python/*/*" -exec rm -rf {} +
}

# This will build the repository from wherever you are and take you back into the mil2 repo
cb() {
	local prev_dir
	prev_dir=$(pwd)        # Store the current directory
	cd $MIL_REPO || return # Change to your workspace

	local packages=()
	local colcon_flags=()
	# Optionally add verbose flags
	for arg in "$@"; do
		if [ "$arg" = "--verbose" ]; then
			colcon_flags+=("--event-handlers" "console_cohesion+" "--cmake-args" "-DCMAKE_VERBOSE_MAKEFILE=ON")
		elif [ "$arg" = "--generate-compile-commands" ]; then
			colcon_flags+=("--cmake-args" "-DCMAKE_EXPORT_COMPILE_COMMANDS=1")
		else
			packages+=("$arg") # Add packages to local variable
		fi
	done

	if [ "${#packages[@]}" -eq 0 ]; then
		colcon build --symlink-install "${colcon_flags[@]}" # Build the workspace
	else
		colcon build --symlink-install "${colcon_flags[@]}" --packages-select "${packages[@]}" # Build the workspace
	fi

	# source based on shell
	if [ "$SHELL" = "/bin/bash" ]; then
		source "$MIL_REPO/install/setup.bash"
	elif [ "$SHELL" = "/bin/zsh" ]; then
		source "$MIL_REPO/install/setup.zsh"
	fi
	cd "$prev_dir" || return # Return to the original directory
}

# Autocomplete for cb based on ROS 2 packages
_cb_autocomplete() {
	local current_phrase
	current_phrase="${COMP_WORDS[COMP_CWORD]}"
	local packages

	# Fetch the list of packages from the ROS 2 workspace (replace this with your workspace)
	packages=$(colcon list --names-only --base-paths $MIL_REPO)
	COMPREPLY=()
	while IFS='' read -r line; do COMPREPLY+=("$line"); done < <(compgen -W "$packages" -- "$current_phrase")
}

# Used to run clang-tidy on packages
# Usage:
#   tidy                  --> Nothing
#   tidy <package>        --> Specific package
#   tidy <file>           --> Specific file
#   tidy --all            --> All packages
#   tidy --no-stop        --> Go through all errors before stopping$
#   tidy --fix            --> Fix changes
#   tidy --diff-files     --> Look at files which have changed in git
tidy() {
	local packages=()
	local all_packages=()
	while IFS='|' read -r line; do all_packages+=("$line"); done < <(colcon list --names-only --base-paths $MIL_REPO)
	local files=()
	local clang_tidy_args=("--config-file=.clang-tidy")
	local dont_stop=false
	for arg in "$@"; do
		if [ "$arg" = "--no-stop" ]; then
			dont_stop=true
		elif [ "$arg" = "--all" ]; then
			while IFS='' read -r line; do packages+=("$line"); done < <(colcon list --names-only --base-paths $MIL_REPO)
		elif [ "$arg" = "--fix" ]; then
			clang_tidy_args+=("--fix")
		elif [ "$arg" = "--verbose" ]; then
			clang_tidy_args+=("--verbose")
		elif [ "$arg" = "--help" ]; then
			echo "Usage: tidy [--all] [--fix] [--verbose] <package>"
			return
		elif [ "$arg" = "--diff-files" ]; then
			# Get the list of modified files in the git repository
			local modified_files=()
			while IFS='' read -r line; do modified_files+=("$line"); done < <(git diff --name-only HEAD)
			# Filter the files to only include .cpp, .h, and .hpp files
			for file in "${modified_files[@]}"; do
				if [[ $file == *.cpp || $file == *.h || $file == *.hpp ]]; then
					echo ">>> found modified file: $file"
					files+=("$file")
				fi
			done
		elif [[ $arg == -* ]]; then
			echo "Unknown option: $arg"
			return 1
		else
			# If arg is package, then use in package array, otherwise only files
			IFS="|"
			if [[ "${IFS}${all_packages[*]}${IFS}" == *"${IFS}${arg}${IFS}"* ]]; then
				echo ">>> found package: $arg"
				packages+=("$arg")
			elif [[ -f $arg ]]; then
				echo ">>> found file: $arg"
				files+=("$arg")
			else
				echo "Not file or package name: $arg"
				return 1
			fi
		fi
	done

	# Run clang-tidy on each package
	for package in "${packages[@]}"; do
		echo ">>> Running clang-tidy on $package..."
		local package_path
		package_path=$(colcon list --base-paths $MIL_REPO --packages-select "$package" --paths-only)
		# all
		if $dont_stop; then
			clang-tidy-20 "${clang_tidy_args[@]}" -p "$MIL_REPO/build/$package" "$(fd --type f --extension cpp --extension h --extension hpp . "$package_path")"
		else
			# only first error
			package_files=()
			while IFS='' read -r line; do package_files+=("$line"); done < <(fd --type f --extension cpp --extension h --extension hpp . "$package_path")
			for file in "${package_files[@]}"; do
				echo ">>> Running clang-tidy on $file..."
				clang-tidy-20 "${clang_tidy_args[@]}" -p "$MIL_REPO/build/$package" "$file"
				if [ $? -ne 0 ]; then
					echo ">>> Stopping on first error in $file"
					break
				fi
			done
		fi
	done

	# Run clang-tidy on each file
	for file in "${files[@]}"; do
		echo ">>> Running clang-tidy on $file..."
		clang-tidy-20 "${clang_tidy_args[@]}" -p "$MIL_REPO/build" "$file"
		if [ $? -ne 0 ]; then
			echo ">>> Stopping on first error in $file"
			break
		fi
	done
}

# bash autocompletion
complete -F _cb_autocomplete cb
complete -F _cb_autocomplete colcon_cd
complete -F _cb_autocomplete ccd
complete -F _cb_autocomplete tidy

# Print all devices on the specified subnet / network prefix
list_lan_devices() {
	if [ $# -lt 1 ]; then
		echo "Usage:   list_lan_devices <subnet>"
		echo "Example: list_lan_devices 192.168.37.1/24"
	fi
	nmap -sP "$1" -oG - | awk '/Up$/{print $2}'
}

alias list_mil_devices="list_lan_devices 192.168.37.1/24"

# aliases for localization and controller service calls
alias start-localization="ros2 service call /subjugator_localization/enable std_srvs/srv/Empty"
alias reset-localization="ros2 service call /subjugator_localization/reset std_srvs/srv/Empty"
alias start-controller='ros2 service call /pid_controller/enable std_srvs/srv/SetBool "{data: true}"'
alias stop-controller='ros2 service call /pid_controller/enable std_srvs/srv/SetBool "{data: false}"'
alias reset-controller="ros2 service call /pid_controller/reset std_srvs/srv/Empty"
