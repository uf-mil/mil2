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

# bash autocompletion
complete -F _cb_autocomplete cb
complete -F _cb_autocomplete colcon_cd
complete -F _cb_autocomplete ccd

# Print all devices on the specified subnet / network prefix
list_lan_devices() {
	if [ $# -lt 1 ]; then
		echo "Usage:   list_lan_devices <subnet>"
		echo "Example: list_lan_devices 192.168.37.1/24"
	fi
	nmap -sP "$1" -oG - | awk '/Up$/{print $2}'
}

alias list_mil_devices="list_lan_devices 192.168.37.1/24"
