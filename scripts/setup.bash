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
alias cb="colbuild"


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
colbuild() {
	local prev_dir
	prev_dir=$(pwd)                # Store the current directory
	cd $MIL_REPO || return         # Change to your workspace
	if [ $# -eq 0 ]; then 
		colcon build --symlink-install # Build the workspace
	else
		colcon build --symlink-install --packages-select "$@" # Build the workspace
	fi
	source ./install/setup.bash    # Source the install script
	cd "$prev_dir" || return       # Return to the original directory
}

# Autocomplete for colbuild based on ROS 2 packages
_colbuild_autocomplete() {
    local cur
    cur="${COMP_WORDS[COMP_CWORD]}"  # Get the current word being typed
    local packages

    # Fetch the list of packages from the ROS 2 workspace (replace this with your workspace)
    packages=$(cd $MIL_REPO && colcon list --names-only)

    mapfile -t package_array <<< "$packages"

    # Filter packages based on the current word (autocomplete logic)
    COMPREPLY=($(compgen -W "${package_array[*]}" -- "$cur"))
}

# Bind the autocomplete function to the colbuild command
complete -F _colbuild_autocomplete colbuild
complete -F _colbuild_autocomplete cb
complete -F _colbuild_autocomplete colcon_cd
complete -F _colbuild_autocomplete ccd
