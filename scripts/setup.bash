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

# Setup colcon autocomplete
source "/usr/share/colcon_cd/function/colcon_cd-argcomplete.bash"

alias mil='cd $MIL_REPO'

alias srcbrc="source ~/.bashrc"
alias search_root='sudo find / ... | grep -i'
alias search='find . -print | grep -i'
alias fd="fdfind"

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
