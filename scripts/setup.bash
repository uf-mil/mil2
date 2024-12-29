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

alias mil='cd $MIL_REPO'

alias srcbrc="source ~/.bashrc"
alias search_root='sudo find / ... | grep -i'
alias search='find . -print | grep -i'
alias fd="fdfind"

# potentially borrowed from forrest
autopush() {
	git push origin +"${1:-HEAD}":refs/heads/autopush-cameron-"$(uuidgen --random | cut -c1-8)"-citmp
}
