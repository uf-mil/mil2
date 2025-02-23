#!/bin/bash
set -euo pipefail

# Parse arguments
POSITIONAL_ARGS=()

# Whether to install ROS2 from source (especially useful when installing on an OS other
# than Ubuntu 24.04) instead of installing from apt.
FROM_SOURCE=false

while [[ $# -gt 0 ]]; do
	case $1 in
		--from-source)
			FROM_SOURCE=true
			shift
			;;
		-*|--*)
			echo "Unknown option $1"
			exit 1
			;;
		*)
			POSITIONAL_ARGS+=("$1")
			shift
			;;
	esac
done

Res='\e[0;0m'
# shellcheck disable=SC2034
Red='\e[0;31m'
Gre='\e[0;32m'
Yel='\e[0;33m'
Pur='\e[0;35m'

color() {
	printf "%b" "$1"
}

# header display functions
hash_header() { echo "########################################"; }

# We only want to clear if the user is actually using a GUI terminal
if [[ -z ${TERM} ]]; then
	clear
fi

# Whether we can only use Ubuntu 24.04
ONLY_24=true
if "$FROM_SOURCE" ; then
	ONLY_24=false
fi

# Before even starting, check if using Ubuntu 20.04
NAME=$(awk -F= '/^NAME/{print $2}' /etc/os-release)
VERSION=$(awk -F'=' '/^VERSION=/{print $2}' /etc/os-release)

# if [[ $NAME != *"Ubuntu"* ]] && [[ "$ONLY_24" = true ]] ; then
# 	printf "${Red}This script is only supported on Ubuntu (you're using: ${NAME}). Please install Ubuntu 24.04.${Res}\n"
# 	exit 1
# fi
# if [[ $VERSION != *"24.04"* ]] && [[ "$ONLY_24" = true ]] ; then
# 	printf "${Red}This script is only supported on Ubuntu 24.04 (you're using: ${VERSION}). Please install Ubuntu 24.04.${Res}\n"
# 	exit 1
# fi

# Display header
cat <<EOF
$(color "$Pur")
         &@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#
       @@@@@@*,...................................................,/@@@@@@
     ,@@@@                                                            ,@@@@
     (@@@(                      /////&@* (@%////*                      &@@@*
     (@@@(                 #         %@* (@(         &                 &@@@*
     (@@@(                @@@@       %@* (@(      ,@@@@                &@@@*
     (@@@(               &@@@@@@%    %@* (@(    @@@@@@@#               &@@@*
     (@@@(              %/  @@@@@@@. %@* (@( ,@@@@@@&  #,              &@@@*
     (@@@(             *&     ,@@@@@@@@* (@@@@@@@@      @.             &@@@*
     (@@@(             @         (@@@@@* (@@@@@,         @             &@@@*
     (@@@(            @             @@@* (@@#            ,@            &@@@*
     (@@@(           &               %@* (@(              /#           &@@@*
     (@@@(          %/               %@* (@(               %,          &@@@*
     (@@@(         *@                %@* (@(                @.         &@@@*
     (@@@(         @                 %@* (@(                 @         &@@@*
     (@@@(        @                  %@* (@(                 ,@        &@@@*
     (@@@(  .@@@@@@@@            &@@@@@* (@@@@@#            @@@@@@@@   &@@@*
     (@@@#  .%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   &@@@*
     .@@@@.                                                           (@@@@
       @@@@@@%////////////////////////////////////////////////////(&@@@@@@
         #@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@/
$(color "$Gre")
$(hash_header)$(hash_header)
                                  Welcome to the
                             Machine Intelligence Lab
                                      at the
                               University of Florida

                                  (now in ROS2!!)
$(hash_header)$(hash_header)
$(color "$Yel")
This script will help to get your system installed with the needed dependencies.
You shouldn't need to do much - sit back and watch the magic happen before your
eyes!$(color "$Res")
EOF
if [[ "$FROM_SOURCE" = true ]]; then
	echo -e "\n$(color "$Red")!!!! This installation will be from source."
fi

sleep 1

mil_system_install() {
	sudo apt install -y "$@"
}

cat <<EOF
$(color "$Pur")
$(hash_header)
Fetching latest apt packages...
$(hash_header)
EOF

# Install neovim (to prevent cameron from going mad)
sudo apt-add-repository ppa:neovim-ppa/stable -y

# Update apt
sudo apt update

# Installation for virtual machines
# Installs the apt-add-repository command
sudo apt-get install software-properties-common -y

# Installs keyboard config without prompting for input
sudo DEBIAN_FRONTEND=noninteractive apt-get install keyboard-configuration -y # Weird bug

cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Gre")Fetched latest apt packages.
$(color "$Pur")Installing needed Linux dependencies...
$(hash_header)$(color "$Res")

EOF

# System dependencies
mil_system_install apt-utils
mil_system_install --no-install-recommends \
	ca-certificates \
	curl \
	dirmngr \
	doxygen \
	doxygen-doc \
	doxygen-gui \
	expect \
	gnupg \
	gnupg2 \
	graphviz \
	lsb-release \
	neovim \
	python3 \
	python3-pip \
	ruby \
	tzdata \
	wget \
	vim

# Turn on breaking system packages in pip (set to 0 by default in noble)
# Generally, our packages shouldn't break the system, but we will continue to monitor
# this for the future. ROS2 and rosdep have a hard time with virtual environments,
# and using system pip packages in the past has been fine.
sudo python3 -m pip config set global.break-system-packages true

sudo pip3 install -U vcstool

cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Gre")Installed needed Linux dependencies.
$(color "$Pur")Setting up ROS distributions...
$(hash_header)$(color "$Res")

EOF

# Ensure that locales are set up correctly
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add universe
sudo add-apt-repository universe -y

# ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# ROS2 apt source
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

# Install Gazebo apt source
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

install_ros_apt() {
# Pull ROS apt key
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
# Pull gazebo apt key
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key D2486D2DD83DB69272AFE98867170598AF249743

# Setup Gazebo GPG key
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
# Update apt again and install ros
sudo apt update

cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Gre")Set up sources for ROS distributions.
$(color "$Pur")Downloading ROS2 Jazzy Jalisco...
$(hash_header)$(color "$Res")
EOF

# Install MESA drivers for GZ
sudo add-apt-repository ppa:kisak/kisak-mesa -y
sudo apt update -y
sudo apt upgrade -y

mil_system_install ros-jazzy-desktop-full ros-jazzy-ros-gz
# Install additional dependencies not bundled by default with ros
# Please put each on a new line for readability
mil_system_install \
	ros-jazzy-rmw-cyclonedds-cpp \
	ros-jazzy-tf2-sensor-msgs \
	ros-jazzy-geographic-msgs \
	ros-jazzy-vision-msgs \
	ros-jazzy-velodyne \
	ros-jazzy-backward-ros \
	python3-colcon-common-extensions
}

install_ros_src() {
#####################################
# Install ROS2 from source
#####################################

# Install development tools
cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Pur")Installing ROS2 development tools...
$(hash_header)$(color "$Res")
EOF
sudo apt update && sudo apt install -y \
  python3-flake8-blind-except \
  python3-flake8-class-newline \
  python3-flake8-deprecated \
  python3-mypy \
  python3-pip \
  python3-pytest \
  python3-pytest-cov \
  python3-pytest-mock \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-pytest-runner \
  python3-pytest-timeout \
  ros-dev-tools

# Get ROS 2 code
cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Pur")Fetching ROS2 Jazzy source...
$(hash_header)$(color "$Res")
EOF
mkdir -p ~/ros2_jazzy/src
cd ~/ros2_jazzy
vcs import --input https://raw.githubusercontent.com/ros2/ros2/jazzy/ros2.repos src

# Install dependencies using rosdep
cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Pur")Installing ROS2 dependencies...
$(hash_header)$(color "$Res")
EOF
sudo apt upgrade
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

# Build code
cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Pur")Building ROS2...
$(hash_header)$(color "$Res")
EOF
cd ~/ros2_jazzy/
colcon build --symlink-install

# Source
. ~/ros2_jazzy/install/local_setup.bash

#####################################
# Install Gazebo from source
#####################################
cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Pur")Fetching Gazebo dependencies...
$(hash_header)$(color "$Res")
EOF
mil_system_install python3-pip python3-venv lsb-release gnupg curl git

# vcstool and colcon from pip
python3 -m venv $HOME/vcs_colcon_installation
. $HOME/vcs_colcon_installation/bin/activate
pip3 install vcstool colcon-common-extensions

# Getting the sources
cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Pur")Getting Gazebo sources...
$(hash_header)$(color "$Res")
EOF
mkdir -p ~/gazebo_ws/src
cd ~/gazebo_ws/src
curl -O https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-harmonic.yaml
vcs import < collection-harmonic.yaml

# Install all Gazebo dependencies
cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Pur")Installing Gazebo dynamic apt dependencies...
$(hash_header)$(color "$Res")
EOF
cd ~/workspace/src
sudo apt -y install \
  $(sort -u $(find . -iname 'packages-'`lsb_release -cs`'.apt' -o -iname 'packages.apt' | grep -v '/\.git/') | sed '/gz\|sdf/d' | tr '\n' ' ')

# Building the Gazebo Libraries
cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Pur")Building Gazebo...
$(hash_header)$(color "$Res")
EOF
cd ~/gazebo_ws/
colcon graph
colcon build --cmake-args ' -DBUILD_TESTING=OFF' --merge-install

# Source
. ~/gazebo_ws/install/setup.bash
}

if [[ "$FROM_SOURCE" = true ]]; then
	echo "$(color "$Pur")Installing ROS2 and Gazebo from source..."
	install_ros_src
else
	echo "$(color "$Pur")Installing ROS2 and Gazebo from apt..."
	install_ros_apt
fi

cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Gre")Downloaded and installed ROS2 Jazzy Jalisco.
$(color "$Pur")Installing Python dependencies...
$(hash_header)$(color "$Res")
EOF

# Disable "automatic updates" Ubuntu prompt (thanks to https://askubuntu.com/a/610623!)
if which update-manager >/dev/null 2>&1; then
	sudo sed -i 's/Prompt=.*/Prompt=never/' /etc/update-manager/release-upgrades
fi

# Install Python 3 dependencies
sudo pip3 install -r requirements.txt

cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Gre")Downloaded and setup Python dependencies.
$(color "$Pur")Initializing rosdep...
$(hash_header)$(color "$Res")
EOF

# Install Colcon
sudo pip3 install -U colcon-common-extensions

# Initialize rosdep
sudo apt-get install -y python3-rosdep

# Update rosdep
sudo rm -rf /etc/ros/rosdep/sources.list.d/* # Delete this file first - if not deleted, error could be thrown
sudo rosdep init
sudo rosdep update

# If this script is not ~/mil2/scripts/install.sh, throw an error to prevent members
# from installing the repo in the wrong location. Setting ALLOW_NONSTANDARD_DIR=1
# will bypass this check.
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
if [[ $SCRIPT_DIR != "$HOME/mil2/scripts" && -z ${ALLOW_NONSTANDARD_DIR:-} ]]; then
	echo "${Red}Error: This script must be located in ~/mil2/scripts/install.sh. Please review the installation guide and try again.${Res}"
	exit 1
fi
# COLCON_SOURCE_DIR="$HOME/src"

# Clone repository
mil_user_install_dependencies() {
	sudo apt update
	sudo apt install -y \
		git \
		tmux \
		vim \
		htop \
		tmuxinator \
		net-tools \
		cifs-utils \
		nmap \
		fd-find \
		ripgrep \
		fzf \
		aptitude \
		lm-sensors
}

# Add line to user's bashrc which source the repo's setup files
# This allows us to update aliases, environment variables, etc
mil_user_setup_rc() {
	# Line(s) added to ~/.bashrc or ~/.zshrc
	# Note that for backwards compatibility this should not be changed
	# unless you have a very good reason.
	BASH_RC_LINES=". $SCRIPT_DIR/setup.bash"
	if [[ $SHELL == "/usr/bin/zsh" ]]; then
		# User is using zsh
		if grep -Fq "$BASH_RC_LINES" ~/.zshrc; then
			echo "milrc is already sourced in ~/.zshrc, skipping"
		else
			echo "Adding source of milrc to ~/.zshrc"
			{
				echo ""
				echo "# Setup environment for MIL development"
				echo "$BASH_RC_LINES"
			} >>~/.zshrc
		fi
	else
		# User is using zsh
		if grep -Fq "$BASH_RC_LINES" ~/.bashrc; then
			echo "milrc is already sourced in ~/.bashrc, skipping"
		else
			echo "Adding source of milrc to ~/.bashrc"
			{
				echo ""
				echo "# Setup environment for MIL development"
				echo "$BASH_RC_LINES"
			} >>~/.bashrc
		fi
	fi

	# Copies bashrc to interactive login shells (like tmux)
	echo 'if [ -n "$BASH_VERSION" ] && [ -n "$PS1" ]; then
		# include .bashrc if it exists
		if [ -f "$HOME/.bashrc" ]; then
			. "$HOME/.bashrc"
		fi
	fi' >>~/.profile

}

add_hosts_entry() {
	sudo grep -qxF "$1" /etc/hosts || echo "$1" | sudo tee -a /etc/hosts >/dev/null
}

# Add /etc/hosts entry for vehicles
add_hosts_entry "192.168.37.60 sub8"
add_hosts_entry "192.168.37.82 navigator-two"

# Builds the MIL repo
mil_user_setup_init_colcon() {
	cd $SCRIPT_DIR/..
	colcon build
}

cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Gre")Initialized rosdep.
$(color "$Pur")Installing user tools and shell...
$(hash_header)$(color "$Res")
EOF

mil_user_install_dependencies
mil_user_setup_rc
set +u
. /opt/ros/jazzy/setup.bash
. "$SCRIPT_DIR/setup.bash"
set -u

cat <<EOF
$(color "$Pur")
$(hash_header)
$(color "$Gre")Setup user tools and shell.
$(color "$Pur")Compiling repository...
$(hash_header)$(color "$Res")
EOF

touch ~/tmux.conf
if grep 'set -g default-terminal "screen-256color"' ~/tmux.conf; then
	echo "Tmux already has 256 color support, skip this step"
else
	echo 'set -g default-terminal "screen-256color"' >>~/.tmux.conf
fi

mil_user_setup_init_colcon

cat <<EOF
$(color "$Gre")
$(hash_header)
Congratulations! Installation complete.
$(hash_header)
$(color "$Res")
EOF
