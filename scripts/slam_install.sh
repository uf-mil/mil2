# Install dependencies
sudo apt update

sudo apt-get install build-essential -y
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev -y

sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev -y

sudo apt-get install libglew-dev libboost-all-dev libssl-dev -y
sudo apt-get install libepoxy -y
sudo apt install libeigen3-dev -y

# Install OpenCV
sudo apt update
sudo apt install libopencv-dev -y


# Install Pangolin
cd ../ext
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
./scripts/install_prerequisites.sh recommended

# Configure and build
cmake -B build
cmake --build build
cd build/
sudo make install
sudo ldconfig

# Install ORB-SLAM3
cd ../..
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3/
sed -i 's/++11/++14/g' CMakeLists.txt
chmod +x build.sh
./build.sh
