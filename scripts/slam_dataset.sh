cd ~
mkdir -p Datasets/EuRoc
cd Datasets/EuRoc/
wget -c http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.zip
mkdir MH01
unzip MH_01_easy.zip -d MH01/
cd ~/Datasets/EuRoc/MH01/mav0/cam0/data
rm 1403636689613555456.png
cp 1403636689663555584.png 1403636689613555456.png
rm 1403636722213555456.png
cp 1403636722263555584.png 1403636722213555456.png
