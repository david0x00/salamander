#!/bin/sh

echo "Make sure to enable SPIDEV first"

#sudo apt update
#sudo apt upgrade

cd ~
mkdir -p ece445
cd ~/ece445
pwd
# Download BCM2835 library
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.57.tar.gz
tar xvf bcm2835-1.57.tar.gz

# Remove zip file
rm bcm2835-1.57.tar.gz

cd bcm2835-1.57
./configure
make
sudo make check
sudo make install
cd ..

# Building RF24 using SPIDEV
git clone https://github.com/nRF24/RF24
cd RF24
./configure --driver=SPIDEV
sudo make install -B
cd ..

# Install RF24Network
wget https://github.com/TMRh20/RF24Network/archive/Development.zip
unzip Development.zip
rm Development.zip
cd RF24Network-Development
sudo make install
cd ..

# Install RF24Mesh
#wget https://github.com/TMRh20/RF24Mesh/archive/master.zip
wget -O master.zip https://github.com/nRF24/RF24Mesh/archive/v1.0.51.zip
unzip master.zip
rm master.zip
cd RF24Mesh-master
sudo make install
cd ..


