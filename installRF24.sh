#!/bin/sh

echo "Make sure to enable SPIDEV first"

sudo apt update
sudo apt upgrade

cd ~
mkdir -p ece445

# Download BCM2835 library
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.57.tar.gz
tar xvf bcm2835-1.57

# Remove zip file
rm bcm2835-1.57.tar.gz

cd bcm2835-1.57.tar.gz
sudo make install -B
cd ..

# Building RF24 using SPIDEV
git clone https://github.com/nRF24/RF24
cd RF24
./configure --driver=SPIDEV
sudo make install -B
cd ..

# Install RF24Network
wget https://github.com/TMRh20/RF24Network/archive/Development.zip
unzip RF24Network-Development.zip
rm RF24Network-Development.zip
cd RF24Network-Development
sudo make install
cd ..

# Install RF24Mesh
wget https://github.com/TMRh20/RF24Mesh/archive/master.zip
unzip RF24Mesh-master.zip
rm RF24Mesh-master.zip
cd RF24Mesh-master
sudo make install
cd ..


