#!/bin/sh

# Update and get the toolchain
sudo apt-add-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get dist-upgrade -y

# Install things we need for running binaries
sudo apt-get install -y \
    patchelf \
    g++-6 \
    gcc-6 \

# Install things we need for building ptpd
sudo apt-get install -y \
    autoconf \
    automake \
    libtool \
    libpcap-dev \
    libsnmp-dev

# On the e38 get hostapd dnsmasq iptables-persistent wpa_supplicant

# Download ptpd and extract and install
# It is used to sync the clocks of the comptuers to microsecond accuracy (hopefully)
wget https://github.com/ptpd/ptpd/archive/ptpd-2.3.1.tar.gz
tar xf ptpd-2.3.1.tar.gz
rm ptpd-2.3.1.tar.gz
cd ptpd-ptpd-2.3.1
autoreconf --install
mkdir build
cd build
../configure
make -j4
sudo make install
cd ../..
rm -rf ptpd-ptpd-2.3.1


# Enable the various services so they automatically run
# The reverse of this can be done (with disable) to stop them running automatically
systemctl enable nuc1.service
systemctl enable nuc1_sensors.service
systemctl enable nuc2.service
systemctl enable nuc3.service
systemctl enable nuc4.service
systemctl enable nuc4_sensors.service

systemctl enable e38_propulsion.service
systemctl enable e38_gnc.service
