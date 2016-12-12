#!/bin/sh

# Update and get the toolchain
sudo apt-add-repository ppa:ubuntu-toolchain-r/test
sudo apt-get update
sudo apt-get dist-upgrade -y

# Install things we need
sudo apt-get install \
    patchelf \
    g++-6 \
    gcc-6 \
    pptpd

# Enable the autopilot service
systemctl enable autopilot.service
