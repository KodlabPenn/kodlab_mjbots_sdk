# Setup
#Toolchain
Taken from https://stackoverflow.com/questions/19162072/how-to-install-the-raspberry-pi-cross-compiler-on-my-linux-host-machine/58559140#58559140

Download the toolchain

    wget https://github.com/Pro/raspi-toolchain/releases/latest/download/raspi-toolchain.tar.gz

Extract the toolchain

    sudo tar xfz raspi-toolchain.tar.gz --strip-components=1 -C /opt

Get the libraries

Add the following lines to your `~/.bashrc`

    export RASPBIAN_ROOTFS=$HOME/raspberry/rootfs
    export PATH=/opt/cross-pi-gcc/bin:$PATH
    export RASPBERRY_VERSION=1