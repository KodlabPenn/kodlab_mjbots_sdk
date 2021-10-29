# Setup
#Toolchain
Taken from https://stackoverflow.com/questions/19162072/how-to-install-the-raspberry-pi-cross-compiler-on-my-linux-host-machine/58559140#58559140

Download the toolchain

    wget https://github.com/Pro/raspi-toolchain/releases/latest/download/raspi-toolchain.tar.gz

Extract the toolchain

    sudo tar xfz raspi-toolchain.tar.gz --strip-components=1 -C /opt

Get the libraries

    rsync -vR --progress -rl --delete-after --safe-links pi@192.168.1.PI:/{lib,usr,opt} $HOME/raspberrypi/rootfs
The command took about 5 minutes on my computer

Add the following lines to your `~/.bashrc`

    export RASPBIAN_ROOTFS=$HOME/raspberry/rootfs
    export PATH=/opt/cross-pi-gcc/bin:$PATH
    export RASPBERRY_VERSION=1

#Pi
* Use realtime pi
* Configure ubuntu local network to share wifi
* Use nmap to find pi ipaddress
* ssh onto pi
* scp setup script
* run setup script (change password and ssid) `sudo python3 setup-system.py`


#LCM
* Download lcm from git
* Try to make lcm - java issue can be fixed here: https://github.com/lcm-proj/lcm/issues/241 

#Building
Current command to build clean is

    cd .. && rm -R build/ && mkdir build && cd build/ && cmake .. -DCMAKE_TOOLCHAIN_FILE=~/mjbots/kodlab_sdk/external/pi.cmake && make