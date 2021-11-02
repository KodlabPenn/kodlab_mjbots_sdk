# Setup

## Pi
* Use realtime pi `https://github.com/guysoft/RealtimePi`
* Configure ubuntu local network to share wifi and connect pi over ethernet
* Use nmap to find pi ipaddress `sudo nmap -sn IP/24`
* ssh onto pi `ssh pi@IP`
* scp setup script onto pi `scp <path to kodlab_mjbots_sdk>/utils/setup-system.py pi@IP:~/`
* run setup script (change password and ssid) `sudo python3 setup-system.py`
* install bcm_host

        sudo apt-get install libraspberrypi-dev raspberrypi-kernel-headers
* Install cmake, libglib2.0-dev, 
* scp lcm zip onto pi
* install lcm https://lcm-proj.github.io/build_instructions.html
* Open `/etc/ld.so.conf` and add to end`/usr/local/lib`
* add the following line to network interface `up route add -net 224.0.0.0 netmask 240.0.0.0 dev lo`
* install boost onto pi `sudo apt-get install libboost-all-dev`
* Reboot pi
* Add pi to `etc/hosts` to make ssh easier
* Add ssh key
* run rsync command to get libraries onto computer (see below)


## Toolchain
Taken from https://stackoverflow.com/questions/19162072/how-to-install-the-raspberry-pi-cross-compiler-on-my-linux-host-machine/58559140#58559140

Download the toolchain

    wget https://github.com/Pro/raspi-toolchain/releases/latest/download/raspi-toolchain.tar.gz

Extract the toolchain

    sudo tar xfz raspi-toolchain.tar.gz --strip-components=1 -C /opt

Get the libraries

    rsync -vR --progress -rl --delete-after --safe-links pi@192.168.1.PI:/{lib,usr,opt} $HOME/raspberrypi/rootfs

Add the following lines to your `~/.bashrc`

    export RASPBIAN_ROOTFS=$HOME/raspberry/rootfs
    export PATH=/opt/cross-pi-gcc/bin:$PATH
    export RASPBERRY_VERSION=1


## LCM
* Download lcm from git
* Try to make lcm - java issue can be fixed here: https://github.com/lcm-proj/lcm/issues/241 
* Install python with
    
    
    cd ../lcm-python
    
    sudo python3 setup.py install
    
## Building
Current command to build clean is

    cd .. && rm -R build/ && mkdir build && cd build/ && cmake .. -DCMAKE_TOOLCHAIN_FILE=~/mjbots/kodlab_mjbots_sdk/cmake/pi.cmake && make

Normal build is 
    cmake .. -DCMAKE_TOOLCHAIN_FILE=~/mjbots/kodlab_mjbots_sdk/cmake/pi.cmake