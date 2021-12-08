# Setup

## Setting up your Pi
* Use a realtime pi kernel and flash sd card `https://github.com/guysoft/RealtimePi`
* Configure ubuntu local network to share wifi and connect pi over ethernet
* Use nmap to find pi ipaddress `sudo nmap -sn IP/24`
* ssh onto pi `ssh pi@IP`, password is raspberry
* scp setup script onto pi `scp <path to kodlab_mjbots_sdk>/utils/setup-system.py pi@IP:~/`
* scp performance governer onto pi `scp <path to kodlab_mjbots_sdk>/utils/performance_governor.sh pi@IP:~/`
* run setup script (change password and ssid in setup script) `sudo python3 setup-system.py`
* Install pi3hat library
* run performance governor script
* install lcm https://lcm-proj.github.io/build_instructions.html
* Install libbot2 from `https://github.com/libbot2/libbot2`
    * I've found its helpful to remove `bot2-vis`, `bot2-lcmgl`, and `bot2-frames` from `tobuild.txt` since they have lots of dependencies and we won't be using them
    * create build directory and then run: `sudo make BUILD_PREFIX=/usr/local`
* On the pi, add the following to you `/etc/rc.local`
    

    ifconfig wlan0 multicast
    sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev wlan0
    bot-lcm-tunnel
* Open `/etc/ld.so.conf` and add to end`/usr/local/lib` (might be missing step here)
* Reboot pi
* Add pi to `etc/hosts` to make ssh easier
* Add ssh key
* run rsync command to get libraries onto computer (see below)


## Toolchain - Run on your computer
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


## LCM - Run on your computer
* Download lcm from git
* Try to make lcm - java issue can be fixed here: https://github.com/lcm-proj/lcm/issues/241 
* Install python with
    
    
    cd ../lcm-python
    
    sudo python3 setup.py install
* Add `export PYTHONPATH="${PYTHONPATH}:$HOME/mjbots/kodlab_mjbots_sdk"` to your `~/.bashrc`
* run `./scripts/make_lcm.sh` to generate lcm files. You will need to rerun this command each time you change an lcm definition.
* Install libbot2 from `https://github.com/libbot2/libbot2`

* On the host computer to setup the connection run `bot-lcm-tunnel <PI-IP/hostname>`. From here you can start logging with


      lcm-logger


## Building
Current command to build clean is

    cd .. && rm -R build/ && mkdir build && cd build/ && cmake .. -DCMAKE_TOOLCHAIN_FILE=~/mjbots/kodlab_mjbots_sdk/cmake/pi.cmake && make

Normal build is 

    cmake .. -DCMAKE_TOOLCHAIN_FILE=~/mjbots/kodlab_mjbots_sdk/cmake/pi.cmake
    
