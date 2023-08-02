# Setup
This setup sets up the PI as an access point and sets up cross compiling on the main computer.
## Setting up your Pi
The setup instructions are for headless setup. If you are not comfortable with the headless operation
of a Pi, now is a good time to start. This setup will install the dependencies on the Pi and turn the
Pi into an access point/hotspot for easy connection to later on.
* Use a realtime pi kernel and flash sd card `https://github.com/guysoft/RealtimePi`
* Configure the ethernet on your laptop to share internet with the pi. 
    * The goal of this setup is to be able to ssh over ethernet onto the pi, and to give the pi access
    to the internet without connecting it to a wifi network
* With the PI power on and connected to your laptop via ethernet, use nmap on your laptop to find pi ipaddress `sudo nmap -sn IP/24`
* ssh onto pi `ssh pi@IP`, password is raspberry
* Change the pi password to something that you will remember
* From your laptop, scp setup script onto pi `scp <path to kodlab_mjbots_sdk>/utils/setup-system.py pi@IP:~/`
* From your laptop scp performance governer onto pi `scp <path to kodlab_mjbots_sdk>/utils/performance_governor.sh pi@IP:~/`
* Edit the setup script on the pi with your desired wifi ssid, wifi password, and wifi ip address
* On the Pi, run setup script  `sudo python3 setup-system.py`
    * This will install dependencies, make the operating system even more realtime, and setup the wifi access point
* On the Pi, run performance governor script via sudo
* Reboot pi
* Add pi to `etc/hosts` on your laptop to make ssh easier
* Add ssh key
* Run rsync command to get libraries onto computer (see below)


## Laptop Toolchain 
Taken from https://stackoverflow.com/questions/19162072/how-to-install-the-raspberry-pi-cross-compiler-on-my-linux-host-machine/58559140#58559140

Download the toolchain

    wget https://github.com/Pro/raspi-toolchain/releases/latest/download/raspi-toolchain.tar.gz

Extract the toolchain on your laptop

    sudo tar xfz raspi-toolchain.tar.gz --strip-components=1 -C /opt

Create the rootfs folder in `$HOME/raspberrypi/rootfs`

Get the libraries

    rsync -vR --progress -rl --delete-after --safe-links pi@<PI_IP>:/{lib,usr,opt} $HOME/raspberrypi/rootfs

Add the following lines to your `~/.bashrc` on your laptop, making sure your raspberry pi version is correct.

    export RASPBIAN_ROOTFS=$HOME/raspberrypi/rootfs
    export PATH=/opt/cross-pi-gcc/bin:$PATH
    export RASPBERRY_VERSION=4


## Laptop LCM
* Download lcm from git and install using make: https://lcm-proj.github.io/build_instructions.html
* If you have the java issue, it can be fixed here: https://github.com/lcm-proj/lcm/issues/241 
* Install lcm python with
    
    
    cd ../lcm-python
    
    sudo python3 setup.py install
* Add `export PYTHONPATH="${PYTHONPATH}:<path to sdk>/kodlab_mjbots_sdk"` to your `~/.bashrc`
* From the `kodlab_mjbots_sdk` repo, run `./scripts/make_lcm.sh` to generate lcm files. You will need to rerun this command each time you change an lcm definition.
* Install libbot2 from `https://github.com/KodlabPenn/libbot2`

* On the host computer to setup the connection Run `bot-lcm-tunnel <PI-IP or hostname>`. From here you can Start logging_ with


      lcm-logger

## Submodules
This repo uses submodules to set them up run the following commands from the repo folder
  
    git submodule init
    git submodule update

## Motor Setup
This section is a work in progress. Currently in order to setup the motors, we set the following parameters on the moteus:
* `servo.pid_position.kp`, `servo.pid_position.kd`, `servo.pid_position.ki` = `0`
* `servopos.position_min`,`servopos.position_max` = `nan`
* id

The pid gains are set to zero to keep the torque packet size down. Servo pos max and min are disabled since they can 
potentially cause confusing faults if you don't understand them.
