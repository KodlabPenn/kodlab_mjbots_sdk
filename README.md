# Overview
The kodlab_mjbots_sdk has a few key key features added to the 
pi3_hat library developed by mjbots: https://github.com/mjbots/pi3hat
1. Cross compiling support from ubuntu 20.04 to Raspberry pi
2. Integration with NYU realtime_tools library: https://github.com/machines-in-motion/real_time_tools 
for better realtime performance
3. Integration with LCM (https://lcm-proj.github.io/) for remote logging and remote input to the robot
4.  The `MjbotsControlLoop` object which handles the structure of the control loop for
the easy creation of new controllers
5. The `MjbotsHardwareInterface` which provides a convenient interface for communicating with any number
of moteus motor controllers 

Note: This library only supports torque commands. If you wish to use
position control, you either must close the loop yourself or modify the 
library to allow for the position loop to Run on the moteus.

#Important
In order to keep the message size down, kp and kd on the motors must be set to 0

# Usage
## MjbotsControlLoop:
To use the `MjbotsControlLoop` create a class which inherits the `MjbotsControlLoop`
and implements `Update` to set the torques in the robot object. 

    class Controller : public MjbotsControlLoop{
      using MjbotsControlLoop::MjbotsControlLoop;
      void Update() override{
        std::vector<float> torques = control_effort;
        robot_->SetTorques(torques);
      }    
    };

## Accessing robot state
To access the robot state use `robot_->GetJointPositions()` or `robot_->GetJointVelocities()`
or `robot_->GetTorqueCmd()`

## Logging
To add logging to your robot either use one of the provided lcm objects or create your own, build
lcm data types with the provided script, and then include the relevant header. Then when defining
the child class of the `MjbotsControlLoop` add the template argument of the lcm class.
 
    class Controller : public MjbotsControlLoop<lcm_type>
 
Next implement the `PrepareLog` to add data to the logging object.

      void PrepareLog()  override{
        log_data_.data = data;
      }
 
Finally when creating the instance of the class set the `log_channel_name` option in the option struct.

      options.log_channel_name = "example";
      Controller control_loop(options)
      
To log data, on your laptop Start the bot lcm tunnel with `bot-lcm-tunnel <IP>` and Start logging using `lcm-logger`

## Input LCM Communication
In order to set gains during run time or to communicate between your laptop and the robot, first define the LCM data
type you would like to use, then build lcm types. Next when defining the child class of 
`MjbotsControlLoop` add the input template argument of the lcm class along with the logging lcm class.

    class Controller : public MjbotsControlLoop<LcmLog, LcmInput>

Next, implement the `ProcessInput` function to do things with the data in `lcm_sub_.data_`

      void ProcessInput()  override{
        gains_ = lcm_sub_.data_.gains;
      }

## Soft Start
To configure the soft Start, set the `options.max_torque` and `options.soft_start_duration`. Where the
max torque is the maximum torque per motor and the soft Start duration is how long the torque ramp should last
in iterations of the control loop. 

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

Create the rootfs folder in `$HOME/raspberry/rootfs`

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

# Building
Current command to build clean is

    cd .. && rm -R build/ && mkdir build && cd build/ && cmake .. -DCMAKE_TOOLCHAIN_FILE=<path to sdk>/cmake/pi.cmake && make

Normal build is 

    cmake .. -DCMAKE_TOOLCHAIN_FILE=~/mjbots/kodlab_mjbots_sdk/cmake/pi.cmake

# Running Code
To Run code, first scp the binary onto the pi, and then Run it as sudo using:

    sudo ./code_binary

# Citation
To cite this repo please use the information in the CITATION.cff file

# Acknowledgement
This work was supported by ONR grant #N00014- 16-1-2817, a Vannevar Bush Fellowship held by Daniel Koditschek,
sponsored by the Basic Research Office of the Assistant Secretary of Defense for Research and Engineering.
