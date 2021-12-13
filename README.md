# Overview
The kodlab_mjbots_sdk has a few key key features added to the 
pi3_hat library developed by mjbots: https://github.com/mjbots/pi3hat
1. Cross compiling support from ubuntu 20.04 to Raspberry pi
2. Integration with NYU realtime_tools library: https://github.com/machines-in-motion/real_time_tools 
for better realtime performance
3. Integration with LCM (https://lcm-proj.github.io/) for remote logging and remote input to the robot
4.  The `mjbots_control_loop` object which handles the structure of the control loop for
the easy creation of new controllers
5. The `mjbots_robot_interface` which provides a convenient interface for communicating with any number
of moteus motor controllers 

Note: This library only supports torque commands. If you wish to use
position control, you either must close the loop yourself or modify the 
library to allow for the position loop to run on the moteus.

# Usage
## mjbots_control_loop:
To use the `mjbots_control_loop` create a class which inherits the `mjbots_control_loop`
and implements the implements `calc_torque` to set the torques in the robot object. 

    class Controller : public Mjbots_Control_Loop{
      using Mjbots_Control_Loop::Mjbots_Control_Loop;
      void calc_torques() override{
        std::vector<float> torques = control_effort;
        m_robot->set_torques(torques);
      }    
    };

## Accessing robot state
To access the robot state use `m_robot->get_joint_positions()` or `m_robot->get_joint_velocities()`
or `m_robot->get_torque_cmd()`

## Logging
To add logging to your robot either use one of the provided lcm objects or create your own, build
lcm data types with the provided script, and then include the relevant header. Then when defining
the child class of the `mjbots_control_loop` add the template argument of the lcm class.
 
    class Controller : public Mjbots_Control_Loop<lcm_type>
 
Next implement the `prepare_log` to add data to the logging object.

      void prepare_log()  override{
        m_log_data.data = data;
      }
 
Finally when creating the instance of the class set the `m_log_channel_name` option in the option struct.

      options.m_log_channel_name = "motor_data";
      Controller control_loop(options)

## Soft start
To configure the soft start, set the `options.m_max_torque` and `options.m_soft_start_duration`. Where the
max torque is the maximum torque per motor and the soft start duration is how long the torque ramp should last
in iterations of the control loop. 

# Setup

## Setting up your Pi
* Use a realtime pi kernel and flash sd card `https://github.com/guysoft/RealtimePi`
* Configure ubuntu local network to share wifi and connect pi over ethernet
* Use nmap to find pi ipaddress `sudo nmap -sn IP/24`
* ssh onto pi `ssh pi@IP`, password is raspberry
* Change the pi password to something that you will remember
* scp setup script onto pi `scp <path to kodlab_mjbots_sdk>/utils/setup-system.py pi@IP:~/`
* scp performance governer onto pi `scp <path to kodlab_mjbots_sdk>/utils/performance_governor.sh pi@IP:~/`
* run setup script (change password and ssid in setup script) `sudo python3 setup-system.py`
* run performance governor script
* Reboot pi
* Add pi to `etc/hosts` to make ssh easier
* Add ssh key
* run rsync command to get libraries onto computer (see below)


## Laptop Toolchain 
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


## Laptop LCM
* Download lcm from git
* Try to make lcm - java issue can be fixed here: https://github.com/lcm-proj/lcm/issues/241 
* Install python with
    
    
    cd ../lcm-python
    
    sudo python3 setup.py install
* Add `export PYTHONPATH="${PYTHONPATH}:$HOME/mjbots/kodlab_mjbots_sdk"` to your `~/.bashrc`
* run `./scripts/make_lcm.sh` to generate lcm files. You will need to rerun this command each time you change an lcm definition.
* Install libbot2 from `https://github.com/KodlabPenn/libbot2`

* On the host computer to setup the connection run `bot-lcm-tunnel <PI-IP/hostname>`. From here you can start m_logging with


      lcm-logger


# Building
Current command to build clean is

    cd .. && rm -R build/ && mkdir build && cd build/ && cmake .. -DCMAKE_TOOLCHAIN_FILE=~/mjbots/kodlab_mjbots_sdk/cmake/pi.cmake && make

Normal build is 

    cmake .. -DCMAKE_TOOLCHAIN_FILE=~/mjbots/kodlab_mjbots_sdk/cmake/pi.cmake
    
