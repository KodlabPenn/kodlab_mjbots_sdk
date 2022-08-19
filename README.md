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
6. The `RobotBase` class which provides an interface for updating robot state and joint torques.

Note: This library only supports torque commands. If you wish to use
position control, you either must close the loop yourself or modify the 
library to allow for the position loop to Run on the moteus.

# Important
In order to keep the message size down, kp and kd on the motors must be set to 0

# Usage

## MjbotsControlLoop:
To use the Mjbots control loop, create a class which inherits the 
`MjbotsControlLoop` object and implements `Update` to set the torques in 
the robot object as follows.

```cpp
class MyControlLoop : public kodlab::mjbots::MjbotsControlLoop
{
  using MjbotsControlLoop::MjbotsControlLoop;
  void CalcTorques() override{
    std::vector<float> torques = control_effort;
    robot_->SetTorques(torques);
  }    
};
```

A simple example using the `MjbotsControlLoop` is provided in
`examples/spint_joints_example.cpp`.  The `MjbotsControlLoop` is optionally templated with an LCM
log type, an LCM input type, and a `RobotBase`-derived class. These are
described below.

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
      
To log data, on your laptop Start the bot lcm tunnel with `bot-lcm-tunnel <IP>` and Start logging using `lcm-logger`.
Refer again to `examples/spin_joints_example.cpp` for an example implementation.

## Input LCM Communication
In order to set gains during run time or to communicate between your laptop and the robot, first define the LCM data
type you would like to use, then build lcm types. Next when defining the child class of 
`MjbotsControlLoop` add the input template argument of the lcm class along with the logging lcm class.

    class Controller : public MjbotsControlLoop<LcmLog, LcmInput>

Next, implement the `ProcessInput` function to do things with the data in `lcm_sub_.data_`

      void ProcessInput()  override{
        gains_ = lcm_sub_.data_.gains;
      }

## Robot Base
The `RobotBase` object is intended to be inherited by a user-defined robot 
class.  The derived class should implement an override of 
`RobotBase::Update()`.  Note that this new `Update()` function must 
increment the cycle count.  A simple implementation follows.

```cpp
class MyRobot : virtual public kodlab::RobotBase
{
  using kodlab::RobotBase::RobotBase;

public:
  int mode = 0; // Member variables encode whatever added state we need
  // Set up robot update function for state and torques
  void Update() override
  { 
    cycle_count_++;
    std::vector<float> torques(num_joints_, 0);
    SetTorques(torques);
  }
};
```

The corresponding control loop definition would be.

```cpp
class MyController : public MjbotsControlLoop<LcmLog, LcmInput, MyRobot>
```

Refer to `include/examples/simple_robot.h` for a sample robot class and 
`examples/robot_example.cpp` for a usage example.

## Behaviors
The `Behavior` abstract class can be derived by the user and used to define 
custom behaviors.  The abstract class includes functions for behavior 
initialization, startup, updating, and stopping, as well as methods for 
declaring whether the behavior is prepared to transition to another behavior.  
For a user-defined behavior, this would look like the following, where 
`UserBehavior` is the new behavior and `UserRobot` is the user's `RobotBase`
child robot class.
```cpp
class UserBehavior : public kodlab::Behavior<UserRobot>
```
If LCM inputs and/or outputs are desired on a behavior level, the user should 
instead make a child of the `IOBehavior` class.  For example, the following 
defines `UserIOBehavior` which runs on `UserRobot` from above, receives inputs
through a `UserInput` LCM message, and logs outputs via a `UserOutput` LCM 
message.
```cpp
class UserIOBehavior : public kodlab::IOBehavior<UserRobot, UserInput, UserOutput>
```

Refer to the `SimpleSpinJointsBehavior` class defined in 
`include/examples/simple_spin_joints_behavior.h` for an example of how the 
behavior class can be implemented, and the `SimpleControlIOBehavior` in 
`include/examples/simple_control_io_behavior.h` for an example of implementing
a behavior with inputs and outputs.

## Behavior Manager
The `BehaviorManager` class is a container for storing and running 
`Behavior`-derived behaviors.  This class maintains a default behavior at the 
beginning index in its internal vector. Additional behaviors can be appended to
the vector and the default behavior can be set by the user. The 
`BehaviorManager` can be composed into a child class of `MjbotsControlLoop` 
and used to maintain a series of behaviors running on a`RobotBase`-derived 
robot. 

## Mjbots Behavior Loop
The `MjbotsBehaviorLoop` extends the [`MjbotsControlLoop`](https://github.com/KodlabPenn/kodlab_mjbots_sdk#mjbotscontrolloop)
to include a `BehaviorManager` which internally manages `Behavior` objects for a
`RobotBase`-derived class.  The `MjbotsBehaviorLoop` works in much the same way
as the `MjbotsControlLoop`, except the user no longer needs to override the
`Update()` method.  An example demonstrating usage of the `MjbotsBehaviorLoop`
is provided in `examples/behavior_robot_example.cpp`.

## Soft Start
To configure the soft Start, set the `options.max_torque` and `options.soft_start_duration`. Where the
max torque is the maximum torque per motor and the soft Start duration is how long the torque ramp should last
in iterations of the control loop. 

## Console Logging
The `log.h` header provides a set of debug logging macros with adjustable
logging severity levels.  In order of increasing severity, the levels are 
`DEBUG`, `INFO`, `WARN`, `ERROR`, and `FATAL`.  

The minimum level for console output is set by defining `LOG_MIN_SEVERITY` 
(default is `DEBUG`).  Setting `LOG_MIN_SEVERITY` to `NONE` will disable macro 
console output.

Usage of the `LOG_XXXX` logging macros (where `XXXX` is `DEBUG`, `INFO`,
etc.) is akin to using [`std::fprintf`](https://en.cppreference.com/w/cpp/io/c/fprintf).
For example,
```cpp
LOG_WARN("This is a warning message.");
LOG_ERROR("%s", "This is an error message.");
```

Conditional logging macros `LOG_IF_XXXX` are also provided, which take a leading
conditional argument before the standard `std::fprintf` input.  For example,
```cpp
LOG_IF_INFO(false, "%s", "This info message will not be logged.");
LOG_IF_FATAL(true, "This fatal message will be logged.");
```

The default output of the `LOG_*` macros follows the format 
```
[SEVERITY][path/to/file | function:line_no] Logged message
```
The output behavior can be changed by redefining the `LOG_ARGS` and `LOG_FORMAT` 
macros.  For example, to produce output of the form 
```
[SEVERITY][path/to/file][line_no][function] Logging message
```
these macros would be redefined as follows
```cpp
#define LOG_ARGS(tag) tag, __FILE__, __LINE__, __func__
#define LOG_FORMAT "[%-5s][%-15s][%d][%s] "
```

Colored terminal output is provided by default via
[ANSI escape codes](https://gist.github.com/fnky/458719343aabd01cfb17a3a4f7296797).
This can be disabled by defining the `NO_COLOR` macro.



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
