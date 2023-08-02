# Usage

## MjbotsControlLoop:
To use the Mjbots control loop, create a class which inherits the 
`MjbotsControlLoop` object and implements `Update` to set the torques in 
the robot object as follows.

```cpp
class MyControlLoop : public kodlab::mjbots::MjbotsControlLoop
{
  using MjbotsControlLoop::MjbotsControlLoop;
  void Update() override{
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

## PD Set points and gains
The SDK is built around just sending ffwd torque commands, but the moteus does have an onboard 
PD loop. In order to use the built in PD loop modify the PD gains on the moteus. Next when setting
up the control loop set:

```cpp
kodlab::mjbots::ControlLoopOptions options;
options.send_pd_commands = true;
```
By setting `send_pd_commands` to true the robot will now send pd scales and set points to the
moteus. This will slow down the communication with the moteus, so only use this option if you
are actually going to use the pd commands.

In order to set the PD gains and set points next when constructing the joint vector set the value
of `moteus_kp` and `moteus_kd` to the values configured on the moteus. Now in the update function
you can use 

```
    robot_->joints[0]->set_joint_position_target(0);
    robot_->joints[0]->set_joint_velocity_target(0);
    robot_->joints[0]->set_kp(0.8);
    robot_->joints[0]->set_kd(0.01);
```
to set gains and targets for the onboard pd loop.

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
`Update` method.  They should still override the `PrepareLog` and `ProcessInput`
methods if they are using control-loop-level LCM logging or inputs.  A simple
`MjbotsBehaviorLoop` implementation with behavior selection input would look 
like the following.

```cpp
class UserBehaviorLoop : public kodlab::mjbots::MjbotsBehaviorLoop<VoidLcm, 
    UserInput, UserRobot> {
  
  using kodlab::RobotBase::RobotBase;
  
  void ProcessInput(const UserInput &input_data) override {
    // Set behavior from `input_data`
    SetBehavior(input_data.behavior);
  }
  
};
```

An example demonstrating usage of the `MjbotsBehaviorLoop` is provided in 
`examples/behavior_robot_example.cpp`.

## Soft Start
Each joint has its own soft start. To configure the soft start set the `MoteusJointConfig.max_torque`
to the maximum allowable torque for the motor and `MoteusJointConfig.soft_start_duration_ms` to the
duration of the soft start. The soft start will ramp the maximum torque from 0 to `max_torque` over `soft_start_duration_ms`.
Once the time is greater than `soft_start_duration_ms`, torque will be limited to `max_torque`.

## Console Logging
The `log.h` header provides a set of debug logging macros with adjustable
logging severity levels.  In order of increasing severity, the levels are 
`TRACE`, `DEBUG`, `INFO`, `NOTICE`, `WARN`, `ERROR`, and `FATAL`.  

The minimum severity level for console output is set by defining the
`LOG_MIN_SEVERITY` macro (default is `SEVERITY_ALL`).  Setting 
`LOG_MIN_SEVERITY` to `SEVERITY_NONE` will disable macro console output.

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

Verbose logging commands are provided for all logging macros, and take the form
`VLOG_XXXX` or `VLOG_IF_XXXX`.  The default output of the log and verbose log 
macros is as follows. 
```
[SEVERITY] Log message
[SEVERITY][path/to/file | function:line_no] Verbose log message
```
The output behavior can be changed by redefining the `LOG_ARGS`, `LOG_FORMAT`,
`VLOG_ARGS`, and `VLOG_FORMAT` macros.  For example, to produce verbose output 
of the form 
```
[SEVERITY][path/to/file][line_no][function] Verbose log message
```
the verbose macros would be redefined as follows
```cpp
#define VLOG_ARGS(tag) tag, __FILE__, __LINE__, __func__
#define VLOG_FORMAT "[%-6s][%-15s][%d][%s] "
```

Colored terminal output is provided by default via
[ANSI escape codes](https://gist.github.com/fnky/458719343aabd01cfb17a3a4f7296797).
This can be disabled by defining the `NO_COLOR` macro.


