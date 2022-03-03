#include joint.h
#include "kodlab_mjbots_sdk/moteus_protocol.h"
#include "kodlab_mjbots_sdk/pi3hat_moteus_interface.h"

/**
 * @brief 
 * Moteus Joint class that includes all the motor specific vectors. 
 */
class JointMoteus:public JointBase{
    /**
     * @brief 
     * 
     */
    public:
        const int can_bus;
        const int can_id;

        mjbots::moteus::Mode mode_ = ::mjbots::moteus::Mode::kStopped;
        mjbots::moteus::Pi3HatMoteusInterface::ServoCommand command;
        mjbots::moteus::PositionResolution res;
        mjbots::moteus::QueryCommand query;
        mjbots::moteus::QueryResult reply_;

        JointMoteus(int can_bus__,int can_id__,
            bool direction = 0, 
            float zero_offset = 0,
            float max_torque = std::numeric_limits<float>::infinity(),
            float gear_ratio = 1.0, 
            float pos_min = -std::numeric_limits<float>::infinity(), 
            float pos_max = std::numeric_limits<float>::infinity(),
            :JointBase(direction,zero_offset,max_torque,gear_ratio,pos_min,pos_max),
            can_bus(can_bus__), can_id(can_id__){
                    // Default resolution is designed for fast torque control
                    // of a 12 DoF robot (ignoring everything else)
                    res.position = ::mjbots::moteus::Resolution::kIgnore;
                    res.velocity = ::mjbots::moteus::Resolution::kIgnore;
                    res.feedforward_torque = ::mjbots::moteus::Resolution::kInt16;
                    res.kp_scale = ::mjbots::moteus::Resolution::kIgnore;
                    res.kd_scale = ::mjbots::moteus::Resolution::kIgnore;
                    res.maximum_torque = ::mjbots::moteus::Resolution::kIgnore;
                    res.stop_position = ::mjbots::moteus::Resolution::kIgnore;
                    res.watchdog_timeout = ::mjbots::moteus::Resolution::kIgnore;

                    // Minimal query reads for speed 
                    query.mode = Resolution::kInt8;
                    query.position = Resolution::kInt16;
                    query.velocity = Resolution::kInt16;
                    query.torque = Resolution::kIgnore;
                    query.q_current = Resolution::kIgnore;
                    query.d_current = Resolution::kIgnore;
                    query.rezero_state = Resolution::kIgnore;
                    query.voltage = Resolution::kIgnore;
                    query.temperature = Resolution::kIgnore;
                    query.fault = Resolution::kIgnore;
        }
        
        void updateMoteus(::mjbots::moteus::QueryResult servo_reply){
            reply_ = servo_reply;
            updateMoteus(servo_reply.position,servo_reply.velocity, servo_reply.mode);
        }
        void updateMoteus(float reply_pos, float reply_vel, ::mjbots::moteus::Mode mode ){
            position_ = direction * (raw_pos * 2 * M_PI) + zero_offset; 
            velocity_= direction * (raw_vel * 2 * M_PI);
            mode_ = mode;
        }
};
