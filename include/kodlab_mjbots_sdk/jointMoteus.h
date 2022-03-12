#ifndef JOINTMOTEUS_H
#define JOINTMOTEUS_H
#include "kodlab_mjbots_sdk/jointBase.h"
#include "kodlab_mjbots_sdk/moteus_protocol.h"
#include "kodlab_mjbots_sdk/pi3hat_moteus_interface.h"

/**
 * @brief 
 * Moteus Joint class that includes all the motor specific vectors. 
 */
namespace kodlab{
class JointMoteus: public JointBase{
    /**
     * @brief 
     * 
     */
    public:
        int can_bus;
        int can_id;

        ::mjbots::moteus::Mode mode_ = ::mjbots::moteus::Mode::kStopped;

        
        JointMoteus(int can_bus_,int can_id_,
            int direction = 1, 
            float zero_offset = 0,
            float max_torque = std::numeric_limits<float>::infinity(),
            float gear_ratio = 1.0, 
            float pos_min = -std::numeric_limits<float>::infinity(), 
            float pos_max = std::numeric_limits<float>::infinity()
            )
            :JointBase(direction,zero_offset,max_torque,gear_ratio,pos_min,pos_max),
            can_bus(can_bus_), can_id(can_id_){

        }
        
        void updateMoteus(float reply_pos, float reply_vel, ::mjbots::moteus::Mode mode ){
            updateState( 2 * M_PI * reply_pos, 2 * M_PI * reply_vel );
            mode_ = mode;
        }
        const ::mjbots::moteus::Mode & getModeReference()   const {return mode_; }
};
}//namespace kodlab
#endif //JOINTMOTEUS_H