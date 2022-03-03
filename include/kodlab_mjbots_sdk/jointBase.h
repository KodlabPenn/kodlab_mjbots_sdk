
//
// Created by kodlab on 02/17/22.
// J. Diego Caporale
//
/**
 * @file jointBase.h
 * @author Kodlab - J. Diego Caporale (jdcap@seas.upenn.edu)
 * @brief Abstract class for joints. Agnostic to servo/motor used.
 * @version 0.1
 * @date 2022-02-17
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <limits>



// struct joint_options{
    
// }


/**
 * @brief 
 * Abstract Base class for any joint used in the robot class.
 */
class JointBase {
    protected:
        float gear_ratio_ = 1.0;
        float zero_offset_ = 0;
        int  direction_ = 1; 
        float max_torque_ = std::numeric_limits<float>::infinity();
        float pos_limit_min_ = -std::numeric_limits<float>::infinity();
        float pos_limit_max_ = -std::numeric_limits<float>::infinity();
        bool  soft_stop_active = true;
        float position_; //Position returned by the servo
        float velocity_; //Velocity returned by the servo
        float torque_cmd_ = 0; //torque returned by the servo

    public:
        JointBase(int direction = 1, 
                float zero_offset = 0,
                float max_torque = std::numeric_limits<float>::infinity(),
                float gear_ratio = 1.0, 
                float pos_min = -std::numeric_limits<float>::infinity(), 
                float pos_max = std::numeric_limits<float>::infinity())
                :direction_(direction), zero_offset_(zero_offset), max_torque_(max_torque), gear_ratio_(gear_ratio){
                    pos_limit_min_ = pos_min;
                    pos_limit_max_ = pos_max;
                    direction_ = (direction_ >= 0) - (direction_ < 0); //set to sign of direction (-1 or 1)  (0 is set to 1)
                }
        
        //TODO CHANGE THIS VIRTUAL OR REMOVE AND REQUIRE THE CHILDREN TO INTERACT WITH POS AND VEL 
        // virtual void updateState(float pos, float vel){position_ = pos; velocity_= vel};

        // Get the values 
        virtual float getPosition()  {return position_;} //required
        virtual float getVelocity()  {return velocity_;} //required
        virtual float getTorqueCmd() {return torque_cmd_;}//required

        //REMOVE virtual float setPosition(){} // Can be used with joints that have built in PID
        //REMOVE virtual float setVelocity(){} // Can be used with joints that have built in PID
        virtual float setTorque(float torq){
            //TODO include velocity? probably a bad idea.
            if (!soft_stop_active) {
                torque_cmd_= torq;} // Can be used with joints that accept torque commands
            else {
                if (position_ > pos_limit_max_  ){
                    torque_cmd_= (torq > 0) ? 0 : torq
                }
                else if (position_ < pos_limit_min_  ){
                    torque_cmd_= (torq < 0) ? 0 : torq  
                }
            }
        }//TODO OPTIMIZE
        void setSoftStop(bool active){soft_stop_active = active;}
        const float& getPositionReference() const {return position_;}
        const float& getVelocityReference() const {return velocity_;}
        const float& getTorqueReference()   const {return torque_cmd_;}

        
};