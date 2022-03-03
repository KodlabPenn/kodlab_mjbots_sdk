#include "jointBase.h"
class RobotBase
{
    protected:
        /* Containers for the actuators and limbs, 
        the main part of this class */
        std::vector<JointBase> joints_; /*THIS SHOULD NOT BE CHANGED AFTER CONSTRUCTION 
            The reference wrappers will fail if the vector reallocates itself
            TODO: Can likely be fixed with unique_pointers if we think it is important*/
        std::vector<LimbBase> limbs_;

    public: 
        /* Read-only current joint positions, velocities, and torques
        in the order given to the robot constructor  */
        std::vector<std::reference_wrapper<const float>> joint_positions;
        std::vector<std::reference_wrapper<const float>> joint_velocities;
        std::vector<std::reference_wrapper<const float>> joint_torques;

        robotBase(std::vector<Joint>);
        virtual void buildLimbs()=0;
};

robotBase::robotBase(std::vector<Joint> joint_vect): joints_(std::move(joint_vect)){
    // Build reference vectors
    for( Joint & j: joints_){
        joint_positions.push_back( j.getPositionReference() );// TODO check that "const"'s are accurate
        joint_positions.push_back( j.getVelocityReference() );
        joint_positions.push_back( j.getTorqueReference() ); 
    }
    buildLimbs();


}
