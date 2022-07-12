#pragma once 
#include "kodlab_mjbots_sdk/robot_super.h"

namespace kodlab
{

  class Behavior
  {
  public:
    Behavior() {}
    void Init(kodlab::RobotSuperBase *robot) ;
    void Update(kodlab::RobotSuperBase *robot) ;
    void End() ;
  };

  class OffB : Behavior
  {
  public:
    OffB() {}
    void Init(kodlab::RobotSuperBase *robot) ;
    void Update(kodlab::RobotSuperBase *robot) 
    {
      for (std::shared_ptr<mjbots::JointMoteus> j : robot->joints)
      {
        j->UpdateTorque(0.0);
      }
    }
    void End() {}
  };


}
