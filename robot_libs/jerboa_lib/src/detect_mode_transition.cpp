//
// Created by shane on 1/20/22.
//

#include <jerboa_lib/detect_mode_transition.h>

void ModeObserver::DetectModeTransition(float leg_comp, float leg_speed, float t) {
  bool leg_stance = hybrid_mode_ == HybridMode::STANCE;
  bool leg_lo;
  switch (hybrid_mode_){
    //if you are UNINITIALIZED, check position to determine flight or stance
    case HybridMode::UNINITIALIZED:
      leg_stance = leg_comp > td_offset_;
      if (leg_stance){
        hybrid_mode_=HybridMode::STANCE;
        last_mode_transition_ = t;
      }else{
        hybrid_mode_=HybridMode::FLIGHT;
        last_mode_transition_ = t;
      }
      break;

      // If you are in flight, check position and velocity of both legs to determine if in stance
    case HybridMode::FLIGHT:
      leg_stance = leg_comp >= td_offset_ && leg_speed < 0;
      if (leg_stance &&
          (t-last_mode_transition_) >= mode_change_wait_){
        hybrid_mode_=HybridMode::STANCE;
        last_mode_transition_ = t;
      }
      break;

      // If you are in stance, check position and velocity to determine if in flight
    case HybridMode::STANCE:
      leg_lo = leg_comp <= lo_offset_;
      if (leg_lo  &&
          (t-last_mode_transition_)>= mode_change_wait_){
        hybrid_mode_=HybridMode::FLIGHT;
        last_mode_transition_ = t;
      }
      break;
    default:
      break;
  }//Switch statement

}
HybridMode ModeObserver::GetMode() {
  return hybrid_mode_;
}
void ModeObserver::Disable() {
  hybrid_mode_ = DISABLE;
}

