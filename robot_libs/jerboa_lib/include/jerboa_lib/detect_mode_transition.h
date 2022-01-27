//
// Created by shane on 1/20/22.
//

#pragma once

#include <cstdint>
enum HybridMode{
  UNINITIALIZED = 0,
  FLIGHT = 1,
  STANCE = 2,
  DISABLE = 3
};

class ModeObserver{
 public:
  void DetectModeTransition(float leg_comp, float leg_speed, float t);

  HybridMode GetMode();

  void Disable();

 private:
  HybridMode hybrid_mode_ = UNINITIALIZED;
  float last_mode_transition_ = 0;

  const float td_offset_ = 0.004;///< Epsilon for how close to 0 for transition stance to flight in meters
  const float lo_offset_ = 0.0005;///< Epsilon for how close to 0 for transition stance to flight in meters
  const uint16_t mode_change_wait_ = 40; ///< How long to wait before looking for next transition in milliseconds

};