//
// Created by shane on 1/20/22.
//

#pragma once

enum HybridMode{
  UNINITIALIZED = 0,
  FLIGHT = 1,
  STANCE = 2,
  DISABLE = 3
};

class ModeObserver{
 public:
  void DetectModeTransition(float leg_comp, float leg_speed);

  HybridMode GetMode();
 private:
  HybridMode mode_ = UNINITIALIZED;
  float
};