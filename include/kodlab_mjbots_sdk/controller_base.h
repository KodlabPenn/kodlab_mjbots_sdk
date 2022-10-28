/**
 * @file controller_base.h
 * @author Kodlab - Chandravaran Kunjeti (kunjeti@seas.upenn.edu)
 * @brief Abstract class for controllers.
 * @version 0.1
 * @date 10/13/2022
 *
 * @copyright 2022 The Trustees of the University of Pennsylvania.
 * All rights reserved.
 *
 */

# pragma once

#include <string.h>
#include <unordered_map>

namespace kodlab {

  class ControllerAbstract {
  public:

    ControllerAbstract() {
    }

    virutal void Update() = 0;

    /**
     * @brief Function to set the time step of your controller
     * @param gains Gains to be set
     */
    void set_time_step(double time_step) {
      time_step_ = time_step;
    }

    /**
     * @brief Function to set the gains of you controller.
     * @param gains Gains to be set
     */
    virtual void set_gains(std::unordered_map<std::string, double> gains) {
      gains_ = gains;
    }

    /**
     * @brief Destructor
     */
    ~ControllerAbstract() {}
  protected:

    /**
     * @brief Time difference between 2 updates
     */
    static double time_step_;

    /**
     * @brief Gains that are going to be used by the controller
     */
    std::unordered_map<std::string, double> gains_;
  };

}
