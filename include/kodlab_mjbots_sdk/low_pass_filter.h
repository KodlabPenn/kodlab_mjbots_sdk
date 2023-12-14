/**
 * @file low_pass_filter.h
 * @author Lucien C. Peach (peach@seas.upenn.edu)
 * @brief A discrete-time low-pass filter class.
 * @date 9/10/2022
 * 
 * @copyright 2022 The Trustees of the University of Pennsylvania. All rights reserved.
 * 
 */

#pragma once

#include "kodlab_mjbots_sdk/filter.h"
#include "Eigen/Geometry"

namespace kodlab
{

class LowPassFilter : public Filter {

    public:

        LowPassFilter(float dt, float k_frequency_cutoff, Eigen::Vector3f raw_data);

        float FilterGain(const float k_frequency_cutoff);

        Eigen::Vector3f Update(Eigen::Vector3f raw_data) override;

        // Accessor for dt_
        float GetDt() {
            return dt_;
        }

        // Accessor for k_frequency_cutoff_
        float GetCutoffFreq() {
            return k_frequency_cutoff_;
        }

        // Accessor for alpha_
        float GetAlpha() {
            return alpha_;
        }

        
    private:

        float dt_;
        float k_frequency_cutoff_;
        float alpha_;

};

} //kodlab



/* Notes for further additions

//main.cpp

LowPassFilter<Eigen::Vector3f> lpf  (dt, cutoff );
while (True){
    //DO STUFF
    filt_data = lpf.Update(data);
    
    //DO STUFF
} */