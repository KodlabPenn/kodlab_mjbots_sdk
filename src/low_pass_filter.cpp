/**
 * @file low_pass_filter.cpp
 * @author Lucien C. Peach (peach@seas.upenn.edu)
 * @brief Implementation of a discrete-time low-pass filter class.
 * @date 9/10/2022
 * 
 * @copyright 2022 The Trustees of the University of Pennsylvania. All rights reserved.
 * 
 */

#include "kodlab_mjbots_sdk/low_pass_filter.h"
#include <math.h>
#include <Eigen/Geometry>

namespace kodlab
{

LowPassFilter::LowPassFilter(float dt, float k_frequency_cutoff, Eigen::Vector3f raw_data) {

    dt_ = dt;
    raw_data_ = raw_data;
    filtered_data_ = raw_data;
    k_frequency_cutoff_ = k_frequency_cutoff; 

};

float LowPassFilter::FilterGain(float k_frequency_cutoff_) {

    float k_filter_time_constant = 1.0 / (2 * M_PI * k_frequency_cutoff_);

    float alpha_ = dt_ / (k_filter_time_constant + dt_);

}

Eigen::Vector3f LowPassFilter::Update(Eigen::Vector3f raw_data) {

    filtered_data_ = alpha_ * raw_data + (1 - alpha_) * filtered_data_;
    raw_data = raw_data;
    
}

// Future Version: Include Reinitialize / Set Filter Data: 

} // kodlab

