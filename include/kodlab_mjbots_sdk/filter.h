/**
 * @file filter.h
 * @author Lucien C. Peach (peach@seas.upenn.edu)
 * @brief Abstract base class for support of filter implementation. 
 * @date 9/10/2022
 * 
 * @copyright 2022 The Trustees of the University of Pennsylvania. All rights reserved.
 * 
 */

#pragma once

#include "Eigen/Geometry"

namespace kodlab
{

class Filter {

    public: 

        // Pure virtual function for filter update calls
        virtual Eigen::Vector3f Update(Eigen::Vector3f raw_data) = 0;

        // Accessor for raw_data_
        Eigen::Vector3f GetRawData() {
            return raw_data_;
        }

        // Accessor for filtered_data_
        Eigen::Vector3f GetFilteredData() {
            return filtered_data_;
        }

   protected:

        Eigen::Vector3f raw_data_;
        Eigen::Vector3f filtered_data_;

};

} // kodlab::mjbots

