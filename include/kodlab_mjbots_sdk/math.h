/**
 * @file math.h
 * @author Shane Rosen-Levy (srozen01@seas.upenn.edu)
 * @author Ethan Musser (emusser@seas.upenn.edu)
 * @brief Common math objects and funcitons
 * @date 7/8/22
 * 
 * @copyright 2022 The Trustees of the University of Pennsylvania. All rights reserved.
 * 
 */

#pragma once

#include <type_traits>
#include <iostream>

namespace kodlab::math
{

/**
 * @brief Signum function
 *
 * @tparam Scalar numeric type
 * @param val numeric value
 * @return -1 if val < 0, 0 if val = 0, 1 if val > 0
 */
template<typename Scalar>
inline int sgn(Scalar val)
{
  static_assert(std::is_arithmetic<Scalar>::value, "sgn requires a numeric type.");
  return (Scalar(0) < val) - (val < Scalar(0));
}

}

