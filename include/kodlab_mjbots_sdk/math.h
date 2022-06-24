/**
 * @file math.h
 * @author Shane Rozen-Levy (srozen01@seas.upenn.edu)
 * @author J. Diego Caporale (jdcap@seas.upenn.edu)
 * @author Ethan Musser (emusser@seas.upenn.edu)
 * @brief
 * @version 0.1
 * @date 2022-06-22
 *
 * @copyright Copyright (c) 2022
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
  template <typename Scalar>
  inline int sgn(Scalar val)
  {
    static_assert(std::is_arithmetic<Scalar>::value, "sgn requires a numeric type.");
    return (Scalar(0) < val) - (val < Scalar(0));
  }


}

