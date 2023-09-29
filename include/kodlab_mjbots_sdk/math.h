/**
 * @file math.h
 * @author Shane Rosen-Levy (srozen01@seas.upenn.edu)
 * @author Ethan Musser (emusser@seas.upenn.edu)
 * @brief Common math objects and functions
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

/**
 * @brief helper class for describing a range or interval
 * @details describes an interval using min() and max() methods.
 * @tparam EigenOrScalar Either Scalar-type (e.g. float, double, etc.) or 
 * EigenVector-type (e.g. Eigen::Vector3f, Eigen::Vector4d, etc.)
 * @note Eigen-types describe a set of intervals described by their elements
 * @note This class is a simple wrapper around a std::pair
*/
template <typename EigenOrScalar>
class Range{

 public:  
  /**
   * @brief Constructs Range and implicitly gets EigenOrScalar type
   * @param min lower end of the interval
   * @param max upper end of the interval
  */
  Range(const EigenOrScalar & min, const EigenOrScalar & max): pair_(min,max) {}

  /**
   * @brief Return the lower/min of the range/interval
  */
  const EigenOrScalar & min(){
    return pair_.first;
  }

  /**
   * @brief Return the upper/max of the range/interval
  */
  const  EigenOrScalar & max(){
    return pair_.second;
  }
  
  /**
   * @brief Set the lower/min of the range/interval
  */
  void set_min(const EigenOrScalar & min){
    pair_.first = min;
  }

  /**
   * @brief Set the upper/max of the range/interval
  */
  void set_max(const EigenOrScalar & max){
    pair_.second = max;
  }

 private:

  /**
   * @brief Internal std::pair that defines the range
  */
  std::pair<EigenOrScalar, EigenOrScalar> pair_;
};

} // namespace kodlab::math
