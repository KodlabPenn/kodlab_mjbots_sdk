/**
 * @file string.h
 * @author Ethan J. Musser (emusser@seas.upenn.edu)
 * @brief Assorted methods relating to `std::string` objects
 * @date 8/21/22
 *
 * @copyright (c) Copyright 2022 The Trustees of the University of Pennsylvania.
 * All rights reserved.
 *
 */

#pragma once

#include <array>  // std::array
#include <iomanip>  // std::fixed, std::setw
#include <string>  // std::string
#include <sstream>  // std::stringstream
#include <vector>  // std::vector

#include <Eigen/Core>  // Eigen::MatrixX, Eigen::IOFormat

namespace kodlab::string {

/**
 * @brief Format a vector of scalars as a string.
 * @details A convenience function to format a `std::vector` of scalars as a
 * `std::string`.
 * @tparam Scalar numeric type
 * @param vector `std::vector` of scalars
 * @param precision nuber of digits after the decimal place for floating point
 * values
 * @param width allotted number of characters per floating point value
 * @return `std::string` containing a comma-separated list of the values in
 * `vector`
 */
template<typename Scalar>
static std::string ScalarVectorToString(const std::vector<Scalar> &vector,
                                        int precision = 2,
                                        int width = 8) {
  std::stringstream ss;
  ss.precision(precision);
  ss << std::fixed << "[ ";
  for (auto it = vector.begin(); it != vector.end(); it++) {
    if (it != vector.begin()) { ss << ", "; }
    ss << std::setw(width) << *it;
  }
  ss << " ]";
  return ss.str();
}

/**
 * @brief Format an array of scalars as a string.
 * @details A convenience function to format a `std::array` of scalars as a
 * `std::string`.
 * @tparam Scalar numeric type
 * @tparam N number of array elements
 * @param array `std::array` of scalarsVector
 * @param precision nuber of digits after the decimal place for floating point
 * values
 * @param width allotted number of characters per floating point value
 * @return `std::string` containing a comma-separated list of the values in
 * `array`
 */
template<typename Scalar, int N>
static std::string ScalarArrayToString(const std::array<Scalar, N> &array,
                                       int precision = 2,
                                       int width = 8) {
  std::stringstream ss;
  ss.precision(precision);
  ss << std::fixed << "[ ";
  for (auto it = array.begin(); it != array.end(); it++) {
    if (it != array.begin()) { ss << ", "; }
    ss << std::setw(width) << *it;
  }
  ss << " ]";
  return ss.str();
}

/**
 * @brief Format an `Eigen::Matrix` of scalars as a string.
 * @details A convenience function to format an `Eigen::Matrix` of scalars as a
 * `std::string`.
 * @tparam Scalar numeric type
 * @param matrix `std::vector` of
 * @param precision nuber of digits after the decimal place for floating point
 * values
 * @param width maximum number of characters per value
 * @return `std::string` containing a comma-separated list of the values in
 * `vector`
 */
template<typename Scalar>
std::string EigenMatrixToString(const Eigen::MatrixX<Scalar> &matrix,
                                int precision = 2) {
  Eigen::IOFormat fmt(precision, 0, ", ", "\n", "[ ", " ]");
  std::stringstream ss;
  ss << matrix.format(fmt);
  return ss.str();
}

} // kodlab::string
