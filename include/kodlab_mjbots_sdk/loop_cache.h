/**
 * @file loop_cache.h
 * @author J. Diego Caporale (jdcap@seas.upenn.edu)
 * @brief Control loop validated cache and id for automatic invalidation synced 
 * with loop
 * @date 2023-03-30
 * 
 * @copyright Copyright (c) 2023 The Trustees of the University of Pennsylvania. 
 * All Rights Reserved
 * BSD 3-Clause License
 */

#pragma once
#include <type_traits>
#include <memory>

#include "kodlab_mjbots_sdk/common_header.h"

namespace kodlab{

/**
 * @brief Simple loop id counter class that allows the user to tell when the 
 * control loop has looped. Useful for forcing recalculations and/or timing.
 * 
 */
class LoopId
{
 public:
  /**
   * @brief Returns the current loop id
   * 
   * @return uint32_t 
   */
  static uint32_t get(){
    return loop_id_;
  }

  /**
   * @brief Override the current loop id
   * 
   * @param id_des 
   */
  static void set(uint32_t id_des){
    loop_id_ = id_des;
  }

  /**
   * @brief increment and return the loop id, note 
   * @note there can only be ONE loop using this type of cache at once to avoid 
   * prematurely invalidating caches, MAKE SURE THIS IS WHAT YOU WANT
   * 
   * @return uint8_t 
   */
  static uint32_t increment(){
      return (++loop_id_);
  }

 protected:
  /**
   * @brief Loop id shared between all instances acts a "global" loop id
   * 
   */
  static uint32_t loop_id_; 
};

/**
 * @brief Subclass of the ValidatedCache which uses the loop id to invalidate 
 * data, while still allowing the user to invalidate manually if so desired.
 * 
 * @tparam T, data type
 */
template<typename T>
class ValidatedLoopCache : public ValidatedCache<T>
{
 public:
  using ValidatedCache<T>::ValidatedCache;
  using ValidatedCache<T>::valid_;
  using ValidatedCache<T>::data_;

  /**
   * @brief Returns the data status
   * @return \c true if data is valid, \c false otherwise
   */
  bool valid() const override{ return (valid_ && loop_id_.get() == last_loop_id_); }
  
  /**
   * @brief Sets the class data and marks it valid and saves loop id
   * @param data valid data
   */
  void set(const T &data) override
  {
    data_ = data;
    valid_ = true;
    last_loop_id_ = loop_id_.get();
  }

 protected:
  uint8_t last_loop_id_; // loop id from the last set event, used to invalidate
  LoopId loop_id_; // Loop id, tied to control loop
};

}// namespace kodlab
