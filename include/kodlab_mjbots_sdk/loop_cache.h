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
#include <atomic>

#include "kodlab_mjbots_sdk/common_header.h"

namespace kodlab{

/**
 * @brief Simple loop id counter class that allows the user to tell when the 
 * control loop has looped. Useful for forcing recalculations and/or timing.
 * @warning This is a singleton, with all the pros and cons that come with that
 * @warning This is not thread safe and should only be used in the control loop
 * 
 */
class LoopId{
    
 public:

  /**
   * @brief Get the LoopId object
   * 
   * @return LoopId& 
   */
  static LoopId& getInstance() {
    static LoopId loop_id_obj;
    return loop_id_obj;
  }

  /**
   * @brief Returns the current loop id
   * 
   * @return uint32_t 
   */
  static uint32_t get(){
    return getInstance().loop_id_;
  }

  /**
   * @brief Override the current loop id
   * 
   * @param id_des 
   */
  static void set(uint32_t id_des){
    getInstance().loop_id_ = id_des;
  }

  /**
   * @brief Initialize the loop id (sets it to uint32_t(-1))
   */
  static void init(){
    getInstance().set(-1); // Set loop to pre-loop value
  }

  /**
   * @brief Increment and return the loop id 
   * @note LoopId is a singleton, there can only be one, only one control loop
   * at a time should be incrementing it otherwise you may be prematurely 
   * invalidating caches.
   * 
   * @return uint32_t 
   */
  static uint32_t increment(){
      return (++getInstance().loop_id_);
  }

  // Delete copy and assignment, singletons should not do either
  LoopId(const LoopId&) = delete; ///< deleted
  LoopId& operator=(const LoopId&) = delete; ///< deleted

 private:
  /**
   * @brief Construct a new Loop Id object.
   */
  LoopId() {}
  /**
   * @brief Destroy the Loop Id object
   * 
   */
  ~LoopId() {}

  /**
   * @brief Loop id shared between all instances acts a "global" loop id
   * 
   */
  std::atomic<std::uint32_t> loop_id_;
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
  using ValidatedCache<T>::operator=;

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
  uint32_t last_loop_id_ = -1; // loop id from the last set event, used to invalidate
  LoopId loop_id_; // Loop id, tied to control loop
};

}// namespace kodlab
