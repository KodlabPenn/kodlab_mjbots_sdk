// Taken from solo8 repo for handling ctrl+c

#pragma once
#include <atomic>
#include <signal.h>  // manage the ctrl+c signal
#include <type_traits>
#include <iostream>
#include <memory>
#include <vector>

namespace kodlab {
/*
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
static std::atomic_bool CTRL_C_DETECTED(false);

/**
 * @brief This function is the callback upon a ctrl+c call from the terminal.
 *
 * @param s is the id of the signal
 */
static void my_handler(int) {
  CTRL_C_DETECTED = true;
}

/**
 * @brief Enable to kill the demos cleanly with a ctrl+c
 */
static void EnableCtrlC() {
  // make sure we catch the ctrl+c signal to kill the application properly.
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  CTRL_C_DETECTED = false;
}

/**
 * @brief Simple object wrapper that caches the latest value until invalidated
 * @tparam T typename
 */
template<typename T>
class ValidatedCache
{

private:
  /**
   * @brief Validation status of \c data_
   */
  bool valid_;

  /**
   * @brief Data being stored
   */
  T data_;

public:
  /**
   * @brief Invalid data constructor
   */
  ValidatedCache() : valid_(false) {}

  /**
   * @brief Valid data constructor
   */
  explicit ValidatedCache(const T &data) : data_(data), valid_(true) {}

  /**
   * @brief Specified-validity data constructor. Useful for initializing, but leaving invalid to avoid a "get" of undefined memory.
   */
  ValidatedCache(const T &data, const bool &valid) : data_(data), valid_(valid) {}

  /**
   * @brief Returns the data status
   * @return \c true if data is valid, \c false otherwise
   */
  bool valid() const { return valid_; }

  /**
   * @brief Marks the class invalid so that future calls will change the cached
   * value
   */
  void invalidate() { valid_ = false; }

  /**
   * @brief Sets the class data and marks it valid
   * @param data valid data
   */
  void set(const T &data)
  {
    data_ = data;
    valid_ = true;
  }

  /**
   * @brief Uses operator= to set the class data and marks it valid
   * @param data valid data
   */
  void operator=(const T &data)
  {
    set(data);
  }

  /**
   * @brief Accessor for data (<b>does not check validity</b>)
   * @return data
   */
  T get()
  {
    if (!valid_)
    {
      std::cerr << "[WARN] Returning invalid data." << std::endl;
    }
    return data_;
  }

  /**
   * @brief Implicitly converts to the class template type \c T
   * @return data
   */
  operator T() { return get(); }

  /**
   * @brief Implicitly converts to the class template type \c T
   * @return data
   */
  operator T() const { return get(); }

};

/**
 * @brief Convert vector to shared pointers using copy constructor
 * @param objs vector of type T
 * @tparam T object type
 */

template< typename T>
std::vector<std::shared_ptr <T> > make_share_vector(std::vector<T> objs){
  std::vector<std::shared_ptr<T>> ptrs;
  for (T obj: objs){
    // Make vector of shared_pointer objects using copy constructer 
    ptrs.push_back(std::make_shared<T>(obj)); 
  }
  return ptrs;
}
}  // namespace kodlab