// Taken from solo8 repo for handling ctrl+c

#pragma once
#include <atomic>
#include <signal.h>  // manage the ctrl+c signal

namespace kodlab {
/**
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
}  // namespace kodlab