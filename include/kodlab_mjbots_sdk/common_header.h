// Taken from solo8 repo for handeling ctrlc

#ifndef KODLAB_MJBOTS_SDK_INCLUDE_KODLAB_MJBOTS_SDK_COMMON_HEADER_H_
#define KODLAB_MJBOTS_SDK_INCLUDE_KODLAB_MJBOTS_SDK_COMMON_HEADER_H_
#include <atomic>
#include <bits/sigaction.h>
#include <signal.h>  // manage the ctrl+c signal

/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
std::atomic_bool CTRL_C_DETECTED(false);

/**
 * @brief This function is the callback upon a ctrl+c call from the terminal.
 *
 * @param s is the id of the signal
 */
void my_handler(int)
{
  CTRL_C_DETECTED = true;
}

/**
 * @brief Enable to kill the demos cleanly with a ctrl+c
 */
void enable_ctrl_c()
{
  // make sure we catch the ctrl+c signal to kill the application properly.
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  CTRL_C_DETECTED = false;
}

#endif //KODLAB_MJBOTS_SDK_INCLUDE_KODLAB_MJBOTS_SDK_COMMON_HEADER_H_
