/**
 * @file common_header.cpp
 * @author J. Diego Caporale (jdcap@seas.upenn.edu)
 * @brief Source for common header, in particular controlling the 
 * CTRL-C flag implementation
 * @date 2023-06-07
 * 
 * @copyright BSD 3-Clause License, 
 * Copyright (c) 2023 The Trustees of the University of Pennsylvania. 
 * All Rights Reserved
 */
#include "kodlab_mjbots_sdk/common_header.h"

namespace kodlab{

/*
 * @brief This boolean flag is used to cleanly kill the application upon ctrl+c
 */
static std::atomic_bool CTRL_C_DETECTED(false);

void ActivateCtrlC(){ CTRL_C_DETECTED=true; }

bool CtrlCDetected(){ return CTRL_C_DETECTED; }

} // namespace kodlab
