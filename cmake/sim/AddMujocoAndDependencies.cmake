#
# AddMujocoAndDependencies.cmake
#
# This script adds mujoco and its dependencies.
#
# The following variables can be set in the calling scope or via environment
# variables, if desired:
#   mujoco_INSTALL_VERSION - the installed mujoco version (default 2.3.0)
#   mujoco_INSTALL_PATH - the mujoco install path
#                         (default ~/.mujoco/mujoco-${mujoco_INSTALL_VERSION}).
#
# The following dependencies are found:
#   mujoco (>= 2.3.0)
#   glfw
# These are the only mujoco requirements as of mujoco 2.3.0. Note that mujoco is
# also added as the namespaced target mujoco::mujoco.
#
# Copyright (c) 2023 The Trustees of the University of Pennsylvania.
# All rights reserved.
#
# See `LICENSE` for license information.
#

# Set mujoco minimum required version
set(mujoco_MIN_VERSION 2.3.0)
set( CMAKE_MESSAGE_LOG_LEVEL "DEBUG" )
# Set mujoco install version
if (DEFINED ENV{mujoco_INSTALL_VERSION})
  set(mujoco_INSTALL_VERSION $ENV{mujoco_INSTALL_VERSION})
elseif (NOT mujoco_INSTALL_VERSION)
  set(mujoco_INSTALL_VERSION 2.3.0)
endif ()
message(DEBUG "mujoco_INSTALL_VERSION is ${mujoco_INSTALL_VERSION}")

# Set mujoco installation path (required for find_package)
if (DEFINED ENV{mujoco_INSTALL_PATH})
  set(mujoco_INSTALL_PATH $ENV{mujoco_INSTALL_PATH})
elseif (NOT mujoco_INSTALL_PATH)
  set(mujoco_INSTALL_PATH $ENV{HOME}/.mujoco/mujoco-${mujoco_INSTALL_VERSION})
endif ()
message(DEBUG "mujoco_INSTALL_PATH is ${mujoco_INSTALL_PATH}")

# Add MuJoCo library
find_package(mujoco ${mujoco_MIN_VERSION} REQUIRED)

# Find glfw
find_package(glfw3 REQUIRED)
if (glfw3_FOUND)
  message(STATUS "Found glfw3 version \"${glfw3_VERSION}\"")
endif ()
