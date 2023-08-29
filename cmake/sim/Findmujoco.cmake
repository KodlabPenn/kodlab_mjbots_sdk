#
# Findmujoco.cmake
#
# This module finds the mujoco package (library and header files) and adds the
# package as a target, if it is not already added.
#
# The mujoco_INSTALL_PATH variable must be set prior to calling find_package.
#
# If mujoco is found:
#   - mujoco_FOUND will be set to TRUE
#   - mujoco_VERSION will be set to the found version
#   - mujoco::mujoco will be added as a target (if not already added).
# Otherwise, mujoco_FOUND will be set to FALSE.
#
# This module will work with the REQUIRED, QUIET, and version-related arguments
# of find_package.
#
# Copyright (c) 2022 The Trustees of the University of Pennsylvania.
# All rights reserved.
#
# See `LICENSE` for license information.
#

include(FindPackageHandleStandardArgs)

# Find library and include directory
find_library(mujoco_LIBRARY
  NAMES mujoco mujoco.${MUJOCO_VERSION}
  HINTS ${mujoco_INSTALL_PATH}/lib ${mujoco_INSTALL_PATH}/bin
  NO_DEFAULT_PATH)
find_path(mujoco_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  HINTS ${mujoco_INSTALL_PATH}/include
  NO_DEFAULT_PATH)

# Identify header version
if (mujoco_INCLUDE_DIR)
  file(STRINGS "${mujoco_INCLUDE_DIR}/mujoco/mujoco.h" version-line
    REGEX "#define[ \t]+mjVERSION_HEADER")
  if (NOT version-line)
    message(AUTHOR_WARNING "mujoco_INCLUDE_DIR found, but mujoco/mujoco.h is missing")
  endif ()
  string(REGEX REPLACE
    "^#define[ \t]+mjVERSION_HEADER[ \t]+([0-9])([0-9])([0-9])$"
    "\\1.\\2.\\3" mujoco_VERSION ${version-line})
endif ()

# Find package, if installed
find_package_handle_standard_args(mujoco
  REQUIRED_VARS mujoco_LIBRARY mujoco_INCLUDE_DIR
  VERSION_VAR mujoco_VERSION
  HANDLE_COMPONENTS
  REASON_FAILURE_MESSAGE "Ensure mujoco_INSTALL_PATH is correct, currently set to ${mujoco_INSTALL_PATH}.")

# Advance variables if package found
if (mujoco_FOUND)
  mark_as_advanced(mujoco_INCLUDE_DIR mujoco_LIBRARY)
endif ()

# Add target if not already added
if (mujoco_FOUND AND NOT TARGET mujoco::mujoco)
  add_library(mujoco::mujoco SHARED IMPORTED)
  set_property(TARGET mujoco::mujoco
    PROPERTY IMPORTED_LOCATION ${mujoco_LIBRARY})
  target_include_directories(mujoco::mujoco
    INTERFACE ${mujoco_INCLUDE_DIR})
endif ()
