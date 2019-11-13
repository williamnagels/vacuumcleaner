# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_vacuumcleaner_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED vacuumcleaner_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(vacuumcleaner_FOUND FALSE)
  elseif(NOT vacuumcleaner_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(vacuumcleaner_FOUND FALSE)
  endif()
  return()
endif()
set(_vacuumcleaner_CONFIG_INCLUDED TRUE)

# output package information
if(NOT vacuumcleaner_FIND_QUIETLY)
  message(STATUS "Found vacuumcleaner: 0.0.0 (${vacuumcleaner_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'vacuumcleaner' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message(WARNING "${_msg}")
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(vacuumcleaner_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${vacuumcleaner_DIR}/${_extra}")
endforeach()
