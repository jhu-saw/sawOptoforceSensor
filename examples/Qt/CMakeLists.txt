#
# (C) Copyright 2015-2019 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8.3)

# find cisst and make sure the required libraries have been compiled
set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonQt
     cisstVector
     cisstVectorQt
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstQt)

find_package (cisst 1.0.11 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawOptoforceSensorExampleQt_IS_CATKIN_BUILT)
  if (sawOptoforceSensorExampleQt_IS_CATKIN_BUILT)
    set (EXECUTABLE_OUTPUT_PATH "${CATKIN_DEVEL_PREFIX}/bin")
  endif ()

  # sawOptoforceSensor has been compiled within cisst, we should find it automatically
  find_package (sawOptoforceSensor 1.1.0)

  if (sawOptoforceSensor_FOUND)

    include_directories (${sawOptoforceSensor_INCLUDE_DIR})

    link_directories (${sawOptoforceSensor_LIBRARY_DIR})

    add_executable (sawOptoforceSensorExampleQt optoforce_json.cpp)
    target_link_libraries (sawOptoforceSensorExampleQt
                           ${sawOptoforceSensor_LIBRARIES})
    cisst_target_link_libraries (sawOptoforceSensorExampleQt ${REQUIRED_CISST_LIBRARIES})

  else (sawOptoforceSensor_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires sawOptoforceSensor")
  endif (sawOptoforceSensor_FOUND)

endif (cisst_FOUND_AS_REQUIRED)
