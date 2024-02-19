#
# (C) Copyright 2016-2023 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawOptoforceSensor VERSION 1.1.0)

# Core libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstNumerical
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst 1.2.0 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_set_output_path ()

  # create/configure file for find_package (sawOptoforceSensor)
  set (sawOptoforceSensor_INCLUDE_DIR
       "${sawOptoforceSensor_SOURCE_DIR}/include"
       "${sawOptoforceSensor_BINARY_DIR}/include")
  set (sawOptoforceSensor_HEADER_DIR "${sawOptoforceSensor_SOURCE_DIR}/include/sawOptoforceSensor")
  set (sawOptoforceSensor_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawOptoforceSensor_LIBRARIES sawOptoforceSensor)

  include_directories (${sawOptoforceSensor_INCLUDE_DIR})

  if (CISST_HAS_QT)
    add_subdirectory (code/Qt)
    set (sawOptoforceSensor_LIBRARIES ${sawOptoforceSensor_LIBRARIES} sawOptoforceSensorQt)
  endif ()

  # add all config files for this component
  cisst_add_config_files (sawOptoforceSensor)

  add_library (sawOptoforceSensor
               code/mtsOptoforce3D.cpp
               ${sawOptoforceSensor_HEADER_DIR}/sawOptoforceSensorExport.h
               ${sawOptoforceSensor_HEADER_DIR}/mtsOptoforce3D.h
               ${sawOptoforceSensor_BINARY_DIR}/include/sawOptoforceSensor/sawOptoforceSensorRevision.h)

   set_target_properties (sawOptoforceSensor PROPERTIES
                          VERSION ${sawOptoforceSensor_VERSION}
                          FOLDER "sawOptoforceSensor")

  cisst_target_link_libraries (sawOptoforceSensor ${REQUIRED_CISST_LIBRARIES})

  # Install target for headers and library
  install (DIRECTORY "${sawOptoforceSensor_SOURCE_DIR}/include/sawOptoforceSensor"
                     "${sawOptoforceSensor_BINARY_DIR}/include/sawOptoforceSensor"
           DESTINATION include
           COMPONENT sawOptoforceSensor-dev
           PATTERN .svn EXCLUDE)

  install (TARGETS sawOptoforceSensor COMPONENT sawOptoforceSensor
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
