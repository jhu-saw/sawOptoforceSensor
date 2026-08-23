#
# (C) Copyright 2016-2026 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 3.10)
project (sawOptoforceSensorExampleConsole VERSION 1.2.0)

# List cisst libraries needed
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.2.0 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  cisst_set_output_path ()

  find_package (sawOptoforceSensor 1.2.0 REQUIRED)
  include_directories (${sawOptoforceSensor_INCLUDE_DIR})
  link_directories (${sawOptoforceSensor_LIBRARY_DIR})

  add_executable (sawOptoforceSensorExampleConsole main.cpp)

  # link with sawOptoforceSensor library
  target_link_libraries (
    sawOptoforceSensorExampleConsole
    ${sawOptoforceSensor_LIBRARIES}
    ${cisst_LIBRARIES}
    )

  install (TARGETS sawOptoforceSensorExampleConsole COMPONENT sawOptoforceSensor-Examples
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

else (cisst_FOUND)
  message ("Information: sawOptoforceSensorExampleConsole will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND)
