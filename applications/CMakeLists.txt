#
# (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

# List cisst libraries needed
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstNumerical
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  find_package (sawOptoforceSensor REQUIRED)
  include_directories (${sawOptoforceSensor_INCLUDE_DIR})
  link_directories (${sawOptoforceSensor_LIBRARY_DIR})

  add_executable (OptoforceCalib calibration.cpp)

  # link with the cisst libraries
  cisst_target_link_libraries (OptoforceCalib ${REQUIRED_CISST_LIBRARIES})

  # link with sawOptoforceSensor library
  target_link_libraries (OptoforceCalib ${sawOptoforceSensor_LIBRARIES})

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: OptoforceCalib will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
