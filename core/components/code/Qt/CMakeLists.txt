#
# (C) Copyright 2013-2026 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstVector
     cisstVectorQt
     cisstNumerical
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstParameterTypesQt)

# cisst
find_package (cisst COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # for Qt generated header files
  include_directories (${CMAKE_CURRENT_BINARY_DIR})
  set (CMAKE_AUTOMOC ON)

  add_library (sawOptoforceSensorQt
               ${sawOptoforceSensor_HEADER_DIR}/sawOptoforceSensorQtExport.h
               ${sawOptoforceSensor_HEADER_DIR}/mtsOptoforce3DQtWidget.h
               mtsOptoforce3DQtWidget.cpp)
  set_target_properties (sawOptoforceSensorQt PROPERTIES
                         VERSION ${sawOptoforceSensor_VERSION}
                         FOLDER "sawOptoforceSensor")
  target_link_libraries (sawOptoforceSensorQt ${cisst_LIBRARIES})

  install (TARGETS sawOptoforceSensorQt COMPONENT sawOptoforceSensor-Qt
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

else (cisst_FOUND)
  message ("Information: code will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND)
