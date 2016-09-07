# Author(s):  H. Tutkun Sen
# Created on: 2014-04-09

cmake_minimum_required (VERSION 2.8)

project(ForceTest)

add_library (mtsOptoforce3D.h
			 mtsOptoforce3D.cpp)

add_executable (ForceTest main.cpp)

#target_link_libraries(ForceTest  ${devFTSensorATILib_LINK_LIBS} devFTSensorATILib)
target_link_libraries(ForceTest  devFTSensorATILib sawOptoforceSensor)

	
cisst_target_link_libraries(ForceTest cisstCommon cisstCommonXML cisstVector cisstNumerical cisstOSAbstraction cisstMultiTask cisstParameterTypes)

	