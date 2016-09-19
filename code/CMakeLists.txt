# Author(s):  H. Tutkun Sen
# Created on: 2014-04-09

Project(devFTSensorATILib)


    # Add a new library files 
    #This library requires the xml parser from dclib. (http://dclib.sourceforge.net/)
    #Can be run as a task or stand alone.
    # Should add ${IS_SHARED} below
add_library(devFTSensorATILib 
                 #ati FT sensors driver
                 devFTSensorATI.cpp
                 devFTSensorATITask.cpp
                 ATIDAQHardwareInterface.h
                 ATIDAQHardwareInterface.cpp
                 ftconfig.c
                 ftrt.c
                 #ati FT sensors driver h files
                 devFTSensorATI.h
                 devFTSensorATITask.h            
                 ftconfig.h
                 ftrt.h
    )
target_link_libraries(devFTSensorATILib  
                        ${devFTSensorATILib_LINK_LIBS}
                        )
cisst_target_link_libraries(devFTSensorATILib cisstCommon cisstCommonXML cisstVector cisstNumerical cisstOSAbstraction cisstMultiTask cisstParameterTypes)

	
	
	