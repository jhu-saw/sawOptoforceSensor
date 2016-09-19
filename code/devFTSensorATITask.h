/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#ifndef _devFTSensorATITask_h
#define _devFTSensorATITask_h

#include "devFTSensorATI.h"
#include <cisstMultiTask.h>
#include <cisstOSAbstraction/osaSleep.h>

//This class is created as a threaded wrapper for the devFTSensorATI class.
//if you don't need a seperate thread use devFTSensorATI class instead.

class devFTSensorATITask: public mtsTaskPeriodic {
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_LOD_INIT_ERROR);
    
 protected:
   
    //Biases the CurrentLoad on the ft sensor
    void BiasCurrentLoad(void);  
	//pass in a large string
    void GetSerialNumber(std::string & serial) const;  
    //vector of 6 max load values
    void GetMaxLoads(mtsDoubleVec &values) const;
    //vector of 6, in mm and deg 
    void SetTransform(const mtsDoubleVec &values);

	
 public:
    // as in previous example
    devFTSensorATITask(const std::string & taskName, double period);
    ~devFTSensorATITask() {};

    //these are the typical defaults for each OS
#ifdef _WIN32
    void Configure(const std::string & filename, const std::string & devName = "Dev1");
#else
    void Configure(const std::string & filename, const std::string & devName = "/dev/comedi0");
#endif

    void Startup(void);
    void Run(void);
    void Cleanup(void) {};

    //mtsStateData<mtsDoubleVec>  FTSensorState;
    vct6						FTSensor;
	vct6						FTSensorCurrent;
	vct6						FTSensorPrevious;
	double						alpha;
	vctFixedSizeVector<double, 3>  FTSensorTemp;
	vctFixedSizeVector<double, 3>  FTSensorTemp1;
    mtsBool                     IsSaturated;
    devFTSensorATI              ATIFTSensor;
	// Force sensor ration matrix to align the sensor to the robot
	// It is assumed that Z axis is over the sensor and Y axis is where
	// US probe is connected
	vctMatRot3					RotationMatrix;
	// Due to second rotation of the sensor(new design)
	vctMatRot3					R;

    //internal command to populate ft values in state table.
    void GetReadings(void); 

};

CMN_DECLARE_SERVICES_INSTANTIATION(devFTSensorATITask);

#endif // _devFTSensorATITask_h

