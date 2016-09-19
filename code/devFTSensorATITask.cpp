/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <cisstCommon/cmnConstants.h>
#include <cisstVector/vctTypes.h>

#include <cisstCommon/cmnKbHit.h>
#include <cisstCommon/cmnGetChar.h>
#include "devFTSensorATITask.h"

CMN_IMPLEMENT_SERVICES(devFTSensorATITask);

devFTSensorATITask::devFTSensorATITask(const std::string & taskName, double period):
    mtsTaskPeriodic(taskName, period, false, 5000)
{
    mtsDoubleVec payLoadProtoTypeV6(6);

    // provided interfaces
    // -------------------------------------------------------------------
    mtsInterfaceProvided *providedInterface;
	providedInterface = AddInterfaceProvided("ProvidesATIFTSensor");

    IsSaturated.Data = false;
    StateTable.AddData(FTSensor,    "FTSensor");
    StateTable.AddData(IsSaturated, "IsSaturdated");
    providedInterface->AddCommandReadState(StateTable, FTSensor,        "GetState");
    providedInterface->AddCommandReadState(StateTable, IsSaturated,     "GetIsSaturated");
    providedInterface->AddCommandVoid(&devFTSensorATITask::BiasCurrentLoad, this, "BiasCurrentLoad");
    providedInterface->AddCommandRead(&devFTSensorATITask::GetMaxLoads,     this, "GetMaxLoads", mtsDoubleVec(6));
    providedInterface->AddCommandWrite(&devFTSensorATITask::SetTransform,   this, "SetTransform", mtsDoubleVec(6));

	FTSensorPrevious.SetAll(0.0);
	alpha = 0.5;
}

void devFTSensorATITask::Startup(void) {
    StateTable.Advance();
}

void devFTSensorATITask::Run(void) {
    ProcessQueuedCommands();
    GetReadings();
}
void devFTSensorATITask::GetReadings(void){

    //CMN_LOG_CLASS_RUN_VERBOSE << "GetFTSensorReadings " << this->GetName() << std::endl;
    //! \warning if the readings are saturated the conversion might produce odd results.

    int result = ATIFTSensor.ReadSingleFTRecord(FTSensor.Pointer());

    if( result == 1 ){  //bad error
        CMN_LOG_CLASS_RUN_ERROR<<"Error: AT FT sensor read failed"<<std::endl;
        //FTSensor.Valid() = false;
    }
    else if (result == 2) {  //saturated
        //For robot Control this is a problem because it prints to the screen
        CMN_LOG_CLASS_RUN_VERBOSE<<"AT FT sensor Saturated"<<std::endl;

        IsSaturated.Data = true;
        //FTSensor.Valid() = false;
    }
    else {
        IsSaturated.Data = false;
        //FTSensor.Valid() = true;
    }

	FTSensorCurrent.Assign(FTSensor);
	FTSensor.Element(0) = FTSensorCurrent.Element(1);
	FTSensor.Element(1) = FTSensorCurrent.Element(0);
	FTSensor.Element(2) = -FTSensorCurrent.Element(2);
	FTSensor.Element(3) = FTSensorCurrent.Element(4);
	FTSensor.Element(4) = FTSensorCurrent.Element(3);
	FTSensor.Element(5) = -FTSensorCurrent.Element(5);
	FTSensorCurrent.Assign(FTSensor);

	for (int i=0; i<3; i++) {
		if(fabs(FTSensorCurrent.Element(i)) < 1)
			FTSensorCurrent.Element(i) = 0.0;
	}
	for (int i=3; i<6; i++) {
		if(fabs(FTSensorCurrent.Element(i)) < 0.3)
			FTSensorCurrent.Element(i) = 0.0;
	}

	for (int i=0; i<6; i++)
		FTSensor.Element(i) = alpha*FTSensorCurrent.Element(i) +
										(1-alpha)*FTSensorPrevious.Element(i);
    FTSensorPrevious.Assign(FTSensor);

}
void devFTSensorATITask::Configure(const std::string & filename, const std::string & devName) {
    // loading FT stuff
    if (0 != (ATIFTSensor.LoadCalibrationFile(filename))){
       CMN_LOG_CLASS_INIT_ERROR<< "Problems with loading ATI FT file" << std::endl;
       osaSleep(3);
    }

    ATIFTSensor.SetForceUnits("N");
    ATIFTSensor.SetTorqueUnits("N-m");

    //this cause a blocking call on the getFTreadings later, choose the arguments wisely
    //10K and 10 avg is done in 3.7ms.
    int ret = ATIFTSensor.StartSingleSampleAcquisition(devName, 10000, 5, 0, false);
    if (ret == -1){
       CMN_LOG_CLASS_INIT_ERROR<< "Could not start Aquisition from the DAQ card for ATI FT file" << std::endl;
        osaSleep(3);
    }

    CMN_LOG_CLASS_RUN_VERBOSE<< "ATI FT Sensor loaded: " << ATIFTSensor.GetSerialNumber()<<std::endl;
    BiasCurrentLoad();
}

void devFTSensorATITask::SetTransform(const mtsDoubleVec &values){
    if(values.size()>6){
       CMN_LOG_CLASS_INIT_ERROR<< "ATI FT Sensor  Transform : wrong size" << std::endl;
        return;
    }
    float64	transform[6];

    for(unsigned int i=0;i<6;i++){
        transform[i]=values[i];   
    }
     /*
    In: The units of the displacements. Acceptable values are "in", "m", "mm", "cm", or "ft". 
    angleUnits
    In: The units of the rotations. Acceptable values are "deg", "degrees", "degree", "rad", "radians", or "radian".*/
    //displacement along x axis
    //transform[1]=-50;       //displacement along y axis
    //transform[2]=0;         //displacement along z axis
    //transform[3]=0;         //rotation about x axis (right-hand rotation)
    //transform[4]=0;         //rotation about y axis (right-hand rotation)
    //transform[5]=45;        //rotatation about z axis (right-hand rotation)
    ATIFTSensor.ToolTransform(transform, "mm", "degrees");

}

void devFTSensorATITask::BiasCurrentLoad(void){
   CMN_LOG_CLASS_RUN_ERROR << "BiasFTSensor " << this->GetName() << std::endl;    
    if (0!=ATIFTSensor.BiasCurrentLoad()){
        CMN_LOG_CLASS_INIT_VERBOSE << "Error Biasing FT Sensor" << ATIFTSensor.GetSerialNumber()<< std::endl;
    }
}

void devFTSensorATITask::GetSerialNumber(std::string & serial) const{
    CMN_LOG_CLASS_RUN_VERBOSE << "Getting FT Serial Number " << ATIFTSensor.GetSerialNumber()<< std::endl;
    serial=ATIFTSensor.GetSerialNumber();
}

void  devFTSensorATITask::GetMaxLoads(mtsDoubleVec &values) const{
    if (values.size()!=6){
        CMN_LOG_CLASS_INIT_VERBOSE << "GetFTSensorMaxLoads argument is wrong size :" << this->GetName() << std::endl;
        return;
    }
    CMN_LOG_CLASS_RUN_VERBOSE << "Getting GetFTSensorMaxLoads " << this->GetName() << std::endl;
    for (unsigned int i=0; i<6;i++){
        values[i]=ATIFTSensor.GetMaxLoad(i);
    }
}
