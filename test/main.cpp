/*-*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-   */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
(C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnKbHit.h>
#include <cisstCommon/cmnGetChar.h>
#include <cisstConfig.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <devFTSensorATITask.h>
#include "mtsOptoforce3D.h"
#include <cisstCommon/cmnLogger.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <iostream>

#include <stdio.h>
#include <stdlib.h>

class ftATISensorClient : public mtsTaskMain {

private:

    mtsFunctionRead GetFTSensorData;
    mtsFunctionRead GetIsSaturated;  
    mtsFunctionRead GetMaxLoads;
    mtsFunctionVoid BiasFTSensor;
    mtsFunctionWrite SetTransform;

	vct6 FTSensorData;
	mtsBool FTIsSaturated;

	mtsFunctionRead GetCount;
    mtsFunctionRead GetStatus;
    mtsFunctionRead GetForce;
    mtsFunctionRead GetForceCartesian;
    mtsFunctionRead GetConnected;
    mtsFunctionRead GetTaskPeriod;
    mtsFunctionRead GetSensorConfig;
    mtsFunctionWrite SetSensorConfig;
    mtsFunctionVoid Zero;
    mtsFunctionVoid UnZero;
    mtsFunctionRead GetBias;
    mtsFunctionWrite SetBias;
    mtsFunctionRead GetLength;
    mtsFunctionWrite SetLength;
    mtsFunctionRead GetScale;
    mtsFunctionWrite SetScale;

	vct3 optoforce;

	std::vector<vct3> ati;
	std::vector<vct3> opto;

	bool flag;
public:

    ftATISensorClient() : mtsTaskMain("ftATISensorClient"),
						  flag(false)
    {
        mtsInterfaceRequired *req = AddInterfaceRequired("atiforce", MTS_OPTIONAL);
        if (req) {
			req->AddFunction("GetState", GetFTSensorData);
			req->AddFunction("BiasCurrentLoad", BiasFTSensor);
            req->AddFunction("GetIsSaturated", GetIsSaturated);
            req->AddFunction("GetMaxLoads", GetMaxLoads);     
            req->AddFunction("SetTransform", SetTransform);

        }

		mtsInterfaceRequired *required = AddInterfaceRequired("optoforce", MTS_OPTIONAL);
        if (required) {
            required->AddFunction("GetTaskPeriod", GetTaskPeriod);
            required->AddFunction("GetCount", GetCount);
            required->AddFunction("GetStatus", GetStatus);
            required->AddFunction("GetForce", GetForce);
            required->AddFunction("GetForceTorque", GetForceCartesian);
            required->AddFunction("GetConnected", GetConnected);
            required->AddFunction("GetSensorConfig", GetSensorConfig);
            required->AddFunction("SetSensorConfig", SetSensorConfig);
            required->AddFunction("Rebias", Zero);
            required->AddFunction("Unbias", UnZero);
            required->AddFunction("GetBias", GetBias);
            required->AddFunction("SetBias", SetBias);
            required->AddFunction("GetLength", GetLength);
            required->AddFunction("SetLength", SetLength);
            required->AddFunction("GetScale", GetScale);
            required->AddFunction("SetScale", SetScale);
        }
    }

    void Configure(const std::string&) {}

    void Startup() {
		vctDouble3 scale;
        GetScale(scale);
        std::cout << "Force sensor scale = " << scale << std::endl;
        std::cout << "Commands: l = set length, s = software bias, z = hardware bias, u = hardware unbias, b = bias the atisensor, w = write sensor reading data into json file, q = quit"
                  << std::endl << std::endl;}

	void Serialization(const std::string &filename)
        {
            std::ofstream jsonStream;
            Json::Value jsonConfig;
            Json::StyledWriter jsonWriter;

			Json::Value atisensorforce, optosensorforce;
            cmnDataJSON<std::vector<vct3>>::SerializeText(ati, atisensorforce);
			cmnDataJSON<std::vector<vct3>>::SerializeText(opto, optosensorforce);
			jsonConfig["ati-readings"] = atisensorforce;
			jsonConfig["sawopto-readings"] = optosensorforce;
		
			jsonStream.open(filename);
			jsonStream << jsonWriter.write(jsonConfig) << std::endl;
			jsonStream.close();
        }

    void Run() {

		GetIsSaturated(FTIsSaturated);

		if (!FTIsSaturated){
		GetFTSensorData(FTSensorData);

		}
		prmForceCartesianGet force;
		GetForceCartesian(force);

		if (force.Valid()) {
		if (cmnKbHit()) {

			 vctDouble3 b;
                vctDouble3 len;
                char c = cmnGetChar();
                switch (c) {
                case 'l':   // Set length
                    std::cout << "Please specify the position where forces are applied." << std::endl;
                    std::cin >> len.X() >> len.Y() >> len.Z();
                    SetLength(len);
                    break;
                case 's':   // Software bias
                    GetBias(b);
                    std::cout << std::endl << "Current force bias = " << b << std::endl;
                    b.SetAll(0);
                    SetBias(b);
                    osaSleep(0.1);
                    for (int i = 0; i < 50; i++) {
                        prmForceCartesianGet force;
                        GetForceCartesian(force);
                        b[0] += force.Force().X();
                        b[1] += force.Force().Y();
                        b[2] += force.Force().Z();
                    }
                    b.Divide(50);
                    osaSleep(0.1);
                    SetBias(b);
                    std::cout << "New force bias = " << b << std::endl;
                    break;
                case 'z':   // Hardware bias (send 9 byte packet to sensor)
                    std::cout << "Biasing.. " << std::endl;
                    Zero();
                    break;
                case 'u':   // Hardware unbias (send 9 byte packet to sensor)
                    std::cout << "Unbiasing.. " << std::endl;
                    UnZero();
                    break;
				case 'b':
					BiasFTSensor();
					flag = true;
					break;
				case 'w':
					Serialization("rawdata.json");
					break;
                case 'q':   // quit program
                    std::cout << "Exiting.. " << std::endl;
                    this->Kill();
                    break;
                }
		}
		optoforce.X() = force.Force().X();
		optoforce.Y() = force.Force().Y();
		optoforce.Z() = force.Force().Z();

		vct3 atiforce;
		atiforce.X() = FTSensorData.X();
		atiforce.Y() = FTSensorData.Y();
		atiforce.Z() = FTSensorData.Z();

		vct6 ati_opto;
		ati_opto[0] = atiforce.X();
		ati_opto[1] = atiforce.Y();
		ati_opto[2] = atiforce.Z();
		ati_opto[3] = optoforce.X();
		ati_opto[4] = optoforce.Y();
		ati_opto[5] = optoforce.Z();

		if (flag) {
			ati.push_back(atiforce);
			opto.push_back(optoforce);
		}

		printf("%8.2lf || %8.2lf || %8.2lf || %8.2lf || %8.2lf || %8.2lf\r",
             ati_opto[1], ati_opto[0], ati_opto[2], ati_opto[3], ati_opto[4], ati_opto[5]);
	}

        else
            printf("Force invalid                                 \r");

        osaSleep(0.01);  // to avoid taking too much CPU time
    }

    void Cleanup() {}

};


int main(int argc, char **argv)
{
	cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ALL);


	mtsOptoforce3D *opto;
    int portNum;
    if (sscanf(argv[1], "%d", &portNum) == 1) {
        std::cout << "Selecting port number: " << portNum << std::endl;
        opto = new mtsOptoforce3D("OptoforceSensor", portNum);
    }
    else {
        std::cout << "Selecting port name: " << argv[1] << std::endl;
        opto = new mtsOptoforce3D("OptoforceSensor", argv[1]);
    }

    #if CISST_HAS_JSON
		opto->Configure(argv[2]);
	#else
		opto->Configure();
	#endif

	devFTSensorATITask   *ATIFTTask   = new devFTSensorATITask("atiFTTask", 50* cmn_ms);

	ATIFTTask->Configure("C:/Projects/OptoforceTest/cisst/devFTSensorATILib/code/calibration/FT13793.cal", "Dev3");


    mtsComponentManager * componentManager = mtsComponentManager::GetInstance();
	componentManager->AddComponent(opto);
    componentManager->AddComponent(ATIFTTask);

    ftATISensorClient client;
    componentManager->AddComponent(&client);

    if (!componentManager->Connect(client.GetName(), "atiforce", ATIFTTask->GetName(), "ProvidesATIFTSensor")) {
        std::cout << "Failed to connect: "
            << client.GetName() << "::Input to "
            << ATIFTTask->GetName() << "::Force" << std::endl;
        delete ATIFTTask;
        return -1;
    }

	if (!componentManager->Connect(client.GetName(), "optoforce", opto->GetName(), "Force")) {
        std::cout << "Failed to connect: "
            << client.GetName() << "::Input to "
            << opto->GetName() << "::Force" << std::endl;
        delete opto;
        return -1;
    }

    componentManager->CreateAll();

    componentManager->StartAll();

    // Main thread passed to client task
	componentManager->KillAll();
    componentManager->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);

    componentManager->Cleanup();

    // stop all logs
    cmnLogger::SetMask(CMN_LOG_ALLOW_NONE);
  
    return 0;
}
