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

#include <cisstConfig.h>
#include <cisstCommon/cmnLogger.h>
#include <cisstCommon/cmnKbHit.h>
#include <cisstCommon/cmnGetChar.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <sawOptoforceSensor/mtsOptoforce3D.h>

class OptoforceSensorClient : public mtsTaskMain {

private:

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

public:

    OptoforceSensorClient() : mtsTaskMain("OptoforceSensorClient")
    {
        mtsInterfaceRequired *req = AddInterfaceRequired("Input", MTS_OPTIONAL);
        if (req) {
            req->AddFunction("GetTaskPeriod", GetTaskPeriod);
            req->AddFunction("GetCount", GetCount);
            req->AddFunction("GetStatus", GetStatus);
            req->AddFunction("GetForce", GetForce);
            req->AddFunction("GetForceTorque", GetForceCartesian);
            req->AddFunction("GetConnected", GetConnected);
            req->AddFunction("GetSensorConfig", GetSensorConfig);
            req->AddFunction("SetSensorConfig", SetSensorConfig);
            req->AddFunction("Rebias", Zero);
            req->AddFunction("Unbias", UnZero);
            req->AddFunction("GetBias", GetBias);
            req->AddFunction("SetBias", SetBias);
            req->AddFunction("GetLength", GetLength);
            req->AddFunction("SetLength", SetLength);
            req->AddFunction("GetScale", GetScale);
            req->AddFunction("SetScale", SetScale);
        }
    }

    void Configure(const std::string&) {}

    void Startup() {
        vctDouble3 scale;
        GetScale(scale);
        std::cout << "Force sensor scale = " << scale << std::endl;
        std::cout << "Commands: l = set length, s = software bias, z = hardware bias, u = hardware unbias, q = quit"
                  << std::endl << std::endl;
    }

    void Run() {

        ProcessQueuedCommands();

        prmForceCartesianGet force;
        unsigned short count, status;
        double period;    // task period in seconds
        GetTaskPeriod(period);
        GetForceCartesian(force);
        GetCount(count);
        GetStatus(status);

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
                case 'q':   // quit program
                    std::cout << "Exiting.. " << std::endl;
                    this->Kill();
                    break;
                }
            }
            //      std::cout << force.Force() << std::endl;
            printf("%8.2lf || %8.2lf || %8.2lf || %8hu || %04hx || %8.3lf\r",
                   force.Force().X(), force.Force().Y(), force.Force().Z(),
                   count, status, period);
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
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClass("mtsOptoforce3D", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cout, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

#if CISST_HAS_JSON
    if (argc < 3) {
        std::cerr << "Syntax: OptoforceExample <port> <json-cfg>" << std::endl;
        std::cerr << "        <port>      port number (>=1) or port name" << std::endl;
        std::cerr << "        <json-cfg>  configuration file (JSON format)" << std::endl;
        return 0;
    }
#else
    if (argc < 2) {
        std::cerr << "Syntax: OptoforceExample <port>" << std::endl;
        std::cerr << "        <port>      port number (>=1) or port name" << std::endl;
        return 0;
    }
#endif
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

    mtsComponentManager * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(opto);

    OptoforceSensorClient client;
    componentManager->AddComponent(&client);

    if (!componentManager->Connect(client.GetName(), "Input", opto->GetName(), "Force")) {
        std::cout << "Failed to connect: "
            << client.GetName() << "::Input to "
            << opto->GetName() << "::Force" << std::endl;
        delete opto;
        return -1;
    }

    componentManager->CreateAll();

    componentManager->StartAll();

    // Main thread passed to client task

    opto->Kill();
    componentManager->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);

    componentManager->Cleanup();

    // stop all logs
    cmnLogger::SetMask(CMN_LOG_ALLOW_NONE);
    delete opto;

    return 0;
}
