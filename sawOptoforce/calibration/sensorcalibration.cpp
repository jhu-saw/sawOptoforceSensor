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
#include <cisstNumerical/nmrLSqLin.h>
#include <cisstNumerical/nmrPInverse.h>

typedef vctFixedSizeVector<double, 6> vctDouble6;

class SensorCalibration : public mtsTaskMain {

private:

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

protected:
    std::vector<vctDouble6> force_pos;    // The vector contains information of applied force and its location
    vctDouble3 Moment;                    //Moment calculated by stan_force and position
    vctDynamicMatrix<double> Matrix_f;
    vctDynamicMatrix<double> f_inverse;
    vctDynamicVector<double> Vector_a;
//	vctDynamicMatrix<double> Matrix_a;
    vctDouble3 SensorRaw;				 //Raw sensor readings used for getting Matrix_s
    std::vector<vctDouble3> S_raw;       //The matrix containing raw sensor readings
    vctDynamicVector<double> S;


public:

    SensorCalibration() : mtsTaskMain("SensorCalibration")
    {
        mtsInterfaceRequired *req = AddInterfaceRequired("Input", MTS_OPTIONAL);
        if (req) {
            req->AddFunction("GetTaskPeriod", GetTaskPeriod);
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
        std::cout << "Commands: p = print menu, z = hardware bias, u = hardware unbias, q = quit"
                  << std::endl << std::endl;
    }
    
    void Run() {

//		ProcessQueuedCommands();

        prmForceCartesianGet force;
        unsigned short count, status;
        double period;    // task period in seconds
        GetTaskPeriod(period);
        GetForceCartesian(force);
        GetStatus(status);

        if (force.Valid()) {
            if (cmnKbHit()) {
                char c = cmnGetChar();
                switch (c) {
                case 'p':   // Print menu
                    PrintMenu();
                    break;
                //case 'z':   // Hardware bias (send 9 byte packet to sensor)
                //	std::cout << "Biasing.. " << std::endl;
                //	Zero();
                //	break;
                //case 'u':   // Hardware unbias (send 9 byte packet to sensor)
                //	std::cout << "Unbiasing.. " << std::endl;
                //	UnZero();
                //	break;
                //case 'q':   // quit program
                //	std::cout << "Exiting.. " << std::endl;
                //	this->Kill();
                //	break;
                }
            }

                //      std::cout << force.Force() << std::endl;
                /*printf("%8.2lf || %8.2lf || %8.2lf || %8hu || %04hx || %8.3lf\r",
                    force.Force().X(), force.Force().Y(), force.Force().Z(),
                    count, status, period);*/
        }
        else
            printf("Force invalid                                 \r");

        osaSleep(0.01);  // to avoid taking too much CPU time
    }

    void Getforce_pos() {

        /*std::ofstream jsonStream;
        Json::Value jsonConfig;
        Json::StyledWriter jsonWriter;*/

        vctDouble6 b;
        std::cout << "Please specify the force (N) used to calculate the calibration matrix" << std::endl;
        std::cin >> b[0] >> b[1] >> b[2];
        std::cout << "Please specify the position of the applied force" << std::endl;
        std::cin >> b[3] >> b[4] >> b[5];
        force_pos.push_back(b);

        /*Json::Value Force_pos;
        cmnDataJSON<vctDouble6>::SerializeText(b, Force_pos);
        jsonConfig["force_position"] = Force_pos;

        jsonStream.open(filename);
        jsonStream << jsonWriter.write(jsonConfig) << std::endl;
        jsonStream.close();*/
    }

    void GetSensorReading(){

        /*std::ofstream jsonStream;
        Json::Value jsonConfig;
        Json::StyledWriter jsonWriter;*/

        vctDouble3 b;  //Temporal vector to save three sensor readings of each iteration
        b.SetAll(0);
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
        S_raw.push_back(b);

        /*Json::Value Sensorreading;
        cmnDataJSON<vctDouble3>::SerializeText(b, Sensorreading);
        jsonConfig["sensorreading"] = Sensorreading;

        jsonStream.open(filename);
        jsonStream << jsonWriter.write(jsonConfig) << std::endl;
        jsonStream.close();*/
    }
    
    void ForceMoment(void)
    {
        vctDynamicMatrix<double> Matrix_f_tem(force_pos.size() * 3, 18, VCT_COL_MAJOR);
        Matrix_f_tem.SetAll(0);

        for (int i = 0; i < force_pos.size(); i++) {
            Matrix_f_tem[3 * i][0] = force_pos[i][0];
            Matrix_f_tem[3 * i][1] = force_pos[i][1];
            Matrix_f_tem[3 * i][2] = force_pos[i][2];
            Matrix_f_tem[3 * i][3] = force_pos[i][5] * force_pos[i][1] - force_pos[i][4] * force_pos[i][2];
            Matrix_f_tem[3 * i][4] = -force_pos[i][5] * force_pos[i][0] + force_pos[i][3] * force_pos[i][2];
            Matrix_f_tem[3 * i][5] = force_pos[i][4] * force_pos[i][0] - force_pos[i][3] * force_pos[i][1];

            Matrix_f_tem[3 * i + 1][6] = force_pos[i][0];
            Matrix_f_tem[3 * i + 1][7] = force_pos[i][1];
            Matrix_f_tem[3 * i + 1][8] = force_pos[i][2];
            Matrix_f_tem[3 * i + 1][9] = force_pos[i][5] * force_pos[i][1] - force_pos[i][4] * force_pos[i][2];
            Matrix_f_tem[3 * i + 1][10] = -force_pos[i][5] * force_pos[i][0] + force_pos[i][3] * force_pos[i][2];
            Matrix_f_tem[3 * i + 1][11] = force_pos[i][4] * force_pos[i][0] - force_pos[i][3] * force_pos[i][1];

            Matrix_f_tem[3 * i + 2][12] = force_pos[i][0];
            Matrix_f_tem[3 * i + 2][13] = force_pos[i][1];
            Matrix_f_tem[3 * i + 2][14] = force_pos[i][2];
            Matrix_f_tem[3 * i + 2][15] = force_pos[i][5] * force_pos[i][1] - force_pos[i][4] * force_pos[i][2];
            Matrix_f_tem[3 * i + 2][16] = -force_pos[i][5] * force_pos[i][0] + force_pos[i][3] * force_pos[i][2];
            Matrix_f_tem[3 * i + 2][17] = force_pos[i][4] * force_pos[i][0] - force_pos[i][3] * force_pos[i][1];
        }
        Matrix_f = Matrix_f_tem;
    }

    void Calibration_cal()
        {
            vctDynamicVector<double> S_tem(force_pos.size() * 3);
            S_tem.SetAll(0);

            for (int i = 0; i < force_pos.size(); i++) {
                S_tem[3 * i] = S_raw[i][0];
                S_tem[3 * i + 1] = S_raw[i][1];
                S_tem[3 * i + 2] = S_raw[i][2];
            }
            S = S_tem;

            vctDynamicMatrix<double> Matrix_f_copy;  //nmrLSqLin alters the content of Matrix_f and S
            Matrix_f_copy = Matrix_f;
            vctDynamicVector<double> S_copy;
            S_copy = S;
            vctDynamicVector<double> Vector_a_copy(18);
            nmrLSqLin(Matrix_f_copy, S_copy, Vector_a_copy);

            Vector_a = Vector_a_copy;

            std::cout << Vector_a << std::endl;

//			vctDynamicMatrix<double> Matrix_f_copy_inverse;
//			nmrPInverse(Matrix_f_copy, Matrix_f_copy_inverse);
//			Vector_a = Matrix_f_copy_inverse*S_copy;
        }

    void Serialization(const std::string &filename)
        {
            std::ofstream jsonStream;
            Json::Value jsonConfig;
            Json::StyledWriter jsonWriter;

            vctDynamicMatrix<double> Matrix_a(3, 6);
            for (int j = 0; j < 3; j++) {
                for (int k = 0; k < 6; k++) {
                    Matrix_a[j][k] = Vector_a[j * 6 + k];
                }
            }

            Json::Value Matrix_cal, Force_Pos, Sensorreading;
            cmnDataJSON<vctDynamicMatrix<double>>::SerializeText(Matrix_a, Matrix_cal);
            cmnDataJSON<std::vector<vctDouble6>>::SerializeText(force_pos, Force_Pos);
            cmnDataJSON<std::vector<vctDouble3>>::SerializeText(S_raw, Sensorreading);
            jsonConfig["cal-matrix"] = Matrix_cal;
            jsonConfig["force-pos"] = Force_Pos;
            jsonConfig["raw-sensor-reading"] = Sensorreading;

            jsonStream.open(filename);
            jsonStream << jsonWriter.write(jsonConfig) << std::endl;
            jsonStream.close();
        }

    void PrintMenu(){
            std::cout << "1) Enter applied force (fx, fy, fz) and length (x, y, z)" << std::endl << "2) Take corresponding sensor readings" << std::endl
                << "3) Compute calibration matrix" << std::endl << "4) Write results to JSON file" << std::endl
                << "5) Exit" << std::endl << "Select Option:" << std::endl;

            char c = cmnGetChar();
            switch (c) {
            case '1':   // Input length and applied force
                Zero();
                std::cout << "Please put the specific weight to the location you chose" << std::endl;
                Getforce_pos();
                std::cout << "Input force = " << force_pos.back()[0] << " " << force_pos.back()[1] << " "
                    << force_pos.back()[2] << std::endl;
                std::cout << "Location of force = " << force_pos.back()[3] << " " << force_pos.back()[4] << " "
                    << force_pos.back()[5] << std::endl;
                PrintMenu();
                break;
            case '2':   // Measurements with input weight
                GetSensorReading();
                std::cout << "Sensor readings = " << S_raw.back()[0] << " " << S_raw.back()[1] << " "
                    << S_raw.back()[2] << std::endl;
                std::cout << "Please remove the weight from the sensor" << std::endl;
                UnZero();
                PrintMenu();
                break;
            case '3':   // Compute calibration matrix
                std::cout << "The number of groups of measurements you used to calculate calibration matrix = "
                    << force_pos.size() << std::endl;
                ForceMoment();
                Calibration_cal();
                PrintMenu();
                break;
            case '4':   // Write results to a json file
                Serialization("matrix_60.json");
                PrintMenu();
                break;
            case '5':   // quit program
                std::cout << "Exiting.. " << std::endl;
                this->Kill();
                break;
            }
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

    mtsComponentManager * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(opto);

    opto->SensorCalibConfig();

    SensorCalibration client;
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
