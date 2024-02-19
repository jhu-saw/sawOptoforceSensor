/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


#ifndef _mtsOptoforceSensor_h
#define _mtsOptoforceSensor_h

#include <cisstMultiTask/mtsForwardDeclarations.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstOSAbstraction/osaSerialPort.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <sawOptoforceSensor/sawOptoforceSensorRevision.h>

// Always include last
#include <sawOptoforceSensor/sawOptoforceSensorExport.h>

// Following uses same naming convention as vctFixedSizeVectorTypes.h
typedef vctFixedSizeMatrix<double, 3, 6> vctDouble3x6;
typedef vctFixedSizeMatrix<double, 6, 3> vctDouble6x3;

class CISST_EXPORT mtsOptoforce3D: public mtsTaskContinuous {

    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

 public:
    mtsOptoforce3D(const std::string &name, unsigned int port);
    mtsOptoforce3D(const std::string &name, const std::string &portName);
    ~mtsOptoforce3D(void) {};
    void Configure(const std::string &filename = "") override;
    void Startup(void) override;
    void Run(void) override;
    void Cleanup(void) override;

 protected:
    mtsInterfaceProvided * mInterface;

    unsigned short mCount;     // Counter returned by force sensor
    unsigned short mStatus;    // Force sensor status
    vctDouble3 mRawSensor;     // Raw sensor readings (not calibrated)
    vctDouble3 mForce;         // Force values (x, y, z)
    vctDouble3 mLength
        = vctDouble3(0.0);     // offset vector to where force is resolved
    prmForceCartesianGet m_measured_cf; // Force/torque (torque is 0)

    vctDouble3 mBias
        = vctDouble3(0.0);      // Force sensor bias, in counts
    vctDouble3 mScale
        = vctDouble3(1.0);      // Force sensor scale, Newtons/count
    vctDouble3x6 mA;            // intermediate calibration matrix (A)
    vctDouble6x3 mL
        = vctDouble6x3(0.0);    // matrix used to store lengths (L)
    vctDouble3x3 mCal;          // final calibration matrix, inv(A*L)
    bool mMatrixAValid = false; // Whether matrix_a is valid

    // shadow variables for sensor speed and filter
    unsigned char mSensorSpeed = 10; // 100Hz
    unsigned char mSensorFilter = 4; // 15Hz
    unsigned char mSensorBias = 0;   // unbias

    void Init(void);          // Initialization (called from constructors)
    void SendCommand(unsigned char speed, unsigned char filter, unsigned char zero);
    void GetSensorConfig(vctUChar3 &parms) const;  // Get speed, filter, and bias (returns shadow copies)
    void SetSensorConfig(const vctUChar3 &parms);  // Set speed, filter, and bias (sends command to sensor)
    void IsCalibrated(bool &flag) const;           // Returns whether sensor is calibrated (matrix_a_valid)
    void Uncalibrate(void);                        // Removes user calibration
    void Rebias(void);        // Bias force sensor (sends command to sensor)
    void Unbias(void);        // Remove previous bias (sends command to sensor)
    void GetBias(vctDouble3 &b) const;      // Current software bias
    void SetBias(const vctDouble3 &b);      // Set software bias
    void GetLength(vctDouble3 &len) const;  // Get length offset to where force is resolved
    void SetLength(const vctDouble3 &len);  // Set length offset to where force is resolved
    void GetScale(vctDouble3 &s) const;     // Get sensor scale (N/bit)
    void SetScale(const vctDouble3 &s);     // Set sensor scale (N/bit)
    osaSerialPort mSerialPort;
    bool mConfigured = false;
    bool mConnected = false;
};


CMN_DECLARE_SERVICES_INSTANTIATION(mtsOptoforce3D);


#endif // _mtsOptoforce3D_h
