/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  (C) Copyright 2016-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstConfig.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <sawOptoforceSensor/mtsOptoforce3D.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstNumerical/nmrGaussJordanInverse.h>
#if CISST_HAS_JSON
#include <cisstVector/vctDataFunctionsFixedSizeVectorJSON.h>
#include <cisstVector/vctDataFunctionsFixedSizeMatrixJSON.h>
#endif
#if (CISST_OS != CISST_WINDOWS)
#include <byteswap.h>
#endif


CMN_IMPLEMENT_SERVICES_DERIVED(mtsOptoforce3D, mtsTaskContinuous);

mtsOptoforce3D::mtsOptoforce3D(const std::string &name, unsigned int port) : mtsTaskContinuous(name)
{
    mSerialPort.SetPortNumber(port);
    Init();
}

mtsOptoforce3D::mtsOptoforce3D(const std::string &name, const std::string &portName) : mtsTaskContinuous(name)
{
    mSerialPort.SetPortName(portName);
    Init();
}

void mtsOptoforce3D::Init(void)
{
    StateTable.AddData(mCount, "Count");
    StateTable.AddData(mStatus, "Status");
    StateTable.AddData(mRawSensor, "ForceRaw");
    StateTable.AddData(mForce, "Force");
    StateTable.AddData(m_measured_cf, "measured_cf");
    StateTable.AddData(mConnected, "Connected");

    mInterface = this->AddInterfaceProvided("Force");
    if (mInterface) {
        mInterface->AddMessageEvents();
        mInterface->AddCommandReadState(StateTable, mCount, "GetCount");
        mInterface->AddCommandReadState(StateTable, mStatus, "GetStatus");
        mInterface->AddCommandReadState(StateTable, mRawSensor, "GetForceRaw");
        mInterface->AddCommandReadState(StateTable, mForce, "GetForce");
        mInterface->AddCommandReadState(StateTable, m_measured_cf, "measured_cf");
        mInterface->AddCommandReadState(StateTable, mConnected, "GetConnected");
        mInterface->AddCommandReadState(StateTable, StateTable.Period, "GetTaskPeriod");
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats, "get_period_statistics");
        mInterface->AddCommandRead(&mtsOptoforce3D::GetSensorConfig, this, "GetSensorConfig");
        mInterface->AddCommandWrite(&mtsOptoforce3D::SetSensorConfig, this, "SetSensorConfig");
        mInterface->AddCommandRead(&mtsOptoforce3D::IsCalibrated, this, "IsCalibrated");
        mInterface->AddCommandVoid(&mtsOptoforce3D::Uncalibrate, this, "Uncalibrate");
        mInterface->AddCommandVoid(&mtsOptoforce3D::Rebias, this, "Rebias");
        mInterface->AddCommandVoid(&mtsOptoforce3D::Unbias, this, "Unbias");
        mInterface->AddCommandRead(&mtsOptoforce3D::GetBias, this, "GetBias");
        mInterface->AddCommandWrite(&mtsOptoforce3D::SetBias, this, "SetBias");
        mInterface->AddCommandRead(&mtsOptoforce3D::GetLength, this, "GetLength");
        mInterface->AddCommandWrite(&mtsOptoforce3D::SetLength, this, "SetLength");
        mInterface->AddCommandRead(&mtsOptoforce3D::GetScale, this, "GetScale");
        mInterface->AddCommandWrite(&mtsOptoforce3D::SetScale, this, "SetScale");
    }

    // Configure the serial port
    mSerialPort.SetBaudRate(osaSerialPort::BaudRate115200);
    mSerialPort.SetCharacterSize(osaSerialPort::CharacterSize8);
    mSerialPort.SetParityChecking(osaSerialPort::ParityCheckingNone);
    mSerialPort.SetStopBits(osaSerialPort::StopBitsOne);
    mSerialPort.SetFlowControl(osaSerialPort::FlowControlNone);

    // Initialize the constant terms in matrix_l
    mL.SetAll(0);
    mL[0][0] = 1;
    mL[1][1] = 1;
    mL[2][2] = 1;

    // Initialize matrix_cal to the identity
    mCal = vctDouble3x3::Eye();
}

void mtsOptoforce3D::Configure(const std::string & filename)
{
    mMatrixAValid = false;
#if CISST_HAS_JSON
    mConfigured = false;
    try {
        std::ifstream jsonStream;
        Json::Value jsonConfig;
        Json::Reader jsonReader;

        jsonStream.open(filename.c_str());
        if (!jsonStream) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: file not found or unable to open" << std::endl
                                     << filename << std::endl;

            return;
        }
        if (!jsonReader.parse(jsonStream, jsonConfig)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration" << std::endl
                                     << filename << std::endl
                                     << jsonReader.getFormattedErrorMessages();
            return;
        }
        const Json::Value jsonScale = jsonConfig["scale"];
        if (jsonScale.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: cannot find \"scale\" data in " << filename << std::endl;
            return;
        } else {
            cmnDataJSON<vctDouble3>::DeSerializeText(mScale, jsonScale);
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: parsed scale = " << mScale << std::endl;
        }
        const Json::Value jsonSpeed = jsonConfig["speed"];
        if (jsonSpeed.isNull()) {
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: \"speed\" (update rate from sensor) not specified,"
                                       << " using default value of " << (unsigned int)mSensorSpeed << std::endl;
        } else {
            mSensorSpeed = static_cast<unsigned char>(jsonSpeed.asUInt());
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: parsed speed (update rate from sensor) = "
                                       << (unsigned int)mSensorSpeed << std::endl;
        }
        const Json::Value jsonFilter = jsonConfig["filter"];
        if (jsonFilter.isNull()) {
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: \"filter\" (cutoff frequency) not specified,"
                                       << " using default value of " << (unsigned int)mSensorFilter << std::endl;
        } else {
            mSensorFilter = static_cast<unsigned char>(jsonFilter.asUInt());
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: parsed filter (cutoff frequency) = "
                                       << (unsigned int)mSensorFilter << std::endl;
        }
        const Json::Value jsonCalMatrix = jsonConfig["cal-matrix"];
        if (jsonCalMatrix.isNull()) {
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: \"cal-matrix\" not specified" << std::endl;
        } else {
            cmnDataJSON<vctDouble3x6>::DeSerializeText(mA, jsonCalMatrix);
            mMatrixAValid = true;
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: parsed cal-matrix (A) = " << std::endl
                                       << mA << std::endl;
        }
        mConfigured = true;
    } catch (const std::runtime_error &e) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: runtime_error parsing JSON file: "
                                 << e.what() << std::endl;
    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: make sure file \""
                                 << filename << "\" is in JSON format" << std::endl;
    }
#else
    // If cisst is not compiled with JSON support, the software returns the raw force values by default.
    CMN_LOG_CLASS_INIT_WARNING << "Configure: JSON support not enabled in cisst, setting scale to 1" << std::endl;
    mScale.SetAll(1.0);
    mConfigured = true;
#endif
}

void mtsOptoforce3D::Startup(void)
{
    if (!mConfigured) {
        mInterface->SendError("Startup: cannot start because component was not correctly configured");
    }
    else {
        // true --> open serial port in blocking mode
        if (!mSerialPort.Open(true)) {
            mInterface->SendError("Startup: cannot open serial port: " + mSerialPort.GetPortName());
        }
        else {
            mConnected = true;
            mInterface->SendStatus("Startup: serial port " + mSerialPort.GetPortName() + " successfully opened");
            // Send the speed, filter, and bias to the sensor so we know how it is configured
            SendCommand(mSensorSpeed, mSensorFilter, mSensorBias);
        }
    }
}

void mtsOptoforce3D::Run(void)
{
    struct optopacket {
        unsigned char header[4];
        unsigned short count;
        unsigned short status;
        short fx;
        short fy;
        short fz;
        unsigned short checksum;
    };

    union PacketDataType {
        unsigned char bytes[16];
        optopacket packet;
    };

    PacketDataType buffer;
    ProcessQueuedCommands();

    if (mConnected) {
        bool found = false;
        unsigned short recvChecksum;
#if (CISST_OS == CISST_WINDOWS)
        // On Windows, mSerialPort.Read seems to always return the requested number
        // of characters, which is sizeof(buffer).
        // Thus, we have to check whether part of the expected packet has been combined
        // with another packet, such as the 7 byte response to the command sent to the sensor.
        int n = mSerialPort.Read((char *)&buffer, sizeof(buffer));
        while (!found) {
            for (int i = 0; i < n - 3; i++) {
                if ((buffer.bytes[i] == 170) && (buffer.bytes[i + 1] == 7)
                    && (buffer.bytes[i + 2] == 8) && (buffer.bytes[i + 3] == 10)) {
                    if (i != 0) {                               // If pattern not found at beginning of buffer
                        memmove(buffer.bytes, buffer.bytes + i, n - i);    //    shift so that 170 is in buffer[0]
                        mSerialPort.Read(buffer.bytes + n - i, i);          //    fill the rest of the buffer
                    }
                    found = true;
                    break;
                }
            }
            if (!found) {                                       // If pattern not yet found
                memmove(buffer.bytes, buffer.bytes + n - 4, 4);               //    move last 4 characters to beginning of buffer
                mSerialPort.Read(buffer.bytes + 4, sizeof(buffer.bytes) - 4);  //    get another 12 characters
            }
        }
        // Now, process the data
        mRawSensor.X() = (double)static_cast<short>(_byteswap_ushort(buffer.packet.fx)) * mScale.X();
        mRawSensor.Y() = (double)static_cast<short>(_byteswap_ushort(buffer.packet.fy)) * mScale.Y();
        mRawSensor.Z() = (double)static_cast<short>(_byteswap_ushort(buffer.packet.fz)) * mScale.Z();
        mCount = _byteswap_ushort(buffer.packet.count);
        mStatus = _byteswap_ushort(buffer.packet.status);
        recvChecksum = _byteswap_ushort(buffer.packet.checksum);
#else
        // On Linux, mSerialPort.Read seems to return a complete packet, even if it is less than the
        // requested size.
        // Thus, we can discard packets that are not the correct size.
        while (!found) {
            if (mSerialPort.Read((char *)&buffer, sizeof(buffer)) == sizeof(buffer)) {
                // Check for expected 4 byte packet header
                found = ((buffer.bytes[0] == 170) && (buffer.bytes[1] == 7)
                         && (buffer.bytes[2] == 8) && (buffer.bytes[3] == 10));
            }
        }
        // Now, process the data
        mRawSensor.X() = (double)static_cast<short>(bswap_16(buffer.packet.fx)) * mScale.X();
        mRawSensor.Y() = (double)static_cast<short>(bswap_16(buffer.packet.fy)) * mScale.Y();
        mRawSensor.Z() = (double)static_cast<short>(bswap_16(buffer.packet.fz)) * mScale.Z();
        mCount = bswap_16(buffer.packet.count);
        mStatus = bswap_16(buffer.packet.status);
        recvChecksum = bswap_16(buffer.packet.checksum);
#endif
        // Verify the checksum (last 2 bytes).
        unsigned short checksum = buffer.bytes[0];
        for (size_t i = 1; i < sizeof(buffer)-2; i++) {
            checksum += buffer.bytes[i];
        }
        // (mStatus == 0) means no errors or overload warnings.
        // For now, we check ((mStatus&0xFC00) == 0), which ignores overload warnings.
        bool valid = (checksum == recvChecksum) && ((mStatus&0xFC00) == 0);
        m_measured_cf.SetValid(valid);

        if (valid) {
            if (mMatrixAValid) {
                // Obtain forces by applying the calibration matrix, which depends on sensor-specific calibration
                // values (A) and the assumed length to where the forces are applied (Length), which determines mL (L).
                // The calibration matrix, mCal, is equal to inv(A*L)
                mForce = mCal * mRawSensor - mBias;  // F = inv(A*L)*S - bias
            }
            else {
                mForce = mRawSensor - mBias;
            }
            m_measured_cf.SetForce(vctDouble6(mForce.X(), mForce.Y(), mForce.Z(), 0.0, 0.0, 0.0));
        }
    }
    else {
        m_measured_cf.SetValid(false);
        osaSleep(0.1);  // If not connected, wait
    }
}

void mtsOptoforce3D::Cleanup(void)
{
    // Close the port
    if (mConnected) {
        mSerialPort.Close();
        mConnected = false;
    }
}

void mtsOptoforce3D::SendCommand(unsigned char speed, unsigned char filter,
                                 unsigned char zero)
{
    if (!mConnected) {
        return;
    }

    struct configpacket {
        unsigned char header[4];
        unsigned char speed;
        unsigned char filter;
        unsigned char zero;
        unsigned char checksum[2];
    };
    configpacket Buffer;
    // Send command via serial port

    Buffer.header[0] = 170;
    Buffer.header[1] = 0;
    Buffer.header[2] = 50;
    Buffer.header[3] = 3;
    Buffer.speed = speed;
    Buffer.filter = filter;
    Buffer.zero = zero;
    unsigned short checksum = 223 + speed + filter + zero;
    Buffer.checksum[0] = (checksum >> 8)&0x00ff;
    Buffer.checksum[1] = checksum&0x00ff;

    mSerialPort.Write((char*)&Buffer, sizeof(Buffer));

    // Optoforce sensor returns a packet with 7 bytes
    // 170 0 80 1 X CS0 CS1
    //   The X byte is 0 if no error.
    //   CS0,CS1 are the checksum
}

void mtsOptoforce3D::SetSensorConfig(const vctUChar3 &parms)
{
    // Currently, not checking for valid values
    mSensorSpeed = parms.X();
    mSensorFilter = parms.Y();
    mSensorBias = parms.Z();
    SendCommand(mSensorSpeed, mSensorFilter, mSensorBias);
}

void mtsOptoforce3D::GetSensorConfig(vctUChar3 &parms) const
{
    // Only returns local (shadow) copies, since there does not appear
    // to be a way to query the sensor.
    parms.X() = mSensorSpeed;
    parms.Y() = mSensorFilter;
    parms.Z() = mSensorBias;
}

void mtsOptoforce3D::IsCalibrated(bool &flag) const
{
    flag = mMatrixAValid;
}

void mtsOptoforce3D::Uncalibrate(void)
{
    mMatrixAValid = false;
}

// Note that two consecutive calls to Rebias will not work;
// it is necessary to call Unbias in between and wait at least
// 2 msec before calling Rebias again.
void mtsOptoforce3D::Rebias(void)
{
    mSensorBias = 255;
    SendCommand(mSensorSpeed, mSensorFilter, mSensorBias);
}

void mtsOptoforce3D::Unbias(void)
{
    mSensorBias = 0;
    SendCommand(mSensorSpeed, mSensorFilter, mSensorBias);
}

void mtsOptoforce3D::GetBias(vctDouble3 &b) const
{
    b = mBias;
}

void mtsOptoforce3D::SetBias(const vctDouble3 &b)
{
    mBias= b;
}

void mtsOptoforce3D::GetLength(vctDouble3 &len) const
{
    len = mLength;
}

void mtsOptoforce3D::SetLength(const vctDouble3 &len)
{
    if (!mMatrixAValid) {
        CMN_LOG_CLASS_RUN_WARNING << "SetLength: mA is not valid (length ignored)" << std::endl;
        return;
    }

    mLength = len;

    // Update mL elements that depend on length
    mL[3][1] = mLength.Z();
    mL[3][2] = -mLength.Y();
    mL[4][0] = -mLength.Z();
    mL[4][2] = mLength.X();
    mL[5][0] = mLength.Y();
    mL[5][1] = -mLength.X();

    // Now, compute inverse of A*L
    vctDouble3x3 matrix_product = mA * mL;
    vctDouble3x3 matrix_product_inverse;
    bool cal_valid;

    nmrGaussJordanInverse3x3(matrix_product, cal_valid, matrix_product_inverse, 0.0);

    if (cal_valid) {
        mCal = matrix_product_inverse;   // if nonsingular, update class member mCal
    } else {
        CMN_LOG_CLASS_RUN_WARNING << "SetLength: calibration matrix is singular" << std::endl;
    }
}

void mtsOptoforce3D::GetScale(vctDouble3 &s) const
{
    s = mScale;
}

void mtsOptoforce3D::SetScale(const vctDouble3 &s)
{
    mScale = s;
    mConfigured = true;
}
