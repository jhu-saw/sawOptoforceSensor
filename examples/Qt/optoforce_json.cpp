/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-21

  (C) Copyright 2014-2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

/*!
  \file
  \brief An example interface for NDI trackers with serial interface.
  \ingroup devicesTutorial
*/

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <sawOptoforceSensor/mtsOptoforce3D.h>
#include <sawOptoforceSensor/mtsForceTorqueQtWidget.h>

#include <QApplication>
#include <QMainWindow>


int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsOptoforce3D", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // parse options
    cmnCommandLineOptions options;
    std::string jsonConfigFile = "";
    std::string serialPort = "";

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &jsonConfigFile);
    options.AddOptionOneValue("s", "serial-port",
                              "serial port as a string",
                              cmnCommandLineOptions::REQUIRED_OPTION, &serialPort);

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    // create the components
    mtsOptoforce3D * sensor = new mtsOptoforce3D("Optoforce3D", serialPort);
    sensor->Configure(jsonConfigFile);

    // add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(sensor);

    // create a Qt user interface if needed
    QApplication * application = new QApplication(argc, argv);
    QTabWidget * tabWidget = new QTabWidget;
    mtsForceTorqueQtWidget * sensorWidget = new mtsForceTorqueQtWidget("Optoforce3D-GUI");
    tabWidget->addTab(sensorWidget, "OptoForce");
    componentManager->AddComponent(sensorWidget);
    componentManager->Connect(sensorWidget->GetName(), "ForceSensor",
                              sensor->GetName(), "Force");

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    tabWidget->show();
    application->exec();

    // kill all components and perform cleanup
    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    cmnLogger::Kill();

    return 0;
}
