/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-21

  (C) Copyright 2014-2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <sawOptoforceSensor/mtsOptoforce3D.h>
#include <sawOptoforceSensor/mtsOptoforce3DQtWidget.h>

#include <cisst_ros_crtk/mts_ros_crtk_bridge_provided.h>

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

    // create ROS node handle
    cisst_ral::ral ral(argc, argv, "optoforce");
    auto rosNode = ral.node();

    // parse options
    cmnCommandLineOptions options;
    std::string jsonConfigFile = "";
    std::string serialPort = "";
    double rosPeriod = 10.0 * cmn_ms;
    std::list<std::string> managerConfig;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &jsonConfigFile);
    options.AddOptionOneValue("s", "serial-port",
                              "serial port as a string",
                              cmnCommandLineOptions::REQUIRED_OPTION, &serialPort);
    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all tool positions (default 0.01, 10 ms, 100Hz).  There is no point to have a period higher than the tracker component",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);
    options.AddOptionMultipleValues("m", "component-manager",
                                    "JSON files to configure component manager",
                                    cmnCommandLineOptions::OPTIONAL_OPTION, &managerConfig);
    options.AddOptionNoValue("D", "dark-mode",
                             "replaces the default Qt palette with darker colors");
    options.AddOptionNoValue("t", "text-only",
                             "text only interface, do not create Qt widgets");

    // check that all required options have been provided
    if (!options.Parse(argc, argv, std::cerr)) {
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    const bool hasQt = !options.IsSet("text-only");

    // create the components
    mtsOptoforce3D * sensor = new mtsOptoforce3D("Optoforce3D", serialPort);
    sensor->Configure(jsonConfigFile);

    // add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(sensor);

    // ROS CRTK bridge
    mts_ros_crtk_bridge_provided * crtk_bridge
        = new mts_ros_crtk_bridge_provided("force_dimension_crtk_bridge", rosNode);
    componentManager->AddComponent(crtk_bridge);

    // create a Qt user interface if needed
    QApplication * application = 0;
    mtsOptoforce3DQtWidget * sensorWidget = 0;
    if (hasQt) {
        application = new QApplication(argc, argv);
        cmnQt::QApplicationExitsOnCtrlC();
        if (options.IsSet("dark-mode")) {
            cmnQt::SetDarkMode();
        }
        sensorWidget = new mtsOptoforce3DQtWidget("Optoforce3D-GUI");
        componentManager->AddComponent(sensorWidget);
        componentManager->Connect(sensorWidget->GetName(), "Device",
                                  sensor->GetName(), "Force");
    }

    crtk_bridge->bridge_interface_provided(sensor->GetName(),
                                           "Force",
                                           "optoforce",
                                           rosPeriod);
    crtk_bridge->Connect();

    // custom user components
    if (!componentManager->ConfigureJSON(managerConfig)) {
        CMN_LOG_INIT_ERROR << "Configure: failed to configure component-manager, check cisstLog for error messages" << std::endl;
        return -1;
    }

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    if (hasQt) {
        sensorWidget->show();
        application->exec();
    } else {
        do {
            std::cout << "Press 'q' to quit" << std::endl;
        } while (cmnGetChar() != 'q');
    }

    // stop all logs
    cmnLogger::Kill();

    // stop ROS node
    cisst_ral::shutdown();

    // kill all components and perform cleanup
    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    return 0;
}
