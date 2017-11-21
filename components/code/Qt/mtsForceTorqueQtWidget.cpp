/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet, Dorothy Hu
  Created on: 2017-01-20

  (C) Copyright 2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/


// system include
#include <iostream>

// Qt includes
#include <QString>
#include <QtGui>
#include <QMessageBox>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <sawOptoforceSensor/mtsForceTorqueQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsForceTorqueQtWidget, mtsComponent, std::string);

mtsForceTorqueQtWidget::mtsForceTorqueQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds * 1000) // Qt timers are in milliseconds
{
    // Setup CISST Interface
    mtsInterfaceRequired * interfaceRequired;
    interfaceRequired = AddInterfaceRequired("ForceSensor");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetForceTorque", ForceSensor.GetForceTorque);
        interfaceRequired->AddFunction("GetPeriodStatistics", ForceSensor.GetPeriodStatistics);
   }

    setupUi();
    startTimer(TimerPeriodInMilliseconds);
}

void mtsForceTorqueQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsForceTorqueQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup" << std::endl;
    if (!parent()) {
        show();
    }
}

void mtsForceTorqueQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}

void mtsForceTorqueQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsForceTorqueQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsForceTorqueQtWidget::setupUi()
{
    QTabWidget * tabWidget = new QTabWidget;

    //--- Tab 1
    QVForceTorque = new vctForceTorqueQtWidget();
    tabWidget->addTab(QVForceTorque, "Force Torque");

    //--- Tab 2
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    tabWidget->addTab(QMIntervalStatistics, "Interval Stats");

    QHBoxLayout * mainLayout = new QHBoxLayout;
    mainLayout->addWidget(tabWidget);
    setLayout(mainLayout);

    setWindowTitle("sawOptoForce Sensor(N, N-mm)");
    resize(sizeHint());
}

void mtsForceTorqueQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    // force torque data
    mtsExecutionResult executionResult;
    executionResult = ForceSensor.GetForceTorque(ForceSensor.ForceTorque);
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "ForceSensor.GetForceTorque failed, \""
                                << executionResult << "\"" << std::endl;
    }
    QVForceTorque->SetValue(ForceSensor.ForceTorque.F(),
                            ForceSensor.ForceTorque.T(),
                            ForceSensor.ForceTorque.Timestamp());

    ForceSensor.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}
