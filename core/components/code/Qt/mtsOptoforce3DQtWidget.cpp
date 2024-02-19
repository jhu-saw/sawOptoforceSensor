/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-07-21

  (C) Copyright 2014-2020 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

// system include
#include <iostream>

// Qt include
#include <QString>
#include <QLabel>
#include <QtGui>
#include <QMessageBox>
#include <QPushButton>

// cisst
#include <cisstVector/vctForceTorqueQtWidget.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstMultiTask/mtsIntervalStatisticsQtWidget.h>
#include <cisstMultiTask/mtsMessageQtWidget.h>

#include <sawOptoforceSensor/mtsOptoforce3DQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsOptoforce3DQtWidget, mtsComponent, std::string);

mtsOptoforce3DQtWidget::mtsOptoforce3DQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds)
{
    QMMessage = new mtsMessageQtWidget();

    // setup interface
    m_device_interface = AddInterfaceRequired("Device");
    if (m_device_interface) {
        QMMessage->SetInterfaceRequired(m_device_interface);
        m_device_interface->AddFunction("measured_cf", Device.measured_cf);
        m_device_interface->AddFunction("get_period_statistics", Device.get_period_statistics);
    }

    setupUi();
    startTimer(TimerPeriodInMilliseconds); // ms
}

void mtsOptoforce3DQtWidget::Configure(const std::string &filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsOptoforce3DQtWidget::Startup(void)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsOptoforce3DQtWidget::Startup" << std::endl;
    if (!parent()) {
        show();
    }
}

void mtsOptoforce3DQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsOptoforce3DQtWidget::Cleanup" << std::endl;
}

void mtsOptoforce3DQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsOptoforce3DQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsOptoforce3DQtWidget::setupUi(void)
{
    QVBoxLayout * mainLayout = new QVBoxLayout;

    // wrench
    QFTWidget = new vctForceTorqueQtWidget();
    mainLayout->addWidget(QFTWidget);

    // timing
    QMIntervalStatistics = new mtsIntervalStatisticsQtWidget();
    mainLayout->addWidget(QMIntervalStatistics);

    // messages
    QMMessage->setupUi();
    mainLayout->addWidget(QMMessage);

    mainLayout->addStretch();

    setLayout(mainLayout);
    setWindowTitle("sawForceDimensionSDK");
    resize(sizeHint());
}

void mtsOptoforce3DQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    mtsExecutionResult executionResult;
    executionResult = Device.measured_cf(m_measured_cf);
    if (executionResult) {
        QFTWidget->SetValue(m_measured_cf.F(), m_measured_cf.T(),
                            m_measured_cf.Timestamp());
    }

    Device.get_period_statistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);
}
