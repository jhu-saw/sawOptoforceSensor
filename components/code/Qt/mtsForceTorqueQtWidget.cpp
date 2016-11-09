/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2013-08-24

  (C) Copyright 2013 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <QHeaderView>
#include <QComboBox>
#include <QLineEdit>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <sawOptoforceSensor/mtsForceTorqueQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsForceTorqueQtWidget, mtsComponent, std::string);

mtsForceTorqueQtWidget::mtsForceTorqueQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    PlotIndex(0),
    TimerPeriodInMilliseconds(periodInSeconds) // Qt timers are in milliseconds
{
    // Setup CISST Interface
    mtsInterfaceRequired * interfaceRequired;
    interfaceRequired = AddInterfaceRequired("RequiresForceTorqueSensor");
    if(interfaceRequired) {
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
    QFont font;
    font.setBold(true);
    font.setPointSize(12);

    QTabWidget * tabWidget = new QTabWidget;

    //--- Tab 1
    QVBoxLayout * tab1Layout = new QVBoxLayout;
    QLabel * instructionsLabel = new QLabel("This widget displays the force and torques values sensed by the ATI NetFT Sensor.\nUnits - Force(N), Torque(N-mm) \nValue in the brackets of each header displays the max F/T(-value,+value)");

    // Spacers
    QSpacerItem * vSpacer = new QSpacerItem(40, 10, QSizePolicy::Expanding, QSizePolicy::Preferred);

    // Force/Torque display widgets and layouts
    QVBoxLayout * spinBoxLayout = new QVBoxLayout;
    QHBoxLayout * ftValuesLayout = new QHBoxLayout;
    QLabel * ftLabel = new QLabel("Values");
    ftValuesLayout->addWidget(ftLabel);

    QFTSensorValues = new vctQtWidgetDynamicVectorDoubleRead();
    QFTSensorValues->SetPrecision(4);

    ftValuesLayout->addWidget(QFTSensorValues);
    spinBoxLayout->addLayout(ftValuesLayout);  

    // Combo box to select the plot item
    QComboBox * QPlotSelectItem = new QComboBox;
    QPlotSelectItem->addItem("Fx");
    QPlotSelectItem->addItem("Fy");
    QPlotSelectItem->addItem("Fz");
    QPlotSelectItem->addItem("FNorm");
    QPlotSelectItem->addItem("Fxyz");
    QPlotSelectItem->addItem("Txyz");

    // Upper and lower limits of the plot
    QVBoxLayout * plotLabelLayout = new QVBoxLayout;
    UpperLimit = new QLabel("U");
    UpperLimit->setAlignment(Qt::AlignTop|Qt::AlignRight);
    LowerLimit = new QLabel("L");
    LowerLimit->setAlignment(Qt::AlignBottom | Qt::AlignRight);
    plotLabelLayout->addWidget(UpperLimit);
    plotLabelLayout->addWidget(LowerLimit);

    // Initilize ft sensor plot
    SetupSensorPlot();

    // Add combo box, upper/lower limits labels and plot to a single layout
    QHBoxLayout * sensorPlotLayout = new QHBoxLayout;
    sensorPlotLayout->addWidget(QPlotSelectItem);
    sensorPlotLayout->addLayout(plotLabelLayout);
    sensorPlotLayout->addWidget(QFTPlot);

    // Layout for Error Messages
    QHBoxLayout * errorLayout = new QHBoxLayout;
    QLabel *errorLabel = new QLabel("Info");

    errorLayout->addWidget(errorLabel);
    // Layout containing rebias button
    QHBoxLayout * buttonLayout = new QHBoxLayout;
    buttonLayout->addStretch();

    // Tab1 layout order
    tab1Layout->addWidget(instructionsLabel);
    tab1Layout->addLayout(spinBoxLayout);
    tab1Layout->addLayout(sensorPlotLayout);
    tab1Layout->addSpacerItem(vSpacer);
    tab1Layout->addLayout(errorLayout);
    tab1Layout->addLayout(buttonLayout);
    tab1Layout->addSpacerItem(vSpacer);

    QWidget * tab1 = new QWidget;
    tab1->setLayout(tab1Layout);

    //--- Tab 2
    QVBoxLayout * tab2Layout = new QVBoxLayout;
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    tab2Layout->addWidget(QMIntervalStatistics);
    tab2Layout->addStretch();

    QWidget * tab2 = new QWidget;
    tab2->setLayout(tab2Layout);

    // Setup tab widget
    tabWidget->addTab(tab1, "Sensor Stats");
    tabWidget->addTab(tab2, "Interval Stats");

    QHBoxLayout * mainLayout = new QHBoxLayout;
    mainLayout->addWidget(tabWidget);
    setLayout(mainLayout);

    setWindowTitle("ATI Force Sensor(N, N-mm)");
    resize(sizeHint());

    // setup Qt Connection
    connect(QPlotSelectItem, SIGNAL(currentIndexChanged(int)), this, SLOT(SlotPlotIndex(int)));

    QPlotSelectItem->setCurrentIndex(FNorm);
}

void mtsForceTorqueQtWidget::SetupSensorPlot()
{
    // Plot to show force/torque graph
    QFTPlot = new vctPlot2DOpenGLQtWidget();
    QFTPlot->SetBackgroundColor(vct3(1.0, 1.0, 1.0));
    QFTPlot->resize(QFTPlot->sizeHint());
    QFTPlot->setSizePolicy(QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));
    ForceScale = QFTPlot->AddScale("Force");
    TorqueScale = QFTPlot->AddScale("Torque");
    ForceSignal[0] = ForceScale->AddSignal("fx");
    ForceSignal[0]->SetColor(vctDouble3(1.0, 0.0, 0.0));
    ForceSignal[0]->SetVisible(false);
    ForceSignal[1] = ForceScale->AddSignal("fy");
    ForceSignal[1]->SetColor(vctDouble3(0.0, 1.0, 0.0));
    ForceSignal[1]->SetVisible(false);
    ForceSignal[2] = ForceScale->AddSignal("fz");
    ForceSignal[2]->SetColor(vctDouble3(0.0, 0.0, 1.0));
    ForceSignal[2]->SetVisible(false);
    TorqueSignal[0] = TorqueScale->AddSignal("tx");
    TorqueSignal[0]->SetColor(vctDouble3(1.0, 0.0, 0.0));
    TorqueSignal[0]->SetVisible(false);
    TorqueSignal[1] = TorqueScale->AddSignal("ty");
    TorqueSignal[1]->SetColor(vctDouble3(0.0, 1.0, 0.0));
    TorqueSignal[1]->SetVisible(false);
    TorqueSignal[2] = TorqueScale->AddSignal("tz");
    TorqueSignal[2]->SetColor(vctDouble3(0.0, 0.0, 1.0));
    TorqueSignal[2]->SetVisible(false);
    FNormSignal = ForceScale->AddSignal("fnorm");
    FNormSignal->SetColor(vctDouble3(0.0, 0.0, 0.0));
    FNormSignal->SetVisible(false);
}

void mtsForceTorqueQtWidget::timerEvent(QTimerEvent * event)
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }
    mtsExecutionResult executionResult;
    if (!executionResult) {
        CMN_LOG_CLASS_RUN_ERROR << "ForceSensor.GetFTData failed, \""
                                << executionResult << "\"" << std::endl;
    }

    // Uppdate the plot
    vctDoubleVec forceOnly(3, 0.0), torqueOnly(3, 0.0);
    ForceSensor.GetPeriodStatistics(IntervalStatistics);
    QMIntervalStatistics->SetValue(IntervalStatistics);

    // Update the lower/upper limits on the plot
    vct2 range;
    if(PlotIndex < Txyz) {
        range = ForceScale->GetViewingRangeY();
    } else if (PlotIndex == Txyz) {
        range = TorqueScale->GetViewingRangeY();
    }
    QString text;
    text.setNum(range[0], 'f', 2);
    LowerLimit->setText(text);
    text.setNum(range[1], 'f', 2);
    UpperLimit->setText(text);


    QFTPlot->updateGL();
}
