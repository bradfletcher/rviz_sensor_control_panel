/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author: Michael X. Grey <mxgrey@gatech.edu>
 * Date: May 08, 2013
 *
 * Humanoid Robotics Lab      Georgia Institute of Technology
 * Director: Mike Stilman     http://www.golems.org
 *
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */


#include "hubo_init.h"
#include "FlowLayout.h"

namespace hubo_init_space
{



HuboInitPanel::HuboInitPanel(QWidget *parent)
    : rviz::Panel(parent)
{
    content = new HuboInitWidget;
    QHBoxLayout* panelLayout = new QHBoxLayout;
    panelLayout->addWidget(content);
    setLayout(panelLayout);
}

HuboInitWidget::~HuboInitWidget()
{
    refreshManager->alive = false;
    refreshManager->quit();
    refreshManager->wait();
}

HuboInitWidget::HuboInitWidget(QWidget *parent)
    : QTabWidget(parent)
{

    groupStyleSheet = "QGroupBox {"
                      "border: 1px solid gray;"
                      "border-radius: 9px;"
                      "margin-top: 0.5em;"
                      "}"
                      "QGroupBox::title {"
                      "subcontrol-origin: margin;"
                      "left: 10px;"
                      "padding: 0 3px 0 3px;"
                      "}";

    //initializeAchStructs();

    //initializeCommandTab();
    //std::cerr << "Command Tab loaded" << std::endl;
    
    //initializeJointStateTab();
    //std::cerr << "Joint State Tab loaded" << std::endl;

    //initializeSensorCmdTab();
    //std::cerr << "Sensor Command Tab loaded" << std::endl;

    //initializeSensorStateTab();
    //std::cerr << "Sensor State Tab loaded" << std::endl;
    
    initializeHokuyoStateTab();
    std::cerr << "Hokuyo Tab Loaded" << std::endl;
    
    initializeFleaStateTab();
    std::cerr << "Flea Tab Loaded" << std::endl;
    
    initializeIMUStateTab();
    std::cerr << "IMU Tab Loadedaaa" << std::endl;

    //addTab(commandTab, "Joint Command");
    //addTab(jointStateTab, "Joint State");
    
    //addTab(sensorCmdTab, "Sensor Command");
    //addTab(sensorStateTab, "Sensor State");
    addTab(hokuyoStateTab, "Hokuyo Control");
    addTab(fleaStateTab, "Flea3 Control");
    addTab(imuStateTab, "IMU Control");


    //initializeAchConnections();
    
    refreshManager = new HuboRefreshManager;
    refreshManager->parentWidget = this;
    connect(this, SIGNAL(sendWaitTime(int)), refreshManager, SLOT(getWaitTime(int)));
    refreshManager->start();
}

void HuboInitWidget::initializeIMUStateTab()
{
    //Create a grid layout.
    QGridLayout* imuStateLayout = new QGridLayout;
    imuStateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
	
	btnSendIMU = new QPushButton;
    btnSendIMU->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    btnSendIMU->setText("Send to IMU");
    btnSendIMU->setToolTip("Sends the command.");
    imuStateLayout->addWidget(btnSendIMU, 0, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
	
    imuStateTab = new QWidget;
    imuStateTab->setLayout(imuStateLayout);
}

void HuboInitWidget::initializeFleaStateTab()
{
	//Create a grid layout.
    QGridLayout* fleaStateLayout = new QGridLayout;
    fleaStateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    
     // Set up the radio btns
    FlowLayout* radioLayoutFlea = new FlowLayout;
    radioCmdButtonsFlea = new QButtonGroup(this);

    radioCmdButtonsFlea->setExclusive(true);

    rbColor = new QRadioButton;
    rbColor->setText("Color");
    rbColor->setToolTip("Get color images");
    rbColor->setChecked(true);
    radioCmdButtonsFlea->addButton(rbColor);
    radioLayoutFlea->addWidget(rbColor);

    rbGrayscale = new QRadioButton;
    rbGrayscale->setText("Grayscale");
    rbGrayscale->setToolTip("Get grayscale images");
    radioCmdButtonsFlea->addButton(rbGrayscale);
    radioLayoutFlea->addWidget(rbGrayscale);
    
    fleaStateLayout->addLayout(radioLayoutFlea, 0, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
    
    /*QVBoxLayout* boxLayout = new QVBoxLayout;
    boxLayout->addLayout(radioLayoutFlea);
    boxLayout->addSpacing(15);
    boxLayout->addLayout(jointCmdLayout);*/
    
    //Send button
    btnSendFlea = new QPushButton;
    btnSendFlea->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    btnSendFlea->setText("Send to Flea");
    btnSendFlea->setToolTip("Sends the command.");
    fleaStateLayout->addWidget(btnSendFlea, 0, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
	
    fleaStateTab = new QWidget;
    fleaStateTab->setLayout(fleaStateLayout);
}

void HuboInitWidget::initializeHokuyoStateTab()
{
	//Create a box layout.
	QVBoxLayout* hokuyoStateLayout = new QVBoxLayout;
	
	//Create a group box.
	hBox = new QGroupBox;
    hBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    hBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    hBox->setStyleSheet(groupStyleSheet);
    hBox->setTitle("Hokuyo Control");

    //Create a grid layout.
    QGridLayout* hStateLayout = new QGridLayout;
    hStateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);

    QLabel* minTheta = new QLabel;
    minTheta->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    minTheta->setText("Starting Theta (Degrees)");
    minTheta->setToolTip("Angle to start scan at.");
    hStateLayout->addWidget(minTheta, 0, 0, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    txtMinTheta = new QLineEdit;
    txtMinTheta->setMaxLength(6);
    txtMinTheta->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    txtMinTheta->setReadOnly(false);
    hStateLayout->addWidget(txtMinTheta, 1, 0, 1, 1, Qt::AlignCenter);
    
    QLabel* lblMaxTheta = new QLabel;
    lblMaxTheta->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    lblMaxTheta->setText("Max Theta (Degrees)");
    lblMaxTheta->setToolTip("Angle Max");
    hStateLayout->addWidget(lblMaxTheta, 0, 1, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    txtMaxTheta = new QLineEdit;
    txtMaxTheta->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    txtMaxTheta->setMaxLength(6);
    txtMaxTheta->setReadOnly(false);
    hStateLayout->addWidget(txtMaxTheta, 1, 1, 1, 1, Qt::AlignCenter);

    QLabel* lblDPS = new QLabel;
    lblDPS->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    lblDPS->setText("Degrees Per Second");
    lblDPS->setToolTip("Scan Rate");
    hStateLayout->addWidget(lblDPS, 0, 2, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    txtDPS = new QLineEdit;
    txtDPS->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    txtDPS->setMaxLength(6);
    txtDPS->setReadOnly(false);
    hStateLayout->addWidget(txtDPS, 1, 2, 1, 1, Qt::AlignCenter);
    
    btnScan = new QPushButton;
    btnScan->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    btnScan->setText("Start Scan");
    btnScan->setToolTip("Starts a scan of the Hokuyo.");
    hStateLayout->addWidget(btnScan, 5, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
    
    //Will need to connect this later
    connect(btnScan, SIGNAL(clicked()), this, SLOT(hokuyoEditHandle()));

    hBox->setLayout(hStateLayout);
    hokuyoStateLayout->addWidget(hBox, Qt::AlignHCenter | Qt::AlignTop);

    hokuyoStateTab = new QWidget;
    hokuyoStateTab->setLayout(hokuyoStateLayout);
}

void HuboRefreshManager::run()
{
    alive = true;
    waitTime = 1000;
    connect(this, SIGNAL(signalRefresh()), parentWidget, SLOT(refreshState()));
    while(alive)
    {
        emit signalRefresh();
        this->msleep(waitTime);
    }
    emit finished();
}

void HuboRefreshManager::getWaitTime(int t)
{
    waitTime = t;
}
/*

void HuboInitWidget::achCreateCatch(QProcess::ProcessError err)
{
    ROS_INFO("Creating Ach Channel Failed: Error Code %d", (int)err);
}

*/


/*void HuboInitWidget::initializeCommandTab()
{
    // Set up the networking box
    QVBoxLayout* achdLayout = new QVBoxLayout;

    achdConnect = new QPushButton;
    achdConnect->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    achdConnect->setText("Connect");
    achdConnect->setToolTip("Connect to Hubo's on board computer");
    achdLayout->addWidget(achdConnect, 0, Qt::AlignCenter);
    connect(achdConnect, SIGNAL(clicked()), this, SLOT(achdConnectSlot()));
    
    achdDisconnect = new QPushButton;
    achdDisconnect->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    achdDisconnect->setText("Disconnect");
    achdDisconnect->setToolTip("Disconnect from Hubo's on board computer");
    achdLayout->addWidget(achdDisconnect, 0, Qt::AlignCenter);
    connect(achdDisconnect, SIGNAL(clicked()), this, SLOT(achdDisconnectSlot()));

    QHBoxLayout* statusLayout = new QHBoxLayout;
    QLabel* staticLabel = new QLabel;
    staticLabel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    staticLabel->setText("Status: ");
    statusLayout->addWidget(staticLabel);
    statusLabel = new QLabel;
    statusLabel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    statusLabel->setText("Not Connected");
    statusLayout->addWidget(statusLabel);
    achdLayout->addLayout(statusLayout);

    QHBoxLayout* networkLayout = new QHBoxLayout;
    networkLayout->addLayout(achdLayout);

    QHBoxLayout* ipLayout = new QHBoxLayout;
    ipLayout->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    ipAddrAEdit = new QLineEdit;
    ipAddrAEdit->setMaxLength(3);
    ipAddrAEdit->setMaximumWidth(50);
    ipAddrAEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    ipAddrAEdit->setToolTip("IP Address for Hubo's on board computer");
    ipLayout->addWidget(ipAddrAEdit);
    connect(ipAddrAEdit, SIGNAL(textEdited(QString)), this, SLOT(ipEditHandle(QString)));
    QLabel* dot1 = new QLabel;
    dot1->setText(".");
    ipLayout->addWidget(dot1);
    ipAddrBEdit = new QLineEdit;
    ipAddrBEdit->setMaxLength(3);
    ipAddrBEdit->setMaximumWidth(50);
    ipAddrBEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    ipAddrBEdit->setToolTip("IP Address for Hubo's on board computer");
    ipLayout->addWidget(ipAddrBEdit);
    connect(ipAddrBEdit, SIGNAL(textEdited(QString)), this, SLOT(ipEditHandle(QString)));
    QLabel* dot2 = new QLabel;
    dot2->setText(".");
    ipLayout->addWidget(dot2);
    ipAddrCEdit = new QLineEdit;
    ipAddrCEdit->setMaxLength(3);
    ipAddrCEdit->setMaximumWidth(50);
    ipAddrCEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    ipAddrCEdit->setToolTip("IP Address for Hubo's on board computer");
    ipLayout->addWidget(ipAddrCEdit);
    connect(ipAddrCEdit, SIGNAL(textEdited(QString)), this, SLOT(ipEditHandle(QString)));
    QLabel* dot3 = new QLabel;
    dot3->setText(".");
    ipLayout->addWidget(dot3);
    ipAddrDEdit = new QLineEdit;
    ipAddrDEdit->setMaxLength(3);
    ipAddrDEdit->setMaximumWidth(50);
    ipAddrDEdit->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    ipAddrDEdit->setToolTip("IP Address for Hubo's on board computer");
    ipLayout->addWidget(ipAddrDEdit);
    connect(ipAddrDEdit, SIGNAL(textEdited(QString)), this, SLOT(ipEditHandle(QString)));

    QLabel* ipTitle = new QLabel;
    ipTitle->setText("IP Address");
    ipTitle->setToolTip("IP Address for Hubo's on board computer");

    QVBoxLayout* ipUpperLayout = new QVBoxLayout;
    ipUpperLayout->addWidget(ipTitle, 0, Qt::AlignLeft | Qt::AlignBottom);
    ipUpperLayout->addLayout(ipLayout);

    networkLayout->addLayout(ipUpperLayout);



    QGroupBox* networkBox = new QGroupBox;
    networkBox->setStyleSheet(groupStyleSheet);
    networkBox->setTitle("Ach Networking");
    networkBox->setLayout(networkLayout);

    setIPAddress(192, 168, 1, 0);

    ////////////


    // Set up the global command box
    homeAll = new QPushButton;
    homeAll->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    homeAll->setText("Home All");
    homeAll->setToolTip("Execute homing command on all joints at once");
    connect(homeAll, SIGNAL(released()), this, SLOT(handleHomeAll()));

    homeBad = new QPushButton;
    homeBad->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    homeBad->setText("Rehome");
    homeBad->setToolTip("Execute homing command only on joints that failed to home");
    connect(homeBad, SIGNAL(released()), this, SLOT(handleHomeBad()));

    initSensors = new QPushButton;
    initSensors->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    initSensors->setText("Start Sensors");
    initSensors->setToolTip("Activate all the sensors and initialize them to zero");
    connect(initSensors, SIGNAL(released()), this, SLOT(handleInitSensors()));

    QLabel* refreshLabel = new QLabel;
    refreshLabel->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    refreshLabel->setText("Refresh Rate:");
    refreshLabel->setToolTip("Rate (Hz) at which state information is refreshed");
    refreshRate = new QDoubleSpinBox;
    refreshRate->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    refreshRate->setToolTip("Time (s) between display refreshes");
    refreshRate->setValue(0.1);
    refreshRate->setMaximum(10);
    refreshRate->setMinimum(0.01);
    refreshRate->setSingleStep(0.1);
    QVBoxLayout* refreshLayout = new QVBoxLayout;
    refreshLayout->addWidget(refreshLabel);
    refreshLayout->addWidget(refreshRate);

    QHBoxLayout* globalCmdLayout = new QHBoxLayout;
    globalCmdLayout->addWidget(homeAll);
    globalCmdLayout->addWidget(homeBad);
    globalCmdLayout->addWidget(initSensors);
    globalCmdLayout->addLayout(refreshLayout);
    QGroupBox* globalCmdBox = new QGroupBox;
    globalCmdBox->setStyleSheet(groupStyleSheet);
    globalCmdBox->setTitle("Initialization Commands");
    globalCmdBox->setLayout(globalCmdLayout);
    /////////////


    // Set up the joint command box
    FlowLayout* radioLayout = new FlowLayout;
    radioCmdButtons = new QButtonGroup(this);

    radioCmdButtons->setExclusive(true);

    home = new QRadioButton;
    home->setText("Home");
    home->setToolTip("Home a specific joint");
    home->setChecked(true);
    radioCmdButtons->addButton(home);
    radioLayout->addWidget(home);

    reset = new QRadioButton;
    reset->setText("Reset");
    reset->setToolTip("Reset encoder and clear error flags");
    radioCmdButtons->addButton(reset);
    radioLayout->addWidget(reset);

    ctrlOn = new QRadioButton;
    ctrlOn->setText("Ctrl On");
    ctrlOn->setToolTip("Turn the motor control on for this joint's board");
    radioCmdButtons->addButton(ctrlOn);
    radioLayout->addWidget(ctrlOn);

    ctrlOff = new QRadioButton;
    ctrlOff->setText("Ctrl Off");
    ctrlOff->setToolTip("Turn off the motor control for this joint's board");
    radioCmdButtons->addButton(ctrlOff);
    radioLayout->addWidget(ctrlOff);

    fetOn = new QRadioButton;
    fetOn->setText("FET On");
    fetOn->setToolTip("Allow voltage through the motor");
    radioCmdButtons->addButton(fetOn);
    radioLayout->addWidget(fetOn);

    fetOff = new QRadioButton;
    fetOff->setText("FET Off");
    fetOff->setToolTip("Disable voltage through the motor");
    radioCmdButtons->addButton(fetOff);
    radioLayout->addWidget(fetOff);

    beep = new QRadioButton;
    beep->setText("Beep");
    beep->setToolTip("Instruct the board to beep");
    radioCmdButtons->addButton(beep);
    radioLayout->addWidget(beep);

    initJoint = new QRadioButton;
    initJoint->setText("Initialize");
    initJoint->setToolTip("Load default settings on board\n"
                          "WARNING: This will overwrite your current settings\n"
                          "(Currently disabled)");
    initJoint->setCheckable(false);
    initJoint->setDisabled(true);
    radioCmdButtons->addButton(initJoint);
    radioLayout->addWidget(initJoint);

    jointCmdButtons.resize(HUBO_JOINT_COUNT);
    jointCmdGroup = new QButtonGroup(this);
    connect(jointCmdGroup, SIGNAL(buttonClicked(int)), this, SLOT(handleJointCmdButton(int)));
    FlowLayout* jointCmdLayout = new FlowLayout;
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        QPushButton* tempPushButton = new QPushButton;
        tempPushButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        tempPushButton->setText(QString::fromLocal8Bit(h_param.joint[i].name));
        tempPushButton->setToolTip("Normal");

        jointCmdGroup->addButton(tempPushButton, i);
        jointCmdLayout->addWidget(tempPushButton);

        jointCmdButtons[i] = tempPushButton;
    }

    QVBoxLayout* cmdLayout = new QVBoxLayout;
    cmdLayout->addLayout(radioLayout);
    cmdLayout->addSpacing(15);
    cmdLayout->addLayout(jointCmdLayout);
    QGroupBox* jointCmdBox = new QGroupBox;
    jointCmdBox->setStyleSheet(groupStyleSheet);
    jointCmdBox->setTitle("Joint Commands");
    jointCmdBox->setLayout(cmdLayout);
    ///////////////


    QVBoxLayout* masterCTLayout = new QVBoxLayout;
    masterCTLayout->addWidget(networkBox);
    masterCTLayout->addWidget(globalCmdBox);
    masterCTLayout->addWidget(jointCmdBox);

    commandTab = new QWidget;
    commandTab->setLayout(masterCTLayout);
}
*/


/*void HuboInitWidget::initializeJointStateTab()
{
    stateFlags = new QLineEdit;
    stateFlags->setReadOnly(true);

    jointStateButtons.resize(HUBO_JOINT_COUNT);
    jointStateGroup = new QButtonGroup(this);
    connect(jointStateGroup, SIGNAL(buttonClicked(int)), this, SLOT(handleJointStateButton(int)));
    FlowLayout* jointStateLayout = new FlowLayout;
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        QPushButton* tempPushButton = new QPushButton;
        tempPushButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        tempPushButton->setText(QString::fromLocal8Bit(h_param.joint[i].name));
        tempPushButton->setToolTip("Normal");

        jointStateGroup->addButton(tempPushButton, i);
        jointStateLayout->addWidget(tempPushButton);

        jointStateButtons[i] = tempPushButton;
    }
    
    copyJoints = new QPushButton;
    copyJoints->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    copyJoints->setText("Copy Data");
    copyJoints->setToolTip("Copy current joint state values to clipboard\n"
                           "(tab separated)");
    connect(copyJoints, SIGNAL(clicked()), this, SLOT(handleJointCopy()));

    radSelectGroup = new QButtonGroup;
    radSelectGroup->setExclusive(true);
    QHBoxLayout* radSelectLayout = new QHBoxLayout;
    radSelectLayout->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

    radSelect = new QRadioButton;
    radSelect->setChecked(true);
    radSelect->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    radSelect->setText("Radians");
    radSelect->setToolTip("Display joint angles in radians");
    radSelectGroup->addButton(radSelect);
    radSelectLayout->addWidget(radSelect);

    degSelect = new QRadioButton;
    degSelect->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    degSelect->setText("Degrees");
    degSelect->setToolTip("Display joint angles in degrees");
    radSelectGroup->addButton(degSelect);
    radSelectLayout->addWidget(degSelect);


    QVBoxLayout* masterJSTLayout = new QVBoxLayout;
    masterJSTLayout->addWidget(stateFlags);
    masterJSTLayout->addLayout(jointStateLayout);
    masterJSTLayout->addWidget(copyJoints, 0, Qt::AlignLeft | Qt::AlignVCenter);
    masterJSTLayout->addSpacing(15);
    masterJSTLayout->addLayout(radSelectLayout);

    jointStateTab = new QWidget;
    jointStateTab->setLayout(masterJSTLayout);
} */
/*

void HuboInitWidget::initializeSensorCmdTab()
{
    radioSensorButtons = new QButtonGroup;
    radioSensorButtons->setExclusive(true);

    QHBoxLayout* radioSensorLayout = new QHBoxLayout;
    nullSensor = new QRadioButton;
    nullSensor->setText("Null Sensor");
    nullSensor->setToolTip("Set sensor values to zero. Must run this before data can be received.");
    radioSensorButtons->addButton(nullSensor);
    radioSensorLayout->addWidget(nullSensor);
    nullSensor->setChecked(true);
    initSensor = new QRadioButton;
    initSensor->setText("Initialize");
    initSensor->setToolTip("Reset sensor board values to default settings.\n"
                           "WARNING: This overwrites the current board settings.\n"
                           "(Currently Disabled)");
    initSensor->setCheckable(false);
    initSensor->setDisabled(true);
    radioSensorButtons->addButton(initSensor);
    radioSensorLayout->addWidget(initSensor);
    // Note: Leaving out initSensor because I think it's a dangerous feature

    QHBoxLayout* sensorLayout1 = new QHBoxLayout;
    lhFTButton = new QPushButton;
    lhFTButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    lhFTButton->setText("Left Hand FT");
    lhFTButton->setToolTip("Left hand force torque sensor");
    sensorLayout1->addWidget(lhFTButton);
    connect( lhFTButton, SIGNAL(clicked()), this, SLOT(handleLHFT()) );
    rhFTButton = new QPushButton;
    rhFTButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    rhFTButton->setText("Right Hand FT");
    rhFTButton->setToolTip("Right hand force torque sensor");
    sensorLayout1->addWidget(rhFTButton);
    connect( rhFTButton, SIGNAL(clicked()), this, SLOT(handleRHFT()) );

    imuButton = new QPushButton;
    imuButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    imuButton->setText("IMU");
    imuButton->setToolTip("Inertial Measurement Unit (waist)");
    connect( imuButton, SIGNAL(clicked()), this, SLOT(handleIMU()) );

    QHBoxLayout* sensorLayout3 = new QHBoxLayout;
    lfFTButton = new QPushButton;
    lfFTButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    lfFTButton->setText("Left Foot FT");
    lfFTButton->setToolTip("Left foot force torque sensor");
    sensorLayout3->addWidget(lfFTButton);
    connect( lfFTButton, SIGNAL(clicked()), this, SLOT(handleLFFT()) );
    rfFTButton = new QPushButton;
    rfFTButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    rfFTButton->setText("Right Foot FT");
    rfFTButton->setToolTip("Right foot force torque sensor");
    sensorLayout3->addWidget(rfFTButton);
    connect( rfFTButton, SIGNAL(clicked()), this, SLOT(handleRFFT()) );


    QVBoxLayout* masterSCTLayout = new QVBoxLayout;
    masterSCTLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    masterSCTLayout->addLayout(radioSensorLayout);
    masterSCTLayout->addLayout(sensorLayout1);
    masterSCTLayout->addWidget(imuButton, 0, Qt::AlignCenter);
    masterSCTLayout->addLayout(sensorLayout3);

    sensorCmdTab = new QWidget;
    sensorCmdTab->setLayout(masterSCTLayout);
}
*/



void HuboInitWidget::initializeSensorStateTab()
{
    QVBoxLayout* sensorStateLayout = new QVBoxLayout;

//force torque grid
    QGroupBox* ftBox = new QGroupBox;
    ftBox->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    ftBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    ftBox->setTitle("Force-Torque Readings");
    ftBox->setStyleSheet(groupStyleSheet);

    QGridLayout* ftStateLayout = new QGridLayout;
    ftStateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);

    QLabel* rhftLab = new QLabel;
    rhftLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    rhftLab->setText("Right Hand");
    ftStateLayout->addWidget(rhftLab, 0, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* lhftLab = new QLabel;
    lhftLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    lhftLab->setText("Left Hand");
    ftStateLayout->addWidget(lhftLab, 0, 2, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* rfftLab = new QLabel;
    rfftLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    rfftLab->setText("Right Foot");
    ftStateLayout->addWidget(rfftLab, 0, 3, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* lfftLab = new QLabel;
    lfftLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    lfftLab->setText("Left Foot");
    ftStateLayout->addWidget(lfftLab, 0, 4, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* mxLab = new QLabel;
    mxLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    mxLab->setText("X Moment");
    mxLab->setToolTip("Moment about the X-Axis (N-m)");
    ftStateLayout->addWidget(mxLab, 1, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* myLab = new QLabel;
    myLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    myLab->setText("Y Moment");
    myLab->setToolTip("Moment about the Y-Axis (N-m)");
    ftStateLayout->addWidget(myLab, 2, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* fzLab = new QLabel;
    myLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    fzLab->setText("Z Force");
    fzLab->setToolTip("Force along the Z-Axis (N)");
    ftStateLayout->addWidget(fzLab, 3, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    ft_mx.resize(4);
    ft_my.resize(4);
    ft_fz.resize(4);
    for(int i=0; i<4; i++)
    {
        QSizePolicy mxPolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        
        QLineEdit* tempMxEdit = new QLineEdit;
        tempMxEdit->setSizePolicy(mxPolicy);
        tempMxEdit->setReadOnly(true);
        //tempMxEdit->setMinimumWidth(1);
        ftStateLayout->addWidget(tempMxEdit, 1, i+1, 1, 1);
        ft_mx[i] = tempMxEdit;

        QLineEdit* tempMyEdit = new QLineEdit;
        tempMyEdit->setSizePolicy(mxPolicy);
        tempMyEdit->setReadOnly(true);
        //tempMyEdit->setMinimumWidth(1);
        ftStateLayout->addWidget(tempMyEdit, 2, i+1, 1, 1);
        ft_my[i] = tempMyEdit;

        QLineEdit* tempFzEdit = new QLineEdit;
        tempFzEdit->setSizePolicy(mxPolicy);
        tempFzEdit->setReadOnly(true);
        //tempFzEdit->setMinimumWidth(1);
        ftStateLayout->addWidget(tempFzEdit, 3, i+1, 1, 1);
        ft_fz[i] = tempFzEdit;
    }

    copyFT = new QPushButton;
    copyFT->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    copyFT->setText("Copy Data");
    copyFT->setToolTip("Copy the Force-Torque data to clipboard (tab and newline separated)");
    ftStateLayout->addWidget(copyFT, 4, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
    connect(copyFT, SIGNAL(clicked()), this, SLOT(handleFTCopy()));
    
    ftBox->setLayout(ftStateLayout);
    sensorStateLayout->addWidget(ftBox, 0, Qt::AlignHCenter | Qt::AlignTop);


    imuBox = new QGroupBox;
    imuBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    imuBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    imuBox->setStyleSheet(groupStyleSheet);
    imuBox->setTitle("IMU Sensor Readings");

    QGridLayout* imuStateLayout = new QGridLayout;
    imuStateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);

    QLabel* tiltx = new QLabel;
    tiltx->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    tiltx->setText("Tilt X");
    tiltx->setToolTip("Angle of tilt about the X-Axis (deg)");
    imuStateLayout->addWidget(tiltx, 0, 0, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    a_x = new QLineEdit;
    a_x->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    a_x->setReadOnly(true);
    imuStateLayout->addWidget(a_x, 1, 0, 1, 1, Qt::AlignCenter);

    QLabel* tilty = new QLabel;
    tilty->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    tilty->setText("Tilt Y");
    tilty->setToolTip("Angle of tilt about the Y-Axis (deg)");
    imuStateLayout->addWidget(tilty, 0, 1, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    a_y = new QLineEdit;
    a_y->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    a_y->setReadOnly(true);
    imuStateLayout->addWidget(a_y, 1, 1, 1, 1, Qt::AlignCenter);

    QLabel* tiltz = new QLabel;
    tiltz->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    tiltz->setText("Tilt Z");
    tiltz->setToolTip("Angle of tilt about the Z-Axis (deg)");
    imuStateLayout->addWidget(tiltz, 0, 2, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    a_z = new QLineEdit;
    a_z->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    a_z->setReadOnly(true);
    imuStateLayout->addWidget(a_z, 1, 2, 1, 1, Qt::AlignCenter);


    QLabel* velx = new QLabel;
    velx->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    velx->setText("Angular Vel X");
    velx->setToolTip("Angular Velocity about the X-Axis");
    imuStateLayout->addWidget(velx, 3, 0, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    w_x = new QLineEdit;
    w_x->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    w_x->setReadOnly(true);
    imuStateLayout->addWidget(w_x, 4, 0, 1, 1, Qt::AlignCenter);

    QLabel* vely = new QLabel;
    vely->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    vely->setText("Angular Vel Y");
    vely->setToolTip("Angular Velocity about the Y-Axis");
    imuStateLayout->addWidget(vely, 3, 1, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    w_y = new QLineEdit;
    w_y->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    w_y->setReadOnly(true);
    imuStateLayout->addWidget(w_y, 4, 1, 1, 1, Qt::AlignCenter);

    QLabel* velz = new QLabel;
    velz->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    velz->setText("Angular Vel Z");
    velz->setToolTip("Angular Velocity about the Z-Axis");
    imuStateLayout->addWidget(velz, 3, 2, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    w_z = new QLineEdit;
    w_z->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    w_z->setReadOnly(true);
    imuStateLayout->addWidget(w_z, 4, 2, 1, 1);
    
    copyIMU = new QPushButton;
    copyIMU->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    copyIMU->setText("Copy Data");
    copyIMU->setToolTip("Copy the IMU data to clipboard (tab and newline separated)");
    imuStateLayout->addWidget(copyIMU, 5, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
    connect(copyIMU, SIGNAL(clicked()), this, SLOT(handleIMUCopy()));

    imuBox->setLayout(imuStateLayout);
    sensorStateLayout->addWidget(imuBox, Qt::AlignHCenter | Qt::AlignTop);

    sensorStateTab = new QWidget;
    sensorStateTab->setLayout(sensorStateLayout);
}

void HuboInitPanel::save(rviz::Config config) const
{
    /*rviz::Panel::save(config);
    config.mapSetValue("Class", getClassId());

    rviz::Config ip_config = config.mapMakeChild("HuboIP");

    QVariant a = QVariant(content->getIPAddress(0));
    QVariant b = QVariant(content->getIPAddress(1));
    QVariant c = QVariant(content->getIPAddress(2));
    QVariant d = QVariant(content->getIPAddress(3));

    ip_config.mapSetValue("ipAddrA", a);
    ip_config.mapSetValue("ipAddrB", b);
    ip_config.mapSetValue("ipAddrC", c);
    ip_config.mapSetValue("ipAddrD", d);*/

}

void HuboInitPanel::load(const rviz::Config &config)
{
    /*rviz::Panel::load(config);
    rviz::Config ip_config = config.mapGetChild("HuboIP");
    QVariant a, b, c, d;
    if( !ip_config.mapGetValue("ipAddrA", &a) || !ip_config.mapGetValue("ipAddrB", &b)
     || !ip_config.mapGetValue("ipAddrC", &c) || !ip_config.mapGetValue("ipAddrD", &d))
        ROS_INFO("Loading the IP Address Failed");
    else
        content->setIPAddress(a.toInt(), b.toInt(), c.toInt(), d.toInt()); */
}


} // End hubo_init_space




#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( hubo_init_space::HuboInitPanel,rviz::Panel )
PLUGINLIB_EXPORT_CLASS( hubo_init_space::HuboInitWidget, QTabWidget )
PLUGINLIB_EXPORT_CLASS( hubo_init_space::HuboRefreshManager, QThread )
