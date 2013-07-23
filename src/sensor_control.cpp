/*
 * Brad Fletcher
 * brad@udel.edu
 * Thanks to Georgia Tech for allowing me to use their 
 * RViz panel code as a model for this.
 */


#include "sensor_control.h"
#include "FlowLayout.h"

//added
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rviz_sensor_control_panel/HokuyoCommand.h"
#include <sstream>

namespace rviz_sensor_control_panel_space
{

SensorControlPanel::SensorControlPanel(QWidget *parent)
    : rviz::Panel(parent)
{
    content = new SensorControlTab;
    QHBoxLayout* panelLayout = new QHBoxLayout;
    panelLayout->addWidget(content);
    setLayout(panelLayout);
}

SensorControlTab::~SensorControlTab()
{
    refreshManager->alive = false;
    refreshManager->quit();
    refreshManager->wait();
}

SensorControlTab::SensorControlTab(QWidget *parent)
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
                     
    //added ros stuff
    int argc = 0;
    char **argv;
    ros::init(argc, argv, "talker");
        hokuyo_pub = n.advertise<rviz_sensor_control_panel::HokuyoCommand>("hokuyo_control", 1000);
    //

    
    initializeHokuyoStateTab();
    std::cerr << "Hokuyo Tab Loaded" << std::endl;
    
    initializeFleaStateTab();
    std::cerr << "Flea Tab Loaded" << std::endl;
    
    initializeIMUStateTab();
    std::cerr << "IMU Tab Loaded" << std::endl;

    addTab(hokuyoStateTab, "Hokuyo ladar");
    addTab(fleaStateTab, "Flea3 camera");
    addTab(imuStateTab, "Microstrain IMU");

    
    refreshManager = new RVizRefreshManager;
    refreshManager->parentWidget = this;
    connect(this, SIGNAL(sendWaitTime(int)), refreshManager, SLOT(getWaitTime(int)));
    refreshManager->start();
}

void SensorControlTab::initializeIMUStateTab()
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

void SensorControlTab::initializeFleaStateTab()
{
	//Create a "Master" layout
	//Create a box layout.
    QVBoxLayout* fleaMasterLayout = new QVBoxLayout;
    
    ////////   Start of Capture Group Box   ///////
    
	//Create a grid layout.
    QGridLayout* fleaStateLayout = new QGridLayout;
    fleaStateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    
    //Create a group box for capture settings
    QGroupBox* capBox = new QGroupBox;
    capBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    capBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    capBox->setStyleSheet(groupStyleSheet);
    capBox->setTitle("Capture Settings");
    
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
    
    fleaStateLayout->addLayout(radioLayoutFlea, 0, 0, 2, 1, Qt::AlignHCenter | Qt::AlignVCenter);
    
     //adding combo box
     // Pyramid levels
     //0 - 1280 x 960
     //1 - 640 x 480
     //2 - 320 x 240
     //3 - 160 x 120
     //4 - 80 x 60
     //5 - 40 x 30
     
     QComboBox *resComboBox = new QComboBox;
     resComboBox->addItem(tr("1280 x 960"));
     resComboBox->addItem(tr("640 x 480"));
     resComboBox->addItem(tr("320 x 240"));
     resComboBox->addItem(tr("160 x 120"));
     resComboBox->addItem(tr("80 x 60"));
     resComboBox->addItem(tr("40 x 30"));
     
     fleaStateLayout->addWidget(resComboBox, 0, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
     
     //res label
     
     QLabel* lblRes = new QLabel;
     lblRes->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
     lblRes->setText("resolution");
     lblRes->setToolTip("resolution to send camera frames at");
     fleaStateLayout->addWidget(lblRes, 0, 2, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
              
     //fps
     QComboBox *fpsComboBox = new QComboBox;
     fpsComboBox->addItem(tr("60"));
     fpsComboBox->addItem(tr("30"));
     fpsComboBox->addItem(tr("15"));
     fpsComboBox->addItem(tr("7.5"));
     fpsComboBox->addItem(tr("3.75"));
     fpsComboBox->addItem(tr("1.875"));
     
     fleaStateLayout->addWidget(fpsComboBox, 1, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
     
     //fps label
     
     QLabel* lblFPS = new QLabel;
     lblFPS->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
     lblFPS->setText("fps");
     lblFPS->setToolTip("fps to send camera frames at");
     fleaStateLayout->addWidget(lblFPS, 1, 2, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
     
     ///////  End of first Group Box for Capture   //////////
     
     /////// Group Box - Message Sending / Display  /////////
     
     //Create a grid layout.
    QGridLayout* fleaMessageStateLayout = new QGridLayout;
    fleaMessageStateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    
    //Create a group box for message / display settings
    QGroupBox* msgBox = new QGroupBox;
    msgBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    msgBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    msgBox->setStyleSheet(groupStyleSheet);
    msgBox->setTitle("Message Send / Display Settings");
    
     // Set up the radio btns
    FlowLayout* radioMsgLayoutFlea = new FlowLayout;
    QButtonGroup* radioMsgCmdButtonsFlea = new QButtonGroup(this);

    radioMsgCmdButtonsFlea->setExclusive(true);

    QRadioButton* rbMsgColor = new QRadioButton;
    rbMsgColor->setText("Color");
    rbMsgColor->setToolTip("Get color images");
    rbMsgColor->setChecked(true);
    radioMsgCmdButtonsFlea->addButton(rbMsgColor);
    radioMsgLayoutFlea->addWidget(rbMsgColor);

    QRadioButton* rbMsgGrayscale = new QRadioButton;
    rbMsgGrayscale->setText("Grayscale");
    rbMsgGrayscale->setToolTip("Get grayscale images");
    radioMsgCmdButtonsFlea->addButton(rbMsgGrayscale);
    radioMsgLayoutFlea->addWidget(rbMsgGrayscale);
    
    fleaMessageStateLayout->addLayout(radioMsgLayoutFlea, 0, 0, 2, 1, Qt::AlignHCenter | Qt::AlignVCenter);
    
     //adding combo box
     // Pyramid levels
     //0 - 1280 x 960
     //1 - 640 x 480
     //2 - 320 x 240
     //3 - 160 x 120
     //4 - 80 x 60
     //5 - 40 x 30
     
     QComboBox *resMsgComboBox = new QComboBox;
     resMsgComboBox->addItem(tr("1280 x 960"));
     resMsgComboBox->addItem(tr("640 x 480"));
     resMsgComboBox->addItem(tr("320 x 240"));
     resMsgComboBox->addItem(tr("160 x 120"));
     resMsgComboBox->addItem(tr("80 x 60"));
     resMsgComboBox->addItem(tr("40 x 30"));
     
     fleaMessageStateLayout->addWidget(resMsgComboBox, 0, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
     
     //res label
     
     QLabel* lblMsgRes = new QLabel;
     lblMsgRes->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
     lblMsgRes->setText("resolution");
     lblMsgRes->setToolTip("resolution to send camera frames at");
     fleaMessageStateLayout->addWidget(lblMsgRes, 0, 2, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
     
     /*
     //auto exp
     QCheckBox *cbxMsgAutoExp = new QCheckBox("Auto Exposure", this);
     fleaMessageStateLayout->addWidget(cbxMsgAutoExp, 0, 3, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
     
     //trigger
     QCheckBox *cbxMsgTrigger = new QCheckBox("Trigger", this);
     fleaMessageStateLayout->addWidget(cbxMsgTrigger, 1, 3, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter); */
         
     //fps
     QComboBox *fpsMsgComboBox = new QComboBox;
     fpsMsgComboBox->addItem(tr("60"));
     fpsMsgComboBox->addItem(tr("30"));
     fpsMsgComboBox->addItem(tr("15"));
     fpsMsgComboBox->addItem(tr("7.5"));
     fpsMsgComboBox->addItem(tr("3.75"));
     fpsMsgComboBox->addItem(tr("1.875"));
     
     fleaMessageStateLayout->addWidget(fpsMsgComboBox, 1, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
     
     //fps label
     
     QLabel* lblMsgFPS = new QLabel;
     lblMsgFPS->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
     lblMsgFPS->setText("fps");
     lblMsgFPS->setToolTip("fps to send camera frames at");
     fleaMessageStateLayout->addWidget(lblMsgFPS, 1, 2, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
     
     /////////////// end of Message Send Display Box ///////////////////////
     
      ////////   Start of Common Group Box   ///////
    
	//Create a grid layout.
    QGridLayout* fleaCommonStateLayout = new QGridLayout;
    fleaCommonStateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    
    //Create a group box for capture settings
    QGroupBox* commonBox = new QGroupBox;
    commonBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    commonBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    commonBox->setStyleSheet(groupStyleSheet);
    commonBox->setTitle("Common Settings");
    
      //auto exp
     QCheckBox *cbxAutoExp = new QCheckBox("Auto Exposure", this);
     fleaCommonStateLayout->addWidget(cbxAutoExp, 0, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
     
     //trigger
     QCheckBox *cbxTrigger = new QCheckBox("Trigger", this);
     fleaCommonStateLayout->addWidget(cbxTrigger, 0, 2, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
    
    ////////////////////   end of Group Box for common buttons   ////////////////////////
    
    //Send button
    btnSendFlea = new QPushButton;
    btnSendFlea->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    btnSendFlea->setText("Send to Flea");
    btnSendFlea->setToolTip("Sends the command.");
    //fleaCommonStateLayout->addWidget(btnSendFlea, 1, 0, 1, 2, Qt::AlignHCenter | Qt::AlignVCenter);
	
    //Create the tab
    fleaStateTab = new QWidget;
    //Set the cap Box's layout
    capBox->setLayout(fleaStateLayout);
    msgBox->setLayout(fleaMessageStateLayout);
    commonBox->setLayout(fleaCommonStateLayout);
    //Add group boxes to the master layout
    fleaMasterLayout->addWidget(capBox, Qt::AlignHCenter | Qt::AlignTop);
    fleaMasterLayout->addWidget(msgBox, Qt::AlignHCenter | Qt::AlignTop);
    fleaMasterLayout->addWidget(commonBox, Qt::AlignHCenter | Qt::AlignTop);
    fleaMasterLayout->addWidget(btnSendFlea, Qt::AlignHCenter | Qt::AlignTop);
    fleaStateTab->setLayout(fleaMasterLayout);
}

void SensorControlTab::initializeHokuyoStateTab()
{
  //Create a box layout.

  QVBoxLayout* hokuyoStateLayout = new QVBoxLayout;
  
  //Create a group box for "single sweep" action

  hBox = new QGroupBox;
  hBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  hBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  hBox->setStyleSheet(groupStyleSheet);
  hBox->setTitle("Single Sweep");
  
  //Create a grid layout

  QGridLayout* hStateLayout = new QGridLayout;
  hStateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
  
  // Min theta label

  QLabel* minTheta = new QLabel;
  minTheta->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  minTheta->setText("Min Theta (degs)");
  minTheta->setToolTip("Tilt angle to start sweep at (negative is down)");
  hStateLayout->addWidget(minTheta, 0, 0, 1, 1, Qt::AlignLeft | Qt::AlignBottom );
  
  // Min theta text box

  txtMinTheta = new QLineEdit;
  txtMinTheta->setMaxLength(6);
  txtMinTheta->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  txtMinTheta->setReadOnly(false);
  txtMinTheta->setText("-10.0");
  txtMinTheta->setAlignment(Qt::AlignRight);
  hStateLayout->addWidget(txtMinTheta, 1, 0, 1, 1, Qt::AlignCenter);

  // Max theta label

  QLabel* lblMaxTheta = new QLabel;
  lblMaxTheta->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  lblMaxTheta->setText("Max Theta (degs)");
  lblMaxTheta->setToolTip("Tilt angle to finish sweep at");
  hStateLayout->addWidget(lblMaxTheta, 0, 1, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

  // Max theta text box

  txtMaxTheta = new QLineEdit;
  txtMaxTheta->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  txtMaxTheta->setMaxLength(6);
  txtMaxTheta->setReadOnly(false);
  txtMaxTheta->setText("10.0");
  txtMaxTheta->setAlignment(Qt::AlignRight);
  hStateLayout->addWidget(txtMaxTheta, 1, 1, 1, 1, Qt::AlignCenter);

  // Speed label

  QLabel* lblDPS = new QLabel;
  lblDPS->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  lblDPS->setText("Speed (degs/s)");
  lblDPS->setToolTip("Tilt speed during sweep");
  hStateLayout->addWidget(lblDPS, 0, 2, 1, 1, Qt::AlignLeft | Qt::AlignBottom );
  
  // Speed text box

  txtDPS = new QLineEdit;
  txtDPS->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  txtDPS->setMaxLength(6);
  txtDPS->setReadOnly(false);
  txtDPS->setText("5.0");
  txtDPS->setAlignment(Qt::AlignRight);
  hStateLayout->addWidget(txtDPS, 1, 2, 1, 1, Qt::AlignCenter);

  // button

  btnScan = new QPushButton;
  btnScan->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
  btnScan->setText("Start Sweep");
  btnScan->setToolTip("Execute single sweep now");
  hStateLayout->addWidget(btnScan, 5, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
  
  //Will need to connect this later

  connect(btnScan, SIGNAL(clicked()), this, SLOT(hokuyoEditHandle()));
  
  hBox->setLayout(hStateLayout);
  hokuyoStateLayout->addWidget(hBox, Qt::AlignHCenter | Qt::AlignTop);
  
  hokuyoStateTab = new QWidget;
  hokuyoStateTab->setLayout(hokuyoStateLayout);
}

void RVizRefreshManager::run()
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

void RVizRefreshManager::getWaitTime(int t)
{
    waitTime = t;
}


void SensorControlPanel::save(rviz::Config config) const
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

void SensorControlPanel::load(const rviz::Config &config)
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


} // rviz_sensor_control_panel_space




#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_sensor_control_panel_space::SensorControlPanel,rviz::Panel )
PLUGINLIB_EXPORT_CLASS( rviz_sensor_control_panel_space::SensorControlTab, QTabWidget )
PLUGINLIB_EXPORT_CLASS( rviz_sensor_control_panel_space::RVizRefreshManager, QThread )
