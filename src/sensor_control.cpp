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

    addTab(hokuyoStateTab, "Hokuyo Control");
    addTab(fleaStateTab, "Flea3 Control");
    addTab(imuStateTab, "IMU Control");

    
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

void SensorControlTab::initializeHokuyoStateTab()
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
