/*
 * Brad Fletcher
 * brad@udel.edu
 * Thanks to Georgia Tech for allowing me to use their 
 * RViz panel code as a model for this.
 */

#ifndef HUBO_INIT_H
#define HUBO_INIT_H

#include <stdio.h>

#include <QApplication>
#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <ros/ros.h>
#include <QTableWidget>
#include <QPushButton>
#include <QLineEdit>
#include <QRadioButton>
#include <QSpinBox>
#include <QLabel>
#include <QProcess>
#include <QGroupBox>
#include <QButtonGroup>
#include <QProcess>
#include <QString>
#include <QStringList>
#include <QTextStream>
#include <QClipboard>
#include <QPalette>
#include <QColor>
#include <QThread>
#include <QComboBox>
#include <QCheckBox>

#include <vector>

#include <rviz/panel.h>

//#include <hubo.h>
//#include <hubo-jointparams.h>

namespace rviz_sensor_control_panel_space
{

class SensorControlTab;

class RVizRefreshManager : public QThread
{
Q_OBJECT
public:
    SensorControlTab* parentWidget;
    bool alive;
    int waitTime;

protected:
    virtual void run();

protected slots:
    void getWaitTime(int t);

signals:
    void signalRefresh();

};

// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
class SensorControlTab: public QTabWidget
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  SensorControlTab( QWidget* parent = 0 );
  ~SensorControlTab();

  QString groupStyleSheet;
  
   ///////////////
  //ROS Related
  
  ros::NodeHandle n;
  ros::Publisher hokuyo_pub;
  ros::Publisher flea_pub;


  // Handler for the nested ach daemon process
  QProcess achChannelState;
  QProcess achChannelCmd;
  QProcess achdState;
  bool stateConnected;
  QProcess achdCmd;
  bool cmdConnected;
  
  // Update timer
  RVizRefreshManager* refreshManager;
  int getRefreshTime();

  // Ach Channels for sending and receiving data
  //  ach_channel_t stateChan;
  //  bool stateOpen;
  //  ach_channel_t cmdChan;
  //  bool cmdOpen;

  void initializeAchConnections();
  void initializeAchStructs();
  void commandSensor();
  void sendCommand();

  void normifyButton(int id); // Used if state is normal
  void purpifyButton(int id); // Used if joint is homing/failed to home
  void reddifyButton(int id); // Used if joint has an error
  void yellifyButton(int id); // Used if joint is inactive

  void setIPAddress(int a, int b, int c, int d);
  int getIPAddress(int index);

  // Structs for storing data to transmit
  //  struct hubo_state h_state;
  //  struct hubo_board_cmd h_cmd;
  //  struct hubo_param h_param;
  
  bool checkJointError(int id);
  bool checkJointHomed(int id);

  // Slots will be "connected" to signals in order to respond to user events
protected:
  int ipAddrA;
  int ipAddrB;
  int ipAddrC;
  int ipAddrD;

signals:
  void sendWaitTime(int t);

protected Q_SLOTS:

  // Send the command once the joint button is released
  void handleJointCmdButton(int id);
  void handleJointStateButton(int id);

  // Send sensor commands once the button is released
  void handleRHFT();
  void handleLHFT();
  void handleRFFT();
  void handleLFFT();
  void handleIMU();

  void handleHomeAll();
  void handleHomeBad();
  void handleInitSensors();
  
  void handleFTCopy();
  void handleIMUCopy();
  void handleJointCopy();
  
  //added 
  void hokuyoEditHandle();
  void sendToFleaHandle();

  // Update all state information
  void refreshState();

  // Deal with achd crashes/failures
  //  void achdSStartedSlot();
  //  void achdCStartedSlot();
  //  void achdSExitError(QProcess::ProcessError err);
  //  void achdSExitFinished(int num, QProcess::ExitStatus status);
  //  void achdCExitError(QProcess::ProcessError err);
  //  void achdCExitFinished(int num, QProcess::ExitStatus status);
  //  void achdConnectSlot();
  //  void achdDisconnectSlot();
  //  void achCreateCatch(QProcess::ProcessError err);
  
  //  void achCreateSHandle();
  //  void achCreateCHandle();
  
  void ipEditHandle(const QString &text);

private:

  std::vector<QString> ftName;
  
  //////////////
  void initializeKinFuStateTab();
  QPushButton* btnKinFuStart;
  QPushButton* btnKinFuStop;
  QWidget* kfStateTab;
  
  ///////////////
  void initializeHokuyoStateTab();
  QWidget* hokuyoStateTab;

    QGroupBox* hBox;
    //std::vector<QLineEdit*> ft_mx;
    //std::vector<QLineEdit*> ft_my;
    //std::vector<QLineEdit*> ft_fz;
    //QPushButton* copyFT;

    //QGroupBox* imuBox;
    QLineEdit* txtMinTheta;
    QLineEdit* txtMaxTheta;
    QLineEdit* txtDPS;
    QLineEdit* txtVoxelRes;
    QCheckBox* cbxVoxelize;

    //QLineEdit* w_x;
    //QLineEdit* w_y;
    //QLineEdit* w_z;
    QPushButton* btnScan;
    
  ///////////////

  void initializeFleaStateTab();
    QWidget* fleaStateTab;
    QPushButton* btnSendFlea;
    
    QButtonGroup* radioCmdButtonsFlea;
    QRadioButton* rbColor;
    QRadioButton* rbGrayscale;
    QComboBox *resComboBox;
    QComboBox *fpsComboBox;
    QCheckBox *cbxAutoExp;
    QCheckBox *cbxTrigger;
    QRadioButton* rbMsgColor;
    QRadioButton* rbMsgGrayscale;
    QComboBox *resMsgComboBox;   
    QComboBox *fpsMsgComboBox;
    
  ///////////////
  
  void initializeIMUStateTab();
    QWidget* imuStateTab;
    QPushButton* btnSendIMU;
    
  ///////////////
  
  void initializeDynamixelStateTab();
  QWidget* dynaStateTab;
  QPushButton* btnSendDyna;
  QPushButton* btnSendDyna2;
  QPushButton* btnSendDyna3;

};


class SensorControlPanel : public rviz::Panel
{
Q_OBJECT
public:
    SensorControlPanel(QWidget *parent = 0);

    // Now we declare overrides of rviz::Panel functions for saving and
    // loading data from the config file.  Here the data is the
    // topic name.
    virtual void load( const rviz::Config& config );
    virtual void save( rviz::Config config ) const;

private:

    SensorControlTab* content;

};

} // end namespace rviz_sensor_control_panel_space


#endif // TELEOP_PANEL_H
