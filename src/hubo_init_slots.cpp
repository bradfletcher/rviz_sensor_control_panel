/*
 * brad@udel.edu
 * Thanks to Georgia Tech for letting me use their rviz plugin as an  example. 
 */

#include "hubo_init.h"

//added
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rviz_sensor_control_panel/HokuyoCommand.h"

namespace rviz_sensor_control_panel_space
{

void RVizSensorControlPanelWidget::refreshState()
{
	
} 

void RVizSensorControlPanelWidget::hokuyoEditHandle()
{
	 int count = 0;
    //ipAddrA = ipAddrAEdit->text().toInt();
    std::cerr << "Hokuyo Scan: start at: " <<  txtMinTheta->text().toStdString() << "end at: " << txtMaxTheta->text().toStdString() << " at a rate of " << txtDPS->text().toStdString() << std::endl;
    //std::cerr << "text" ; 
    
    //added ros stuff
    count = txtMinTheta->text().toInt();
 
    ros::Rate loop_rate(10);  
	loop_rate.sleep();
	
    //std_msgs::String msg;
    //std::stringstream ss;
    //ss << "hello world " << count;
    
    rviz_sensor_control_panel::HokuyoCommand msg;
    msg.minTheta = txtMinTheta->text().toFloat();
    msg.maxTheta = txtMaxTheta->text().toFloat();
    msg.degreesPerSecond = txtDPS->text().toFloat();
    
    //msg.data = ss.str();
    //ROS_INFO("%s", msg.data.c_str());
    ros::spinOnce();
    hokuyo_pub.publish(msg);
    loop_rate.sleep();
    ++count;       
    //
}

} 
