/*
 * Brad Fletcher
 * brad@udel.edu
 * Thanks to Georgia Tech for allowing me to use their 
 * RViz panel code as a model for this.
 */


#include "sensor_control.h"

//added
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rviz_sensor_control_panel/HokuyoCommand.h"
#include "rviz_sensor_control_panel/FleaCommand.h"
#include "rviz_sensor_control_panel/KinFuCommand.h"

namespace rviz_sensor_control_panel_space
{

void SensorControlTab::refreshState()
{
	
} 

void SensorControlTab::hokuyoEditHandle()
{
    std::cerr << "Hokuyo Scan: start at: " << txtMinTheta->text().toStdString() << "end at: " << txtMaxTheta->text().toStdString() << " at a rate of " << txtDPS->text().toStdString() << std::endl;
    
    //added ros stuff
 
    ros::Rate loop_rate(10);
    loop_rate.sleep();

    printf("hokuyo edit\n"); fflush(stdout);

    rviz_sensor_control_panel::HokuyoCommand msg;
    msg.minTheta = txtMinTheta->text().toFloat();
    msg.maxTheta = txtMaxTheta->text().toFloat();
    msg.degreesPerSecond = txtDPS->text().toFloat();
     if(cbxVoxelize->isChecked() == true)
    {
      msg.voxelize = 1;
    }
    else
    {
		msg.voxelize = 0;
	}
	msg.voxelResolution = txtVoxelRes->text().toFloat();
    hokuyo_pub.publish(msg);
    

    //ROS_INFO("%s", msg.data.c_str());
    ros::spinOnce();

    //    loop_rate.sleep();
}

void SensorControlTab::sendToFleaHandle()
{
    //std::cerr << "Hokuyo Scan: start at: " << txtMinTheta->text().toStdString() << "end at: " << txtMaxTheta->text().toStdString() << " at a rate of " << txtDPS->text().toStdString() << std::endl;
   
    ros::Rate loop_rate(10);
    loop_rate.sleep();

    printf("send to flea \n"); fflush(stdout);
    
   
    rviz_sensor_control_panel::FleaCommand msg;
    //load common info
    if(cbxAutoExp->isChecked() == true)
    {
      msg.autoExposure = 1;
    }
    else
    {
		msg.autoExposure = 0;
	}
	if(cbxTrigger->isChecked() == true)
    {
      msg.trigger = 1;
    }
    else
    {
		msg.trigger = 0;
	}
    
    // load capture settings
    if(rbColor->isChecked() == true)
    {
      msg.captureColorMode = 1;
    }
    else
    {
		msg.captureColorMode = 0;
	}
	msg.captureImageSize = resComboBox->currentIndex();
	msg.captureFPS = fpsComboBox->currentIndex();
	
	 // load msg send settings
	 
    if(rbMsgColor->isChecked() == true)
    {
      msg.sendColorMode = 1;
    }
    else
    {
		msg.sendColorMode = 0;
	}
	msg.sendImageSize = resMsgComboBox->currentIndex();
	msg.sendFPS = fpsMsgComboBox->currentIndex();
	
	
	// publish
		
    flea_pub.publish(msg);
    

    //ROS_INFO("%s", msg.data.c_str());
    ros::spinOnce();

    //    loop_rate.sleep();
}
void SensorControlTab::sendToKinFuHandle()
{
  ros::Rate loop_rate(10);
  loop_rate.sleep();

  printf("send to kinfu \n"); fflush(stdout);
    
  rviz_sensor_control_panel::KinFuCommand msg;
    	
}

} // End: namespace rviz_sensor_control_panel_space
