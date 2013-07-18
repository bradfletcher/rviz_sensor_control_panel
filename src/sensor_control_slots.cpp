/*
 * Brad Fletcher
 * brad@udel.edu
 * Thanks to Georgia Tech for allowing me to use their 
 * RViz panel code as a model for this.
 */


#include "sensor_control.h"

namespace rviz_sensor_control_panel_space
{

void SensorControlTab::refreshState()
{
	
} 

void SensorControlTab::hokuyoEditHandle()
{
    //ipAddrA = ipAddrAEdit->text().toInt();
    std::cerr << "Hokuyo Scan: start at: " <<  txtMinTheta->text().toStdString() << "end at: " << txtMaxTheta->text().toStdString() << " at a rate of " << txtDPS->text().toStdString() << std::endl;
    //std::cerr << "text" ; 
}

} // End: namespace rviz_sensor_control_panel_space
