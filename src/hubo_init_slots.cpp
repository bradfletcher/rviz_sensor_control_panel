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




namespace hubo_init_space
{

void HuboInitWidget::refreshState()
{
	/*
    size_t fs;
    ach_status_t r = ACH_OK;
    if(stateOpen)
        r = ach_get(&stateChan, &h_state, sizeof(h_state), &fs, NULL, ACH_O_LAST);
    if( r != ACH_OK && r != ACH_STALE_FRAMES && r != ACH_MISSED_FRAME )
        std::cerr << ach_result_to_string(r) << std::endl;


    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        if( h_state.joint[i].active == 0 )
            yellifyButton(i);
        else if( checkJointError(i) )
            reddifyButton(i);
        else if( !checkJointHomed(i) )
            purpifyButton(i);
        else
            normifyButton(i);
        
        if(currentIndex()==1)
        {
            QString jointText;
            QString jointVal;
            double pos = h_state.joint[i].pos;
            if( degSelect->isChecked() )
                jointVal.sprintf("%4.1f", pos*180.0/M_PI);
            else
                jointVal.sprintf("%2.3f", pos);

            QTextStream(&jointText) << QString::fromLocal8Bit(h_param.joint[i].name)
                                    << "\n" << jointVal;
            jointStateButtons[i]->setText(jointText);
        }
    }
    
    if(currentIndex()==3)
    {
        for(int i=0; i<4; i++)
        {
            ft_mx[i]->setText(QString::number(h_state.ft[i].m_x));
            ft_my[i]->setText(QString::number(h_state.ft[i].m_y));
            ft_fz[i]->setText(QString::number(h_state.ft[i].f_z));
        }
    
        a_x->setText(QString::number(h_state.imu[2].a_x));
        a_y->setText(QString::number(h_state.imu[2].a_y));
        a_z->setText(QString::number(h_state.imu[2].a_z));

        w_x->setText(QString::number(h_state.imu[2].w_x));
        w_y->setText(QString::number(h_state.imu[2].w_y));
        w_z->setText(QString::number(h_state.imu[2].w_z));
    }

    emit sendWaitTime(getRefreshTime()); */
} 
/*
void HuboInitWidget::normifyButton(int id)
{
    QColor color(230,230,230);
    QString style = "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #dadbde, stop: 1 ";

    if(currentIndex()==0)
        jointCmdButtons[id]->setStyleSheet(style + color.name() + ")");

    if(currentIndex()==1)
        jointStateButtons[id]->setStyleSheet(style + color.name() + ")");
    
    
    jointCmdButtons[id]->setToolTip("Normal");
    jointStateButtons[id]->setToolTip("Normal");
} */
/*
void HuboInitWidget::yellifyButton(int id)
{
    QColor color(250,250,210);
    QString style = "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #dadbde, stop: 1 ";

    if(currentIndex()==0)
        jointCmdButtons[id]->setStyleSheet(style + color.name() + ")");
    if(currentIndex()==1)
        jointStateButtons[id]->setStyleSheet(style + color.name() + ")");
    
    
    jointCmdButtons[id]->setToolTip("Inactive");
    jointStateButtons[id]->setToolTip("Inactive");
} */
/*
void HuboInitWidget::reddifyButton(int id)
{
    QColor color(200,0,0);
    QString style = "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #dadbde, stop: 1 ";

    if(currentIndex()==0)
        jointCmdButtons[id]->setStyleSheet(style + color.name() + ")");

    if(currentIndex()==1)
        jointStateButtons[id]->setStyleSheet(style + color.name() + ")");
    
    
    jointCmdButtons[id]->setToolTip("Error!");
    jointStateButtons[id]->setToolTip("Error!");
} */
/*
void HuboInitWidget::purpifyButton(int id)
{
    QColor color(200,0,200);
    QString style = "background-color: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #dadbde, stop: 1 ";

    if(currentIndex()==0)
        jointCmdButtons[id]->setStyleSheet(style + color.name() + ")");

    if(currentIndex()==1)
        jointStateButtons[id]->setStyleSheet(style + color.name() + ")");
    
    
    jointCmdButtons[id]->setToolTip("Not Homed!");
    jointStateButtons[id]->setToolTip("Not Homed!");
}
*/
/*
void HuboInitWidget::handleFTCopy()
{
    QString copyText;
    
    for(int i=0; i<4; i++)
    {
        if(i>0)
            QTextStream(&copyText) << "\t";
        QTextStream(&copyText) << QString::number(h_state.ft[i].m_x);
    }
    QTextStream(&copyText) << "\n";
    for(int i=0; i<4; i++)
    {
        if(i>0)
            QTextStream(&copyText) << "\t";
        QTextStream(&copyText) << QString::number(h_state.ft[i].m_y);
    }
    QTextStream(&copyText) << "\n";
    for(int i=0; i<4; i++)
    {
        if(i>0)
            QTextStream(&copyText) << "\t";
        QTextStream(&copyText) << QString::number(h_state.ft[i].f_z);
    }
    
    QClipboard* clipboard = QApplication::clipboard();
    clipboard->setText(copyText);
} */

/*
void HuboInitWidget::handleIMUCopy()
{
    QString copyText;

    QTextStream(&copyText) << QString::number(h_state.imu[2].a_x)
                   << "\t" << QString::number(h_state.imu[2].a_y)
                   << "\t" << QString::number(h_state.imu[2].a_z)
                   << "\n" << QString::number(h_state.imu[2].w_x)
                   << "\t" << QString::number(h_state.imu[2].w_y)
                   << "\t" << QString::number(h_state.imu[2].w_z);
    
    QClipboard* clipboard = QApplication::clipboard();
    clipboard->setText(copyText);
} */

/*void HuboInitWidget::handleJointCopy()
{
    QString copyText;
    
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        double pos = h_state.joint[i].pos;
        if( degSelect->isChecked() )
            pos = pos*180.0/M_PI;
        
        if(i>0)
            QTextStream(&copyText) << "\t";
        QTextStream(&copyText) << QString::number(pos);
    }
    QClipboard* clipboard = QApplication::clipboard();
    clipboard->setText(copyText);
}*/

/*
bool HuboInitWidget::checkJointHomed(int id)
{
    bool result = false;
    if( h_state.status[id].homeFlag == HUBO_HOME_OK )
        result = true;
    return result;
} */

/* bool HuboInitWidget::checkJointError(int id)
{
    bool result = false;
    

    if(h_state.status[id].jam == 1)
        result = true;
    if(h_state.status[id].pwmSaturated == 1)
        result = true;
    if(h_state.status[id].bigError == 1)
        result = true;
    if(h_state.status[id].encError == 1)
        result = true;
    if(h_state.status[id].driverFault == 1)
        result = true;
    if(h_state.status[id].posMinError == 1)
        result = true;
    if(h_state.status[id].posMaxError == 1)
        result = true;
    if(h_state.status[id].velError == 1)
        result = true;
    if(h_state.status[id].accError == 1)
        result = true;
    if(h_state.status[id].tempError == 1)
        result = true;
    
    return result;
} */

/*
void HuboInitWidget::handleHomeAll()
{
    h_cmd.type = D_GOTO_HOME_ALL;
    sendCommand();
} */

/* void HuboInitWidget::handleHomeBad()
{
    bool ignore[HUBO_JOINT_COUNT];
    for(int i=0; i<HUBO_JOINT_COUNT; i++)
        ignore[i] = false;

    for(int i=0; i<HUBO_JOINT_COUNT; i++)
    {
        if( (h_state.status[i].homeFlag != 6 || h_state.status[i].bigError == 1 ) && !ignore[i] && h_state.joint[i].active == 1 )
        {
            if( i != LF1 && i !=LF2 && i != LF3 && i != LF4 && i != LF5
             && i != RF1 && i !=RF2 && i != RF3 && i != RF4 && i != RF5)
            {
                int numMot = h_param.joint[i].numMot;
                int jmc = h_param.joint[i].jmc;

                for(int j=0; j < numMot; j++)
                {
                    h_cmd.joint = h_param.driver[jmc].joints[j];
                    ignore[h_cmd.joint] = true;

                    h_cmd.type = D_ZERO_ENCODER;
                    sendCommand();

                    h_cmd.type = D_GOTO_HOME;
                    sendCommand();
                }
            }
        }
    }
}
*/
/*
void HuboInitWidget::handleInitSensors()
{
    h_cmd.type = D_NULL_SENSORS_ALL;
    sendCommand();
}
*/
/*
void HuboInitWidget::handleJointCmdButton(int id)
{
    h_cmd.joint = id;
    
    if( home->isChecked() )
    {
        h_cmd.type = D_ZERO_ENCODER;
        sendCommand();
        h_cmd.type = D_GOTO_HOME;
    }
    else if( reset->isChecked() )
        h_cmd.type = D_ZERO_ENCODER;
    else if( ctrlOn->isChecked() )
    {
        h_cmd.type = D_CTRL_SWITCH;
        h_cmd.param[0] = D_ENABLE;
    }
    else if( ctrlOff->isChecked() )
    {
        h_cmd.type = D_CTRL_SWITCH;
        h_cmd.param[0] = D_DISABLE;
    }
    else if( fetOn->isChecked() )
    {
        h_cmd.type = D_FET_SWITCH;
        h_cmd.param[0] = D_ENABLE;
    }
    else if( fetOff->isChecked() )
    {
        h_cmd.type = D_FET_SWITCH;
        h_cmd.param[0] = D_DISABLE;
    }
    else if( beep->isChecked() )
    {
        h_cmd.type = D_JMC_BEEP;
        h_cmd.dValues[0] = 1;
    }
    else if( initJoint->isChecked() )
    {
        h_cmd.type = D_JMC_INITIALIZE;
    }
    
    sendCommand();
}
*/
/*
void HuboInitWidget::sendCommand()
{
    if(cmdOpen)
        ach_put(&cmdChan, &h_cmd, sizeof(h_cmd));
    else
        fprintf(stderr, "Command channel is not open!");
}
*/
/*
void HuboInitWidget::commandSensor()
{
    if( nullSensor->isChecked() )
        h_cmd.type = D_NULL_SENSOR;
    if( initSensor->isChecked() )
        h_cmd.type = D_INIT_FT_ACC_SENSOR;
    
    sendCommand();
} */
/*

void HuboInitWidget::handleRHFT()
{
    h_cmd.param[0] = D_R_HAND_FT;
    commandSensor();
}
void HuboInitWidget::handleLHFT()
{
    h_cmd.param[0] = D_L_HAND_FT;
    commandSensor();
}
void HuboInitWidget::handleRFFT()
{
    h_cmd.param[0] = D_R_FOOT_FT;
    commandSensor();
}
void HuboInitWidget::handleLFFT()
{
    h_cmd.param[0] = D_L_FOOT_FT;
    commandSensor();
}
void HuboInitWidget::handleIMU()
{
    h_cmd.param[0] = D_IMU_SENSOR_0;
    commandSensor();
    h_cmd.param[0] = D_IMU_SENSOR_1;
    commandSensor();
    h_cmd.param[0] = D_IMU_SENSOR_2;
    commandSensor();
}
* */

/*void HuboInitWidget::handleJointStateButton(int id)
{
    QString lineStatus;
    QString toolStatus;
    
    QTextStream(&lineStatus) << QString::fromLocal8Bit(h_param.joint[id].name) << " ";
    QTextStream(&lineStatus) << QString::number(h_state.joint[id].pos) << " -- ";
    QTextStream(&toolStatus) << QString::fromLocal8Bit(h_param.joint[id].name) << ":";
    if(h_state.status[id].homeFlag != 6)
    {
        QTextStream(&lineStatus) << "H:" << h_state.status[id].homeFlag << " ";
        QTextStream(&toolStatus) << "\nNot Homed";
    }
    else
        QTextStream(&toolStatus) << "\nHomed";
    if(h_state.joint[id].zeroed==1)
    {
        QTextStream(&lineStatus) << "Z ";
        QTextStream(&toolStatus) << "\nZeroed";
    }
    if(h_state.status[id].jam == 1)
    {
        QTextStream(&lineStatus) << "JAM ";
        QTextStream(&toolStatus) << "\nMechanical Jam";
    }
    if(h_state.status[id].pwmSaturated == 1)
    {
        QTextStream(&lineStatus) << "PWM ";
        QTextStream(&toolStatus) << "\nPWM Saturated";
    }
    if(h_state.status[id].bigError == 1)
    {
        QTextStream(&lineStatus) << "BigE ";
        QTextStream(&toolStatus) << "\nBig Error (Needs to be Reset)";
    }
    if(h_state.status[id].encError == 1)
    {
        QTextStream(&lineStatus) << "ENC ";
        QTextStream(&toolStatus) << "\nEncoder Error";
    }
    if(h_state.status[id].driverFault == 1)
    {
        QTextStream(&lineStatus) << "DF ";
        QTextStream(&toolStatus) << "\nDrive Fault";
    }
    if(h_state.status[id].posMinError == 1)
    {
        QTextStream(&lineStatus) << "POS< ";
        QTextStream(&toolStatus) << "\nMinimum Position Error";
    }
    if(h_state.status[id].posMaxError == 1)
    {
        QTextStream(&lineStatus) << "POS> ";
        QTextStream(&toolStatus) << "\nMaximum Position Error";
    }
    if(h_state.status[id].velError == 1)
    {
        QTextStream(&lineStatus) << "VEL ";
        QTextStream(&toolStatus) << "\nVelocity Error";
    }
    if(h_state.status[id].accError == 1)
    {
        QTextStream(&lineStatus) << "ACC ";
        QTextStream(&toolStatus) << "\nAcceleration Error";
    }
    if(h_state.status[id].tempError == 1)
    {
        QTextStream(&lineStatus) << "TMP ";
        QTextStream(&toolStatus) << "\nTemperature Error";
    }
    
    stateFlags->setText(lineStatus);
    stateFlags->setToolTip(toolStatus);
}*/
/*
int HuboInitWidget::getRefreshTime()
{
    return (int)(refreshRate->value()*1000); // milliseconds
}

void HuboInitWidget::initializeAchStructs()
{
    memset(&h_state, 0, sizeof(h_state));
    memset(&h_param, 0, sizeof(h_param));
    memset(&h_cmd, 0, sizeof(h_cmd));
    setJointParams(&h_param, &h_state);
    setSensorDefaults(&h_param);

    for(int i=0; i<HUBO_JOINT_COUNT; i++)
        if( strcmp(h_param.joint[i].name, "") == 0 )
            sprintf( h_param.joint[i].name, "N/A\0" );
}

void HuboInitWidget::initializeAchConnections()
{
    stateConnected = false;
    cmdConnected = false;
    stateOpen = false;
    cmdOpen = false;


    achChannelState.start("ach mk " + QString::fromLocal8Bit(HUBO_CHAN_STATE_NAME)
                          + " -1 -m 10 -n 8000 -o 666", QIODevice::ReadWrite);
    connect(&achChannelState, SIGNAL(error(QProcess::ProcessError)),
            this, SLOT(achCreateCatch(QProcess::ProcessError)));
    connect(&achChannelState, SIGNAL(started()), this, SLOT(achCreateSHandle()));
    
    achChannelCmd.start("ach -C " + QString::fromLocal8Bit(HUBO_CHAN_BOARD_CMD_NAME)
                        + " -1 -m 10 -n 3000 -o 666", QIODevice::ReadWrite);
    connect(&achChannelCmd, SIGNAL(error(QProcess::ProcessError)),
            this, SLOT(achCreateCatch(QProcess::ProcessError)));
    connect(&achChannelCmd, SIGNAL(started()), this, SLOT(achCreateCHandle()));
    
    connect(&achdState, SIGNAL(started()), this, SLOT(achdSStartedSlot()));
    connect(&achdCmd, SIGNAL(started()), this, SLOT(achdCStartedSlot()));

    
    connect(&achdState, SIGNAL(error(QProcess::ProcessError)), this, SLOT(achdSExitError(QProcess::ProcessError)));
    connect(&achdCmd, SIGNAL(error(QProcess::ProcessError)), this, SLOT(achdCExitError(QProcess::ProcessError)));
    
    connect(&achdState, SIGNAL(finished(int,QProcess::ExitStatus)),
            this, SLOT(achdSExitFinished(int,QProcess::ExitStatus)));
    connect(&achdCmd, SIGNAL(finished(int,QProcess::ExitStatus)),
            this, SLOT(achdCExitFinished(int,QProcess::ExitStatus)));

}

void HuboInitWidget::achCreateSHandle()
{
    ach_status_t r = ach_open(&stateChan, HUBO_CHAN_STATE_NAME, NULL);
    if( r == ACH_OK )
        stateOpen = true;
    else
    {
        ROS_INFO("State Channel failed to open");
        fprintf(stderr, "State Channel failed to open: %s", ach_result_to_string(r));
    }
}

void HuboInitWidget::achCreateCHandle()
{
    ach_status_t r = ach_open(&cmdChan, HUBO_CHAN_BOARD_CMD_NAME, NULL);
    if( r == ACH_OK )
        cmdOpen = true;
    else
    {
        ROS_INFO("Command Channel failed to open");
        fprintf(stderr, "Command Channel failed to open: %s", ach_result_to_string(r));
    }
}

void HuboInitWidget::achdConnectSlot()
{
    achdState.start("achd pull " + QString::number(ipAddrA)
                                 + "." + QString::number(ipAddrB)
                                 + "." + QString::number(ipAddrC)
                                 + "." + QString::number(ipAddrD)
                    + " " + QString::fromLocal8Bit(HUBO_CHAN_STATE_NAME));
    achdCmd.start("achd push " + QString::number(ipAddrA)
                               + "." + QString::number(ipAddrB)
                               + "." + QString::number(ipAddrC)
                               + "." + QString::number(ipAddrD)
                     + " " + QString::fromLocal8Bit(HUBO_CHAN_BOARD_CMD_NAME));
}

void HuboInitWidget::achdDisconnectSlot()
{
    achdState.kill();
    achdCmd.kill();
}

void HuboInitWidget::achdSStartedSlot()
{
    stateConnected = true;
    if( stateConnected && cmdConnected )
        statusLabel->setText("Connected");
}

void HuboInitWidget::achdCStartedSlot()
{
    cmdConnected = true;
    if( stateConnected && cmdConnected )
        statusLabel->setText("Connected");
}

void HuboInitWidget::achdSExitError(QProcess::ProcessError err)
{
    stateConnected = false;
    statusLabel->setText("Not Connected");
}

void HuboInitWidget::achdCExitError(QProcess::ProcessError err)
{
    cmdConnected = false;
    statusLabel->setText("Not Connected");
}

void HuboInitWidget::achdSExitFinished(int num, QProcess::ExitStatus status)
{
    stateConnected = false;
    statusLabel->setText("Not Connected");
}

void HuboInitWidget::achdCExitFinished(int num, QProcess::ExitStatus status)
{
    cmdConnected = false;
    statusLabel->setText("Not Connected");
}


void HuboInitWidget::setIPAddress(int a, int b, int c, int d)
{
    ipAddrA = a;
    ipAddrB = b;
    ipAddrC = c;
    ipAddrD = d;

    ipAddrAEdit->setText(QString::number(ipAddrA));
    ipAddrBEdit->setText(QString::number(ipAddrB));
    ipAddrCEdit->setText(QString::number(ipAddrC));
    ipAddrDEdit->setText(QString::number(ipAddrD));
}

int HuboInitWidget::getIPAddress(int index)
{
    switch(index)
    {
    case 0:
        return ipAddrA; break;
    case 1:
        return ipAddrB; break;
    case 2:
        return ipAddrC; break;
    case 3:
        return ipAddrD; break;
    }
}

void HuboInitWidget::ipEditHandle(const QString &text)
{
    ipAddrA = ipAddrAEdit->text().toInt();
    ipAddrB = ipAddrBEdit->text().toInt();
    ipAddrC = ipAddrCEdit->text().toInt();
    ipAddrD = ipAddrDEdit->text().toInt();
}
*/

void HuboInitWidget::hokuyoEditHandle()
{
    //ipAddrA = ipAddrAEdit->text().toInt();
    std::cerr << "Hokuyo Scan: start at: " <<  txtMinTheta->text().toStdString() << "end at: " << txtMaxTheta->text().toStdString() << " at a rate of " << txtDPS->text().toStdString() << std::endl;
    //std::cerr << "text" ; 
}

} // End: namespace hubo_init_space
