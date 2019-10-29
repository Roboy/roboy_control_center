#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <common_utilities/CommonDefinitions.h>
#include <common_utilities/MotorConfig.hpp>
#include <roboy_middleware_msgs/MotorState.h>
#include <roboy_middleware_msgs/MotorInfo.h>
#include <map>
#include <rqt_gui_cpp/plugin.h>

#endif

using namespace std;
using namespace ros;
using namespace roboy_middleware_msgs;

class Icebus:public rqt_gui_cpp::Plugin,public MotorConfig{
    Q_OBJECT
public:
    Icebus():rqt_gui_cpp::Plugin(){

    };

    void initTopics(NodeHandlePtr nh){
        startTime = ros::Time::now().toSec();
        motorState = nh->subscribe("/roboy/middleware/MotorState",1,&Icebus::MotorState, this);
        motorInfo = nh->subscribe("/roboy/middleware/MotorInfo",1,&Icebus::MotorInfo, this);
    }

    void MotorState(const MotorStateConstPtr &msg){
        motorStateTimeStamps.push_back(ros::Time::now().toSec()-startTime);
        for(int i=0;i<msg->encoder0_pos.size();i++){
            int motor_id_global = icebus[0/*msg->icebus*/][i]->motor_id_global;
            setpoint[motor_id_global].push_back(msg->setpoint[i]);
            encoder0_pos[motor_id_global].push_back(msg->encoder0_pos[i]);
            encoder1_pos[motor_id_global].push_back(msg->encoder1_pos[i]);
            displacement[motor_id_global].push_back(msg->displacement[i]);
            current[motor_id_global].push_back(msg->current[i]);
            if(motorStateTimeStamps.size()>samples){
                setpoint[motor_id_global].push_back(msg->setpoint[i]);
                encoder0_pos[motor_id_global].pop_front();
                encoder1_pos[motor_id_global].pop_front();
                displacement[motor_id_global].pop_front();
                current[motor_id_global].pop_front();
            }
        }
        if(motorStateTimeStamps.size()>samples){
            motorStateTimeStamps.pop_front();
        }
        if((motorStateTimeStamps.back()-lastMotorStateUpdate)>0.1){ // update at 10Hz
            lastMotorStateUpdate = motorStateTimeStamps.back();
            Q_EMIT triggerMotorStateUpdate();
        }
    };

    void MotorInfo(const MotorInfoConstPtr &msg){
        motorInfoTimeStamps.push_back(ros::Time::now().toSec()-startTime);
        for(int i=0;i<msg->communication_quality.size();i++){
            int motor_id_global = icebus[0/*msg->icebus*/][i]->motor_id_global;
            control_mode[motor_id_global] = msg->control_mode[i];
            Kp[motor_id_global] = msg->Kp[i];
            Ki[motor_id_global] = msg->Ki[i];
            Kd[motor_id_global] = msg->Kd[i];
            deadband[motor_id_global] = msg->deadband[i];
            IntegralLimit[motor_id_global] = msg->IntegralLimit[i];
            PWMLimit[motor_id_global] = msg->PWMLimit[i];
            pwm[motor_id_global].push_back(msg->pwm[i]);
            communication_quality[motor_id_global].push_back(msg->communication_quality[i]);
            gearBoxRatio[motor_id_global] = msg->gearBoxRatio[i];

            if(motorInfoTimeStamps.size()>samples){
                setpoint[motor_id_global].pop_front();
                pwm[motor_id_global].pop_front();
                communication_quality[motor_id_global].pop_front();
            }
        }
        if(motorInfoTimeStamps.size()>samples){
            motorInfoTimeStamps.pop_front();
        }
        if((motorInfoTimeStamps.back()-lastMotorInfoUpdate)>1){ // update at 10Hz
            lastMotorInfoUpdate = motorInfoTimeStamps.back();
            Q_EMIT triggerMotorInfoUpdate();
        }
    };

    Q_SIGNALS:
        void triggerMotorStateUpdate();
        void triggerMotorInfoUpdate();
public:
    Subscriber motorState, motorInfo;
    int samples = 500;
    QVector<double> motorStateTimeStamps, motorInfoTimeStamps;
    map<int, QVector<double>> encoder0_pos,encoder1_pos,displacement,current,communication_quality, setpoint, pwm;
    map<int, int> control_mode, Kp, Ki, Kd, deadband, IntegralLimit, PWMLimit, gearBoxRatio;
    double lastMotorStateUpdate, lastMotorInfoUpdate, startTime;
};