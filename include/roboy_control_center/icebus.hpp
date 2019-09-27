#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <common_utilities/CommonDefinitions.h>
#include <common_utilities/MotorConfig.hpp>
#include <roboy_middleware_msgs/MotorState.h>
#include <roboy_middleware_msgs/MotorInfo.h>
#include <map>

#endif

using namespace std;
using namespace ros;
using namespace roboy_middleware_msgs;

class Icebus:public MotorConfig{
public:
    Icebus(){

    };

    void init(){
        motorState = nh->subscribe("roboy/middleware/MotorState",1,&Icebus::MotorState, this);
        motorInfo = nh->subscribe("roboy/middleware/MotorInfo",1,&Icebus::MotorInfo, this);
    }

    void MotorState(const MotorStateConstPtr &msg){
        time.push_back(ros::Time::now().toSec());
        for(int i=0;i<msg->encoder0_pos.size();i++){
            int motor_id_global = icebus[msg->icebus][i]->motor_id_global;
            encoder0_pos[motor_id_global].push_back(msg->encoder0_pos[i]);
            encoder1_pos[motor_id_global].push_back(msg->encoder1_pos[i]);
            displacement[motor_id_global].push_back(msg->displacement[i]);
            if(time.size()>samples){
                encoder0_pos[motor_id_global].pop_front();
                encoder1_pos[motor_id_global].pop_front();
                displacement[motor_id_global].pop_front();
            }
        }
        if(time.size()>samples){
            time.pop_front();
        }
    };

    void MotorInfo(const MotorStateConstPtr &msg){

    };
    NodeHandlePtr nh;
    Subscriber motorState, motorInfo;
    int samples = 100;
    QVector<double> time;
    map<int, QVector<double>> encoder0_pos,encoder1_pos,displacement,current;
};