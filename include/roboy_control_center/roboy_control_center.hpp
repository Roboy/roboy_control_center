#pragma once

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <QWidget>
#include <QtQuick/QQuickView>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QSlider>
#include <QLineEdit>
#include <QScrollArea>
#include <QRadioButton>
#include <QVBoxLayout>
#include <QFileDialog>
#include <QLabel>
#include "icebus.hpp"
#include <roboy_control_center/ui_roboy_control_center.h>
#include <roboy_middleware_msgs/ControlMode.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <roboy_middleware_msgs/MotorConfigService.h>
#include <common_utilities/CommonDefinitions.h>
#include <common_utilities/MotorConfig.hpp>

#include <std_srvs/SetBool.h>

#endif

using namespace std;

class RoboyControlCenter
        : public rqt_gui_cpp::Plugin, Icebus {
    Q_OBJECT
public:
    RoboyControlCenter();

    virtual void initPlugin(qt_gui_cpp::PluginContext &context);

    virtual void shutdownPlugin();

    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings,
                              qt_gui_cpp::Settings &instance_settings) const;

    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                 const qt_gui_cpp::Settings &instance_settings);
public Q_SLOTS:
    void stopButtonAllClicked();
private:
    Ui::RoboyControlCenter ui;
    QWidget *widget_;
    ros::NodeHandlePtr nh;
    ros::Publisher motorCommand;
    ros::Subscriber motorState, motorInfo;
    ros::ServiceClient motorControl, motorConfig, emergencyStop;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
private:
    vector<QWidget*> widgets;
};
