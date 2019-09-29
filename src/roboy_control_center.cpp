#include <roboy_control_center/roboy_control_center.hpp>

RoboyControlCenter::RoboyControlCenter()
        : rqt_gui_cpp::Plugin(), widget_(0) {
    setObjectName("RoboyControlCenter");
}

void RoboyControlCenter::initPlugin(qt_gui_cpp::PluginContext &context) {
    // access standalone command line arguments
    QStringList argv = context.argv();
    // create QWidget
    widget_ = new QWidget();
    // extend the widget with all attributes and children from UI file
    ui.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    nh = ros::NodeHandlePtr(new ros::NodeHandle);
    if (!ros::isInitialized()) {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "RoboyControlCenter");
    }

    string configFile;
    nh->getParam("motorConfigFile", configFile);
    if(!readConfig(configFile)){
        motorConfigFile = QFileDialog::getOpenFileName(widget_,
                                                       tr("Select motor config file"), motorConfigFile,
                                                       tr("motor config file (*.yaml)"));
        if(!readConfig(motorConfigFile.toStdString())){
            ROS_FATAL("could not get any config file, i give up!");
        }
    }else{
        motorConfigFile = QString::fromStdString(configFile);
    }
    initTopics(nh);
    QScrollArea* scrollArea = widget_->findChild<QScrollArea *>("icebus");
    scrollArea->setWidgetResizable(true);

    QScrollArea* scrollArea2 = widget_->findChild<QScrollArea *>("motor");
    scrollArea2->setWidgetResizable(true);

    QScrollArea* scrollArea3 = widget_->findChild<QScrollArea *>("body_part_control");
    scrollArea3->setWidgetResizable(true);

    QWidget *icebus_scrollarea = new QWidget(widget_);
    icebus_scrollarea->setObjectName("icebus_scrollarea");
    icebus_scrollarea->setLayout(new QVBoxLayout(icebus_scrollarea));
    scrollArea->setWidget(icebus_scrollarea);

    QWidget *motor_scrollarea = new QWidget(widget_);
    motor_scrollarea->setObjectName("motor_scrollarea");
    motor_scrollarea->setLayout(new QVBoxLayout(motor_scrollarea));
    scrollArea2->setWidget(motor_scrollarea);

    QWidget *body_part_scrollarea = new QWidget(widget_);
    body_part_scrollarea->setObjectName("bodypart_scrollarea");
    body_part_scrollarea->setLayout(new QVBoxLayout(motor_scrollarea));
    scrollArea3->setWidget(body_part_scrollarea);

    for(int i=0;i<number_of_icebuses;i++){
        QWidget *widget = new QWidget(icebus_scrollarea);
        icebus_ui[i].setupUi(widget);
        icebus_ui[i].icebus_name->setText(QString::asprintf("icebus%d",i));
        for(int j=0;j<icebus[i].size();j++){
            icebus_ui[i].communication_quality->addGraph();
            icebus_ui[i].communication_quality->graph(j)->setPen(QPen(color_pallette[j%16]));
            icebus_ui[i].communication_quality->xAxis->setLabel("");
            icebus_ui[i].communication_quality->yAxis->setLabel("%");
            icebus_ui[i].communication_quality->yAxis->setLabelPadding(50);
            icebus_ui[i].communication_quality->yAxis->setRange(0,100);
            icebus_ui[i].communication_quality->yAxis->setTickLabels(false);

            QWidget *widget2 = new QWidget(motor_scrollarea);
            motor_ui[icebus[i][j]->motor_id_global].setupUi(widget2);
            motor_ui[icebus[i][j]->motor_id_global].globalID->setText(QString::asprintf("globalID:  %d",icebus[i][j]->motor_id_global));
            motor_ui[icebus[i][j]->motor_id_global].icebus->setText(QString::asprintf  ("icebus:    %d",icebus[i][j]->icebus));
            motor_ui[icebus[i][j]->motor_id_global].icebusID->setText(QString::asprintf("icebusID:  %d",icebus[i][j]->icebus_id));
            motor_ui[icebus[i][j]->motor_id_global].muscleType->setText(QString::asprintf("%s",icebus[i][j]->muscleType.c_str()));
            if(icebus[i][j]->muscleType=="myoMuscle")
                motor_ui[icebus[i][j]->motor_id_global].muscleType->setStyleSheet("background-color: green");
            else if(icebus[i][j]->muscleType=="myoBrick100")
                motor_ui[icebus[i][j]->motor_id_global].muscleType->setStyleSheet("background-color: violet");
            else if(icebus[i][j]->muscleType=="myoBrick300")
                motor_ui[icebus[i][j]->motor_id_global].muscleType->setStyleSheet("background-color: yellow");
            else
                motor_ui[icebus[i][j]->motor_id_global].muscleType->setStyleSheet("background-color: red");

            motor_scrollarea->layout()->addWidget(widget2);
        }
        icebus_scrollarea->layout()->addWidget(widget);
    }

    for(int i=0;i<body_part.size();i++){
        QWidget *widget = new QWidget(body_part_scrollarea);
        body_part_ui[body_part[i]->name].setupUi(widget);
        body_part_ui[body_part[i]->name].body_part_name->setText(QString::asprintf("%s",body_part[i]->name.c_str()));
        body_part_scrollarea->layout()->addWidget(widget);
    }

    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();

  }

void RoboyControlCenter::shutdownPlugin() {
    // unregister all publishers here
}

void RoboyControlCenter::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
     instance_settings.setValue("motorConfigFile", motorConfigFile);
}

void RoboyControlCenter::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    motorConfigFile = instance_settings.value("motorConfigFile").toString();
}

void RoboyControlCenter::stopButtonAllClicked() {
    std_srvs::SetBool msg;
    if (ui.stop_button_all->isChecked()) {
        ui.stop_button_all->setStyleSheet("background-color: red");
        msg.request.data = 1;
        emergencyStop.call(msg);
    } else {
        ui.stop_button_all->setStyleSheet("background-color: green");
        msg.request.data = 0;
        emergencyStop.call(msg);
    }
}

PLUGINLIB_EXPORT_CLASS(RoboyControlCenter, rqt_gui_cpp::Plugin)
