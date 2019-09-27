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

    QString motorConfigFile = QFileDialog::getOpenFileName(widget_,
                                                    tr("Select motor config file"), "",
                                                    tr("motor config file (*.yaml)"));
    readConfig(motorConfigFile.toStdString());


//    QScrollArea* scrollArea = widget_->findChild<QScrollArea *>("motor_command");
//    scrollArea->setBackgroundRole(QPalette::Window);
//    scrollArea->setFrameShadow(QFrame::Plain);
//    scrollArea->setFrameShape(QFrame::NoFrame);
//    scrollArea->setWidgetResizable(true);
//
//    //vertical box that contains all the checkboxes for the filters
//    motor_command_scrollarea = new QWidget(widget_);
//    motor_command_scrollarea->setObjectName("motor_command_scrollarea");
//    motor_command_scrollarea->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
//    motor_command_scrollarea->setLayout(new QVBoxLayout(motor_command_scrollarea));
//    scrollArea->setWidget(motor_command_scrollarea);
//
//    nh = ros::NodeHandlePtr(new ros::NodeHandle);
//    if (!ros::isInitialized()) {
//        int argc = 0;
//        char **argv = NULL;
//        ros::init(argc, argv, "motor_command_rqt_plugin");
//    }
//
//    spinner.reset(new ros::AsyncSpinner(2));
//    spinner->start();
//
//    if(nh->hasParam("number_of_fpgas")){
//        nh->getParam("number_of_fpgas",number_of_fpgas);
//        ROS_INFO("found number_of_fpgas %d on parameter server", number_of_fpgas);
//    }
//
//    setpoint_slider_widget_all = widget_->findChild<QSlider *>("motor_setPoint_slider_all");
//    QObject::connect(setpoint_slider_widget_all, SIGNAL(valueChanged(int)), this, SLOT(setPointAllChangedSlider()));
//    setpoint_widget_all = widget_->findChild<QLineEdit *>("motor_setPoint_all");
//    QObject::connect(setpoint_widget_all, SIGNAL(editingFinished()), this, SLOT(setPointAllChanged()));
//    scale = widget_->findChild<QLineEdit *>("motor_scale");
//
//    motorCommand = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand", 1);
//    ui.stop_button_all->setStyleSheet("background-color: green");
//    QObject::connect(ui.stop_button_all, SIGNAL(clicked()), this, SLOT(stopButtonAllClicked()));
  }

void RoboyControlCenter::shutdownPlugin() {
    // unregister all publishers here
}

void RoboyControlCenter::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
    // instance_settings.setValue(k, v)
}

void RoboyControlCenter::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
    // v = instance_settings.value(k)
}

void RoboyControlCenter::stopButtonAllClicked() {
    std_srvs::SetBool msg;
    if (ui.stop_button_all->isChecked()) {
        ui.stop_button_all->setStyleSheet("background-color: red");
        msg.request.data = 1;
        for (uint fpga = 2; fpga <= 6; fpga++)
            emergencyStop.call(msg);
    } else {
        ui.stop_button_all->setStyleSheet("background-color: green");
        msg.request.data = 0;
        for (uint fpga = 2; fpga <= 6; fpga++)
            emergencyStop.call(msg);
    }
}

PLUGINLIB_EXPORT_CLASS(RoboyControlCenter, rqt_gui_cpp::Plugin)
