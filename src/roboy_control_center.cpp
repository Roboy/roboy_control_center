#include <roboy_control_center/roboy_control_center.hpp>

RoboyControlCenter::RoboyControlCenter()
        : widget_(0) {
    this->setObjectName("RoboyControlCenter");
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
        nh->setParam("motorConfigFile",motorConfigFile.toStdString());
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
    icebus_scrollarea->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    icebus_scrollarea->setLayout(new QVBoxLayout(icebus_scrollarea));

    QWidget *motor_scrollarea = new QWidget(widget_);
    motor_scrollarea->setObjectName("motor_scrollarea");
    motor_scrollarea->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    motor_scrollarea->setLayout(new QVBoxLayout(motor_scrollarea));

    QWidget *body_part_scrollarea = new QWidget(widget_);
    body_part_scrollarea->setObjectName("bodypart_scrollarea");
    body_part_scrollarea->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    body_part_scrollarea->setLayout(new QVBoxLayout(body_part_scrollarea));

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
            icebus_ui[i].communication_quality->yAxis->setTickStep(1);
            icebus_ui[i].communication_quality->yAxis->setSubTickCount(10);

            QWidget *widget2 = new QWidget(motor_scrollarea);
            widget2->setFixedSize(1031,251);
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
            motor_ui[icebus[i][j]->motor_id_global].encoder0_pos->addGraph();
            motor_ui[icebus[i][j]->motor_id_global].encoder0_pos->addGraph();
            motor_ui[icebus[i][j]->motor_id_global].encoder0_pos->graph(0)->setPen(QColor(Qt::blue));
            motor_ui[icebus[i][j]->motor_id_global].encoder0_pos->graph(1)->setPen(QColor(Qt::lightGray));
            motor_ui[icebus[i][j]->motor_id_global].encoder0_pos->graph(1)->setName(QString::number(icebus[i][j]->motor_id_global));
            motor_ui[icebus[i][j]->motor_id_global].encoder0_pos->graph(0)->setName(QString::number(icebus[i][j]->motor_id_global));
            motor_ui[icebus[i][j]->motor_id_global].encoder0_pos->xAxis->setTickLabels(false);
            motor_ui[icebus[i][j]->motor_id_global].encoder0_pos->setAttribute(Qt::WA_NoMousePropagation, false);
            motor_ui[icebus[i][j]->motor_id_global].encoder1_pos->addGraph();
            motor_ui[icebus[i][j]->motor_id_global].encoder1_pos->addGraph();
            motor_ui[icebus[i][j]->motor_id_global].encoder1_pos->graph(0)->setPen(QColor(Qt::red));
            motor_ui[icebus[i][j]->motor_id_global].encoder1_pos->graph(1)->setPen(QColor(Qt::lightGray));
            motor_ui[icebus[i][j]->motor_id_global].encoder1_pos->graph(1)->setName(QString::number(icebus[i][j]->motor_id_global));
            motor_ui[icebus[i][j]->motor_id_global].encoder1_pos->graph(0)->setName(QString::number(icebus[i][j]->motor_id_global));
            motor_ui[icebus[i][j]->motor_id_global].encoder1_pos->xAxis->setTickLabels(false);
            motor_ui[icebus[i][j]->motor_id_global].encoder1_pos->setAttribute(Qt::WA_NoMousePropagation, false);
            motor_ui[icebus[i][j]->motor_id_global].displacement->addGraph();
            motor_ui[icebus[i][j]->motor_id_global].displacement->addGraph();
            motor_ui[icebus[i][j]->motor_id_global].displacement->graph(0)->setPen(QColor(Qt::green));
            motor_ui[icebus[i][j]->motor_id_global].displacement->graph(1)->setPen(QColor(Qt::lightGray));
            motor_ui[icebus[i][j]->motor_id_global].displacement->graph(1)->setName(QString::number(icebus[i][j]->motor_id_global));
            motor_ui[icebus[i][j]->motor_id_global].displacement->graph(0)->setName(QString::number(icebus[i][j]->motor_id_global));
            motor_ui[icebus[i][j]->motor_id_global].displacement->xAxis->setTickLabels(false);
            motor_ui[icebus[i][j]->motor_id_global].displacement->setAttribute(Qt::WA_NoMousePropagation, false);
            motor_ui[icebus[i][j]->motor_id_global].current->addGraph();
            motor_ui[icebus[i][j]->motor_id_global].current->graph(0)->setPen(QColor(Qt::darkBlue));
            motor_ui[icebus[i][j]->motor_id_global].current->graph(0)->setName(QString::number(icebus[i][j]->motor_id_global));
            motor_ui[icebus[i][j]->motor_id_global].current->xAxis->setTickLabels(false);
            motor_ui[icebus[i][j]->motor_id_global].current->setAttribute(Qt::WA_NoMousePropagation, false);
            QObject::connect(motor_ui[icebus[i][j]->motor_id_global].encoder0_pos,
                    SIGNAL(plottableDoubleClick(QCPAbstractPlottable*, QMouseEvent*)),
                    this, SLOT(focusEncoder0Plot(QCPAbstractPlottable*, QMouseEvent*)));
            QObject::connect(motor_ui[icebus[i][j]->motor_id_global].encoder1_pos,
                             SIGNAL(plottableDoubleClick(QCPAbstractPlottable*, QMouseEvent*)),
                             this, SLOT(focusEncoder1Plot(QCPAbstractPlottable*, QMouseEvent*)));
            QObject::connect(motor_ui[icebus[i][j]->motor_id_global].displacement,
                             SIGNAL(plottableDoubleClick(QCPAbstractPlottable*, QMouseEvent*)),
                             this, SLOT(focusDisplacementPlot(QCPAbstractPlottable*, QMouseEvent*)));
            QObject::connect(motor_ui[icebus[i][j]->motor_id_global].current,
                             SIGNAL(plottableDoubleClick(QCPAbstractPlottable*, QMouseEvent*)),
                             this, SLOT(focusCurrentPlot(QCPAbstractPlottable*, QMouseEvent*)));
            motor_scrollarea->layout()->addWidget(widget2);
        }
        icebus_scrollarea->layout()->addWidget(widget);
    }

    scrollArea->setWidget(icebus_scrollarea);
    scrollArea2->setWidget(motor_scrollarea);
    scrollArea3->setWidget(body_part_scrollarea);

    for(int i=0;i<body_part.size();i++){
        QWidget *widget = new QWidget(body_part_scrollarea);
        body_part_ui[body_part[i]->name].setupUi(widget);
        body_part_ui[body_part[i]->name].body_part_name->setText(QString::asprintf("%s",body_part[i]->name.c_str()));
        body_part_scrollarea->layout()->addWidget(widget);
    }

    QObject::connect(this, SIGNAL(triggerMotorInfoUpdate()), this, SLOT(plotMotorInfo()));
    QObject::connect(this, SIGNAL(triggerMotorStateUpdate()), this, SLOT(plotMotorState()));

    spinner.reset(new ros::AsyncSpinner(2));
    spinner->start();

  }

void RoboyControlCenter::shutdownPlugin() {
    // unregister all publishers here
}

void RoboyControlCenter::saveSettings(qt_gui_cpp::Settings &plugin_settings,
                                    qt_gui_cpp::Settings &instance_settings) const {
}

void RoboyControlCenter::restoreSettings(const qt_gui_cpp::Settings &plugin_settings,
                                       const qt_gui_cpp::Settings &instance_settings) {
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

void RoboyControlCenter::plotMotorInfo() {
    if(ui.tabs->currentIndex()==0) {
        for (int i = 0; i < number_of_icebuses; i++) {
            for (int j = 0; j < icebus[i].size(); j++) {
                int motor_id_global = icebus[i][j]->motor_id_global;
                icebus_ui[i].communication_quality->graph(j)->setData(motorInfoTimeStamps,
                                                                      communication_quality[motor_id_global]);
                icebus_ui[i].communication_quality->xAxis->rescale();
                icebus_ui[i].communication_quality->replot();
            }
        }
    }
}

void RoboyControlCenter::plotMotorState() {
    if(ui.tabs->currentIndex()==1) {
        for (int i = 0; i < number_of_icebuses; i++) {
            for (int j = 0; j < icebus[i].size(); j++) {
                int motor_id_global = icebus[i][j]->motor_id_global;
                motor_ui[motor_id_global].encoder0_pos->graph(0)->setData(motorStateTimeStamps,
                                                                          encoder0_pos[motor_id_global]);
                motor_ui[motor_id_global].encoder1_pos->graph(0)->setData(motorStateTimeStamps,
                                                                          encoder1_pos[motor_id_global]);
                motor_ui[motor_id_global].displacement->graph(0)->setData(motorStateTimeStamps,
                                                                          displacement[motor_id_global]);
                motor_ui[motor_id_global].current->graph(0)->setData(motorStateTimeStamps, current[motor_id_global]);
                switch (control_mode[motor_id_global]) {
                    case 0:
                        motor_ui[motor_id_global].encoder0_pos->graph(1)->setData(motorStateTimeStamps,
                                                                                  setpoint[motor_id_global]);
                        motor_ui[motor_id_global].encoder1_pos->graph(1)->clearData();
                        motor_ui[motor_id_global].displacement->graph(1)->clearData();
                    case 1:
                        motor_ui[motor_id_global].encoder0_pos->graph(1)->clearData();
                        motor_ui[motor_id_global].encoder1_pos->graph(1)->setData(motorStateTimeStamps,
                                                                                  setpoint[motor_id_global]);
                        motor_ui[motor_id_global].displacement->graph(1)->clearData();
                    case 2:
                        motor_ui[motor_id_global].encoder0_pos->graph(1)->clearData();
                        motor_ui[motor_id_global].encoder1_pos->graph(1)->clearData();
                        motor_ui[motor_id_global].displacement->graph(1)->setData(motorStateTimeStamps,
                                                                                  setpoint[motor_id_global]);
                }
                motor_ui[motor_id_global].encoder0_pos->rescaleAxes();
                motor_ui[motor_id_global].encoder1_pos->rescaleAxes();
                motor_ui[motor_id_global].displacement->rescaleAxes();
                motor_ui[motor_id_global].current->rescaleAxes();
                motor_ui[motor_id_global].encoder0_pos->replot();
                motor_ui[motor_id_global].encoder1_pos->replot();
                motor_ui[motor_id_global].displacement->replot();
                motor_ui[motor_id_global].current->replot();
            }
        }
    }
}

void RoboyControlCenter::plotData(){
    if(ui.tabs->currentIndex()==3) {
        ui.plot->graph(0)->setData(*x, *y);
        ui.plot->rescaleAxes();
        ui.plot->replot();
    }
}

void RoboyControlCenter::focusEncoder0Plot(QCPAbstractPlottable *plottable, QMouseEvent* event){
    QObject::disconnect(plotConnection);
    ROS_INFO("encoder0 %s", plottable->name().toStdString().c_str());
    int m = plottable->name().toInt();
    ui.plot->clearGraphs();
    ui.plot->addGraph();
    x = &motorStateTimeStamps;
    y = &encoder0_pos[m];
    plotConnection = QObject::connect(this, SIGNAL(triggerMotorStateUpdate()), this, SLOT(plotData()));
}
void RoboyControlCenter::focusEncoder1Plot(QCPAbstractPlottable *plottable, QMouseEvent* event){
    ROS_INFO("encoder1 %s", plottable->name().toStdString().c_str());
}
void RoboyControlCenter::focusDisplacementPlot(QCPAbstractPlottable *plottable, QMouseEvent* event){
    ROS_INFO("displacment %s", plottable->name().toStdString().c_str());
}
void RoboyControlCenter::focusCurrentPlot(QCPAbstractPlottable *plottable, QMouseEvent* event){
    ROS_INFO("current %s", plottable->name().toStdString().c_str());
}

PLUGINLIB_EXPORT_CLASS(RoboyControlCenter, rqt_gui_cpp::Plugin)
