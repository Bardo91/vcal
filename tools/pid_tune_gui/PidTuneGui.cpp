//
//
//
//
//
//

#include "PidTuneGui.h"
#include <QTime>

PidTuneGui::PidTuneGui( std::string _refTopic, 
                        std::string _valTopic, 
                        std::vector<std::pair<std::string, std::string>> _pidBaseTopics,     
                        std::unordered_map<std::string, std::string> _pidComParams,
                        QWidget *parent): QMainWindow(parent) {

    mCentralWidget = new QWidget();
    mMainLayout = new QHBoxLayout();
    mCentralWidget->setLayout(mMainLayout);
    setCentralWidget(mCentralWidget);
    this->setWindowTitle("PID Tune Gui");

    // Add text entries for topics     
    mButtonsLayout = new QVBoxLayout();

    mPosEdit = new QLineEdit(_valTopic.c_str());
    mPosEdit->setMaximumWidth(200);
    mButtonsLayout->addWidget(mPosEdit);

    mRefEdit = new QLineEdit(_refTopic.c_str());
    mRefEdit->setMaximumWidth(200);
    mButtonsLayout->addWidget(mRefEdit);

    mButtonsOptionsLayout = new QHBoxLayout();
    mChangeSubscribers = new QPushButton("Change subscribers");
    mButtonsOptionsLayout->addWidget(mChangeSubscribers);
    connect(mChangeSubscribers, SIGNAL (released()), this, SLOT (connectTopics()));
    mButtonsLayout->addLayout(mButtonsOptionsLayout);

    // BUTTONS FOR K: X, Y, Z
    mButtonsLayoutFilaK = new QHBoxLayout();
    for(auto &baseTopic: _pidBaseTopics){
        mButtonsModifyXK = new QVBoxLayout();
        mWidgetsParams[baseTopic.first+" kp"] = new ParamWidget(baseTopic.first+" kp",   0.8f);
        mButtonsModifyXK->addLayout(mWidgetsParams[baseTopic.first+" kp"]);

        mWidgetsParams[baseTopic.first+" ki"] = new ParamWidget(baseTopic.first+" ki",   0.0f);
        mButtonsModifyXK->addLayout(mWidgetsParams[baseTopic.first+" ki"]);

        mWidgetsParams[baseTopic.first+" kd"] = new ParamWidget(baseTopic.first+" kd",   0.7f);
        mButtonsModifyXK->addLayout(mWidgetsParams[baseTopic.first+" kd"]);

        mWidgetsParams[baseTopic.first+" sat"] = new ParamWidget(baseTopic.first+" sat",  0.0f);
        mButtonsModifyXK->addLayout(mWidgetsParams[baseTopic.first+" sat"]);

        mWidgetsParams[baseTopic.first+" wind"] = new ParamWidget(baseTopic.first+" wind", 0.0f);
        mButtonsModifyXK->addLayout(mWidgetsParams[baseTopic.first+" wind"]);

        mButtonsLayoutFilaK->addLayout(mButtonsModifyXK);
    }
    mButtonsLayout->addLayout(mButtonsLayoutFilaK);
    // BUTTONS FOR MOVE
    mMainLayout->addLayout(mButtonsLayout); 
    
    /// GRAPH X
    mGraphLayout = new QVBoxLayout();
    mGraphPositionX = new QCustomPlot();
    mGraphLayout->addWidget(mGraphPositionX);
    mMainLayout->addLayout(mGraphLayout);

    mGraphPositionX->addGraph(); // red line X
    QPen pen;
    pen.setWidthF(4);
    pen.setColor(QColor(255, 0, 0));
    mGraphPositionX->graph(0)->setPen(pen);

    mGraphPositionX->addGraph(); // red line X ref
    pen.setColor(QColor(255, 0, 0));
    pen.setStyle(Qt::DotLine);
    mGraphPositionX->graph(1)->setPen(pen);
    
    QSharedPointer<QCPAxisTickerTime> mTimeTicker(new QCPAxisTickerTime);
    mTimeTicker->setTimeFormat("%h:%m:%s");
    mGraphPositionX->xAxis->setTicker(mTimeTicker);
    mGraphPositionX->yAxis->setRange(-1.0, 1.0);

    // GRAPH Y
    mGraphPositionY = new QCustomPlot();
    mGraphLayout->addWidget(mGraphPositionY);
    mGraphPositionY->addGraph(); // green line Y
    pen.setColor(QColor(0, 255, 0));
    pen.setStyle(Qt::SolidLine);
    mGraphPositionY->graph(0)->setPen(pen);

    mGraphPositionY->addGraph(); // green line Y ref
    pen.setColor(QColor(0, 255, 0));
    pen.setStyle(Qt::DotLine);
    mGraphPositionY->graph(1)->setPen(pen);

    mGraphPositionY->xAxis->setTicker(mTimeTicker);
    mGraphPositionY->yAxis->setRange(-1.0, 1.0);

    // GRAPH Z
    mGraphPositionZ = new QCustomPlot();
    mGraphLayout->addWidget(mGraphPositionZ);
    mGraphPositionZ->addGraph(); // blue line Z
    pen.setColor(QColor(0, 0, 255));
    pen.setStyle(Qt::SolidLine);
    mGraphPositionZ->graph(0)->setPen(pen);

    mGraphPositionZ->addGraph(); // blue line Z ref
    pen.setColor(QColor(0, 0, 255));
    pen.setStyle(Qt::DotLine);
    mGraphPositionZ->graph(1)->setPen(pen);

    mGraphPositionZ->xAxis->setTicker(mTimeTicker);
    mGraphPositionZ->yAxis->setRange(-1.0, 1.0);
    
    connect(mGraphPositionX->xAxis, SIGNAL(rangeChanged(QCPRange)), mGraphPositionX->xAxis2, SLOT(setRange(QCPRange)));
    connect(mGraphPositionX->yAxis, SIGNAL(rangeChanged(QCPRange)), mGraphPositionX->yAxis2, SLOT(setRange(QCPRange)));
     
    connect(mGraphPositionY->xAxis, SIGNAL(rangeChanged(QCPRange)), mGraphPositionY->xAxis2, SLOT(setRange(QCPRange)));
    connect(mGraphPositionY->yAxis, SIGNAL(rangeChanged(QCPRange)), mGraphPositionY->yAxis2, SLOT(setRange(QCPRange)));

    connect(mGraphPositionZ->xAxis, SIGNAL(rangeChanged(QCPRange)), mGraphPositionZ->xAxis2, SLOT(setRange(QCPRange)));
    connect(mGraphPositionZ->yAxis, SIGNAL(rangeChanged(QCPRange)), mGraphPositionZ->yAxis2, SLOT(setRange(QCPRange)));

    mDataTimer = new QTimer(this);
    connect(mDataTimer, SIGNAL(timeout()), this, SLOT(realTimePlot()));
    mDataTimer->start(0);

    mEstimateSubscriber = new fastcom::Subscriber<TripleFloat>(_pidComParams["uav_address"], atoi(_pidComParams["estimate_vision"].c_str()));
    mEstimateSubscriber->attachCallback([&](const TripleFloat &_params){
        mLastX = _params.f1;
        mLastY = _params.f2;
        mLastZ = _params.f3;
    });
    
    mReferenceSubscriber = new fastcom::Subscriber<TripleFloat>(_pidComParams["uav_address"], atoi(_pidComParams["reference_vision"].c_str()));
    mReferenceSubscriber->attachCallback([&](const TripleFloat &_params){
        mLastRefX = _params.f1;
        mLastRefY = _params.f2;
        mLastRefZ = _params.f3;
    });

    mSubscriberPidX = new fastcom::Subscriber<PIDParams>(_pidComParams["uav_address"], atoi(_pidComParams["port_pid_x"].c_str()));
    mSubscriberPidX->attachCallback([&](const PIDParams &_params){
        mWidgetsParams["X kp"]->setValue(_params.kp);
        mWidgetsParams["X ki"]->setValue(_params.ki);
        mWidgetsParams["X kd"]->setValue(_params.kd);
        mWidgetsParams["X sat"]->setValue(_params.sat);
        mWidgetsParams["X wind"]->setValue(_params.wind);
    });

    mSubscriberPidY = new fastcom::Subscriber<PIDParams>(_pidComParams["uav_address"], atoi(_pidComParams["port_pid_y"].c_str()));
    mSubscriberPidX->attachCallback([&](const PIDParams &_params){
        mWidgetsParams["Y kp"]->setValue(_params.kp);
        mWidgetsParams["Y ki"]->setValue(_params.ki);
        mWidgetsParams["Y kd"]->setValue(_params.kd);
        mWidgetsParams["Y sat"]->setValue(_params.sat);
        mWidgetsParams["Y wind"]->setValue(_params.wind);
    });
    
    mSubscriberPidZ = new fastcom::Subscriber<PIDParams>(_pidComParams["uav_address"], atoi(_pidComParams["port_pid_z"].c_str()));
    mSubscriberPidX->attachCallback([&](const PIDParams &_params){
        mWidgetsParams["Z kp"]->setValue(_params.kp);
        mWidgetsParams["Z ki"]->setValue(_params.ki);
        mWidgetsParams["Z kd"]->setValue(_params.kd);
        mWidgetsParams["Z sat"]->setValue(_params.sat);
        mWidgetsParams["Z wind"]->setValue(_params.wind);
    });

}


void PidTuneGui::closeEvent(QCloseEvent *event) {

}

void PidTuneGui::realTimePlot(){
    static QTime time(QTime::currentTime());
    // calculate two new data points:
    double key = time.elapsed()/1000.0; // time elapsed since start of demo, in seconds
    static double lastPointKey = 0;
    if (key-lastPointKey > 0.005) { // at most add point every 2 ms
      // add data to lines:
      mGraphPositionX->graph(0)->addData(key, mLastX);
      mGraphPositionX->graph(1)->addData(key, mLastRefX);
      
      mGraphPositionY->graph(0)->addData(key, mLastY);
      mGraphPositionY->graph(1)->addData(key, mLastRefY);

      mGraphPositionZ->graph(0)->addData(key, mLastZ);
      mGraphPositionZ->graph(1)->addData(key, mLastRefZ);

      lastPointKey = key;
    }
    // make key axis range scroll with the data (at a constant range size of 8):
    
    double min = mLastX-1 < mLastRefX - 1? mLastX -1: mLastRefX -1;
    double max = mLastX+1 > mLastRefX + 1? mLastX +1: mLastRefX +1;
    mGraphPositionX->yAxis->setRange(min, max);

    min = mLastY-1 < mLastRefY - 1? mLastY -1: mLastRefY -1;
    max = mLastY+1 > mLastRefY + 1? mLastY +1: mLastRefY +1;
    mGraphPositionY->yAxis->setRange(min, max);

    min = mLastZ-1 < mLastRefZ - 1? mLastZ -1: mLastRefZ -1;
    max = mLastZ+1 > mLastRefZ + 1? mLastZ +1: mLastRefZ +1;
    mGraphPositionZ->yAxis->setRange(min, max);

    mGraphPositionX->xAxis->setRange(key, 8, Qt::AlignRight);
    mGraphPositionX->replot();

    mGraphPositionY->xAxis->setRange(key, 8, Qt::AlignRight);
    mGraphPositionY->replot();

    mGraphPositionZ->xAxis->setRange(key, 8, Qt::AlignRight);
    mGraphPositionZ->replot();
}

void PidTuneGui::connectTopics(){
    // other
    ros::NodeHandle n;
    mSubPos = n.subscribe<geometry_msgs::PoseStamped>(mPosEdit->text().toStdString(),1, &PidTuneGui::positionCallback, this);
    mSubRef = n.subscribe<geometry_msgs::PoseStamped>(mRefEdit->text().toStdString(), 1,&PidTuneGui::referenceCallback, this);
    //mSubXKp = n.subscribe<std_msgs::Float32>("/PID_x/kp", 1,&PidTuneGui::changePIDCallback, this);

}

//void PidTuneGui::changePIDCallback(const std_msgs::Float32::ConstPtr &_data){}



void PidTuneGui::positionCallback(const geometry_msgs::PoseStamped::ConstPtr &_data){
    mLastX = _data->pose.position.x;
    mLastY = _data->pose.position.y;
    mLastZ = _data->pose.position.z;
}

void PidTuneGui::referenceCallback(const geometry_msgs::PoseStamped::ConstPtr &_data){
    mLastRefX = _data->pose.position.x;
    mLastRefY = _data->pose.position.y;
    mLastRefZ = _data->pose.position.z;
}
