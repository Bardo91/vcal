//
//
//
//
//

#ifndef PIDTUNEGUI_H_
#define PIDTUNEGUI_H_

    #include <ros/ros.h>
    #include <QHBoxLayout>
    #include <QVBoxLayout>
    #include <QMainWindow>
    #include <QLineEdit>
    #include <QTextEdit>
    #include "ParamWidget.h"
    #include <map>
    #include <geometry_msgs/PoseStamped.h>
    #include "qcustomplot.h"
    #include "AnalisysWindow.h"
    #include <vector>
    #include <fastcom/fastcom.h>
    #include <unordered_map>

    class PidTuneGui : public QMainWindow {
        Q_OBJECT
    public:
        PidTuneGui( std::string _refTopic, 
                    std::string _valTopic,  
                    std::vector<std::pair<std::string, std::string>> _pidBaseTopics, 
                    std::unordered_map<std::string, std::string> pidComParams,
                    QWidget *parent = 0);
        virtual ~PidTuneGui(){};

    protected:
        void closeEvent(QCloseEvent *event) override;
        
    private slots:
        void realTimePlot();
        void connectTopics();

    private:
        void referenceCallback(const geometry_msgs::PoseStamped::ConstPtr &_data);
        void positionCallback(const geometry_msgs::PoseStamped::ConstPtr &_data);
        //void changePIDCallback(const std_msgs::Float32::ConstPtr &_data);
        
    private:
        QHBoxLayout *mMainLayout, *mButtonsOptionsLayout;

        QVBoxLayout *mButtonsModifySat, *mButtonsModifyWP, *mButtonsModifyXK, *mButtonsModifyYK, *mButtonsModifyZK, *mButtonsModifyRef, *mButtonsTime, *mAnalysisTimeError, *mAnalysisTimePercentage;
        QHBoxLayout *mButtonsLayoutFilaK, *mButtonsLayoutFilaSW, *mButtonsLayoutFilaRef, *mLayoutFilaErr;

        QVBoxLayout *mButtonsLayout, *mGraphLayout;
        QWidget     *mCentralWidget;
        QLineEdit *mPosEdit, *mRefEdit;
        QLineEdit *mNameError, *mNamePercentage, *mTextErrorX, *mTextErrorY, *mTextErrorZ, *mTextPercentageX, *mTextPercentageY, *mTextPercentageZ;
        QPushButton *mChangeSubscribers;
       

        QCustomPlot *mGraphPositionX, *mGraphPositionY, *mGraphPositionZ;
        QTimer *mDataTimer;
        ros::Subscriber mSubPos, mSubRef, mSubXKp;
        float mLastX = 0, mLastY = 0, mLastZ = 0, mLastRefX = 0, mLastRefY = 0, mLastRefZ = 0;

        QPushButton *mAnalisysWindowButton;
        AnalisysWindow *mLastAnalisysWindow = nullptr;

        float mErrorX = 0, mErrorY = 0, mErrorZ = 0, mPercentageX = 0, mPercentageY = 0, mPercentageZ = 0;
        float mStepSizeX = 0.0, mStepSizeY = 0.0, mStepSizeZ = 0.0;

        std::vector<float> mAnalysisRef, mAnalysisData, mAnalysisKey, mAnalysisKeyTs; 
        int mRefToSave = 0; 
        bool mFinishAnalysisX = false, mFinishAnalysisY = false, mFinishAnalysisZ = false;

        struct PIDParams{
            float kp, ki, kd, sat, wind;
        };

        struct TripleFloat{
            float f1, f2, f3;
        };

        fastcom::Subscriber<TripleFloat> *mEstimateSubscriber, *mReferenceSubscriber;
        fastcom::Subscriber<PIDParams> *mSubscriberPidX, *mSubscriberPidY, *mSubscriberPidZ;
        std::unordered_map<std::string, ParamWidget*> mWidgetsParams;
    };

#endif
