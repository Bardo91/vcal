//---------------------------------------------------------------------------------------------------------------------
//  VCAL - Visual Control Abstraction Layer
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 ViGUS University of Seville
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
//  and associated documentation files (the "Software"), to deal in the Software without restriction, 
//  including without limitation the rights to use, copy, modify, merge, publish, distribute, 
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial 
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES 
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#include <limits>
#include <thread>

#ifdef HAS_ROS
    #include <ros/ros.h>
    #include <std_msgs/Float32MultiArray.h>
#endif
#ifdef HAS_FASTCOM
    #include <fastcom/fastcom.h>
#endif

class PID {
public:
    struct PIDParams{
        float kp, ki, kd, sat, wind;
    };

    PID(float _kp, float _ki, float _kd,
        float _minSat = std::numeric_limits<float>::min(),
        float _maxSat = std::numeric_limits<float>::max(),
        float _minWind = std::numeric_limits<float>::min(),
        float _maxWind = std::numeric_limits<float>::max());
    
    float update(float _val, float _incT);
 
    void enableRosPublisher(std::string _topic);
    void enableRosSubscriber(std::string _topic);
    void enableFastcomPublisher(int _port);
    void enableFastcomSubscriber(int _port);

    float reference() { return mReference; }
    void reference(float _ref) { mReference = _ref; mAccumErr = 0; mLastError = 0; mLastResult = 0; mBouncingFactor = 0.1;}
 
    float kp() const { return mKp; }
    float ki() const { return mKi; }
    float kd() const { return mKd; }
 
    void kp(float _kp) { mKp = _kp; }
    void ki(float _ki) { mKi = _ki; }
    void kd(float _kd) { mKd = _kd; }
 
    void setSaturations(float _min, float _max) { mMinSat = _min; mMaxSat = _max; }
    void getSaturations(float _min, float _max) { _min = mMinSat; _max = mMaxSat; }
 
    void setWindupTerms(float _min, float _max) { mWindupMin = _min; mWindupMax = _max; }
    void getWindupTerms(float _min, float _max) { _min = mWindupMin; _max = mWindupMax; }
 
private:
    #ifdef HAS_ROS
    void rosSubCallback(const std_msgs::Float32MultiArray::ConstPtr &_msg);
    #endif

private:
    float mReference;
    float mKp, mKi, mKd;
    float mMinSat, mMaxSat;
    float mWindupMin, mWindupMax;
    float mLastResult, mLastError, mAccumErr;
    double mBouncingFactor = 0.1;
    bool run_ = false;
    std::thread mParamPubThread;
    int mPort;
    int ignoreCounter = 0;
    #ifdef HAS_ROS
        ros::NodeHandle nh_;
        ros::Publisher mRosPubParams;
        ros::Subscriber mRosSubParams;
    #endif

    #ifdef HAS_FASTCOM
        fastcom::Publisher<PIDParams> *mFastcomPubParams = nullptr;
        fastcom::Subscriber<PIDParams> *mFastcomSubParams = nullptr;
    #endif
};
