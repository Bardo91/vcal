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

#include <vcal/PID.h>
#include <algorithm>
#include <thread>
#include <chrono>

#ifdef HAS_ROS
    #include <ros/ros.h>
    #include <std_msgs/Float32MultiArray.h>
#endif

PID::PID(float _kp, float _ki, float _kd, float _minSat, float _maxSat, float _minWind, float _maxWind) {
    mKp = _kp;
    mKi = _ki;
    mKd = _kd;
    mMinSat = _minSat;
    mMaxSat = _maxSat;
    mWindupMin = _minWind;
    mWindupMax = _maxWind;
}

float PID::update(float _val, float _incT) {
    float dt = _incT; // 666 input arg?

    float err = mReference - _val;
    mAccumErr += err * dt;
    // Apply anti wind-up 777 Analyze other options
    mAccumErr = std::min(std::max(mAccumErr, mWindupMin), mWindupMax);

    // Compute PID
    mLastResult = mKp * err + mKi * mAccumErr + mKd * (err - mLastError) / dt;
    mLastError = err;

    // Saturate signal
    mLastResult = std::min(std::max(mLastResult, mMinSat), mMaxSat);
    mLastResult *= mBouncingFactor;
    mBouncingFactor *= 2.0;
    mBouncingFactor = mBouncingFactor > 1.0 ? 1.0 : mBouncingFactor;
    return mLastResult;
}

//------------------------------------------------------------------------------------------------------------------------------
void PID::enableRosPublisher(std::string _topic){
    #ifdef HAS_ROS
        mRosPubParams = nh_.advertise<std_msgs::Float32MultiArray>(_topic, 1);
        run_ = false;
        if(mParamPubThread.joinable())
            mParamPubThread.join();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        run_ = true;
        mParamPubThread = std::thread([&](){
            while(run_){
                std_msgs::Float32MultiArray data;
                data.data = {mKp, mKi, mKd, mMaxSat, mWindupMax};
                mRosPubParams.publish(data);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        });
    #endif
}

//------------------------------------------------------------------------------------------------------------------------------
void PID::enableRosSubscriber(std::string _topic){
    #ifdef HAS_ROS
        mRosSubParams = nh_.subscribe<std_msgs::Float32MultiArray>(_topic, 1, &PID::rosSubCallback, this);
    #endif
}

//------------------------------------------------------------------------------------------------------------------------------
void PID::enableFastcomPublisher(int _port){
    #ifdef HAS_FASTCOM
        if(mFastcomPubParams) delete mFastcomPubParams;
        mFastcomPubParams = new fastcom::Publisher<PIDParams>(_port);
        run_ = false;
        if(mParamPubThread.joinable())
            mParamPubThread.join();
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        run_ = true;
        mParamPubThread = std::thread([&](){
            while(run_){
                PIDParams params = {mKp, mKi, mKd, mMaxSat, mWindupMax};
                mFastcomPubParams->publish(params);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        });
    #endif
}

//------------------------------------------------------------------------------------------------------------------------------
void PID::enableFastcomSubscriber(int _port){
    #ifdef HAS_FASTCOM
        if(mFastcomSubParams) delete mFastcomSubParams;
        mFastcomSubParams  = new fastcom::Subscriber<PIDParams>("10.0.0.111", _port);
std::cout << "trying to subscribe" << std::endl;
        mPort = _port;
        auto callback = [this](const PIDParams &_params){
ignoreCounter++;
if(ignoreCounter < 10)
	return;
                mKp = _params.kp;
                mKi = _params.ki;
                mKd = _params.kd;
                mMinSat = -_params.sat;
                mMaxSat =  _params.sat;
                mWindupMin = -_params.wind;
                mWindupMax =  _params.wind;
            };
        mFastcomSubParams->attachCallback(callback);
    #endif
}


#ifdef HAS_ROS
void PID::rosSubCallback(const std_msgs::Float32MultiArray::ConstPtr &_msg){
    std::cout << "Received message" << std::endl;
    this->mKp = _msg->data[0];
    this->mKi = _msg->data[1];
    this->mKd = _msg->data[2];
    this->mMinSat = -_msg->data[3];
    this->mMaxSat = _msg->data[3];
    this->mWindupMin = -_msg->data[4];
    this->mWindupMax = _msg->data[4];
}
#endif
