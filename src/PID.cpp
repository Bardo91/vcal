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

#include "PID.h"
#include <algorithm>
#include <std_msgs/Float32.h>
#include <thread>
#include <chrono>

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
