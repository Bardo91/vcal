//---------------------------------------------------------------------------------------------------------------------
//  VCAL - Visual Control Abstraction Layer
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2018 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
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

#include <VisualControlScheme.h>

#include <chrono>
#include <algorithm>

namespace vcal{
        
        //-------------------------------------------------------------------------------------------------------------
        VisualControlScheme::VisualControlScheme(std::function<Eigen::Vector3f(const cv::Mat &)> &_imageCallback){
                initLogFile();
                mCallbackMonocular = _imageCallback;
        }

        //-------------------------------------------------------------------------------------------------------------
        VisualControlScheme::VisualControlScheme(std::function<Eigen::Vector3f(const cv::Mat &, const cv::Mat &)> &_imageCallback){
                initLogFile();
                mCallbackStereo = _imageCallback;
        } 

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureInterface(   const eModules _module, 
                                                        const eComTypes _comType, 
                                                        const std::unordered_map<std::string, std::string> &_names){
                
        
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureImageStream(const eCamerasType _cameraType){
                configureImageStream(_cameraType, {});
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureImageStream(const eCamerasType _cameraType, std::unordered_map<std::string, std::string> &_params){
                mCameraType = _cameraType;
                mCameraParams = _params;
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::startPipe(){
                if(checkCamera && checkCom()){
                        register("config", "Starting pipeline")
                        mRun = true;
                        mLoopThread = std::thread(&VisualControlScheme::VisualControlLoop, this);

                        return true;       
                }else{
                        register("config", "Can't start pipeline")
                        return false;
                }
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::stopPipe(){
                mRun = false;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                if(mLoopThread.joinable()){
                        mLoopThread.join();
                }
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::initLogFile(){
                auto timeNow = std::chrono::system_clock::now();
                std::string timeStr = std::put_time(std::localtime(&timeNow), "%H").str();
                mLogFile.open(timeStr+"_vcal_log.txt");
        }

        //-------------------------------------------------------------------------------------------------------------
        void VisualControlScheme::registerLog(std::string &_tag, std::string &_register){
                auto timeNow = std::chrono::system_clock::now();
                std::string timeStr = std::put_time(std::localtime(&timeNow), "%H").str();
                std::string fullRegister = "["+ timeStr + "] ["+ _tag + "]\t"+_register+"\n";
                mLogFile << fullRegister;
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::checkCamera(){
                if(mCameraType == eCamerasType::NONE){
                        std::cout << cTextRed << "Error, camera type not configured" << cTextReset << std::endl;
                        registerLog("config", "Error, camera type not configured");
                        return false;
                }else{
                        bool result = false;
                        switch(mCameraType){
                        case eCamerasType::KINECT:
                                result = checkCameraKinect();
                                mHasDepth = true;
                                break;
                        case eCamerasType::REALSENSE:
                                result = checkCameraRealsense();
                                mHasDepth = true;
                                break;
                        case eCamerasType::MONOCULAR:
                                result = checkCameraMonocular();
                                mHasDepth = false;
                                break;
                        case eCamerasType::DATASET:
                                result = checkCameraDataset();
                                break;
                        }

                        if(!result){
                                std::cout << cTextRed << "Failed camera configuration" << cTextReset << std::endl;
                                registerLog("config", "Failed camera configuration");
                                return false;
                        }else{
                                std::cout << cTextGreen << "Finished camera configuration" << cTextReset << std::endl;
                                registerLog("config", "Finished camera configuration");
                                return true;
                        }
                }
        }
        
        //-------------------------------------------------------------------------------------------------------------
        bool checkCameraKinect(){
                mCamera = rgbd::StereoCamera::create(rgbd::StereoCamera::eModel::Kinect);
                cjson::Json config;
                return mCamera && mCamera->init(config);
        }

        //-------------------------------------------------------------------------------------------------------------
        bool checkCameraRealsense(){
                mCamera = rgbd::StereoCamera::create(rgbd::StereoCamera::eModel::RealSense);
                cjson::Json config;
                config["deviceId"] = 0;
                config["syncStreams"] = true;
                config["depth"]["resolution"]=640;
                config["depth"]["fps"]=30;
                config["rgb"]["resolution"]=640;
                config["rgb"]["fps"]=30;
                config["useUncolorizedPoints"] = true;

                return mCamera && mCamera->init(config);
        }

        //-------------------------------------------------------------------------------------------------------------
        bool checkCameraMonocular(){
                mCamera = rgbd::StereoCamera::create(rgbd::StereoCamera::eModel::Custom);
                cjson::Json config;
                config["device"]["type"] = "opencv";
                config["device"]["left"] = atoi(mCameraParams["device_id"].c_str());
                config["cloud"]["type"] = "null";

                if(mCallbackStereo){
                        std::cout << cTextRed << "Can't use stereo callback with monocular devices" << cTextReset << std::endl;
                        registerLog("config", "Can't use stereo callback with monocular devices");
                        return false;
                }

                return mCamera && mCamera->init(config);
        }

        //-------------------------------------------------------------------------------------------------------------
        bool checkCameraDataset(){
                mCamera = rgbd::StereoCamera::create(rgbd::StereoCamera::eModel::Virtual);
                cjson::Json config;
                if(mCameraParams.contains("color_images"))
                        config["input"]["left"]= mCameraParams["color_images"];
                else
                        return false;    
                
                if(mCameraParams.contains("color_images_second"))
                        config["input"]["right"]= mCameraParams["color_images_second"];
                
                if(mCameraParams.contains("depth_images"))
                        config["input"]["depth"]= mCameraParams["depth_images"];

                mHasDepth = mCameraParams.contains("depth_images") || mCameraParams.contains("color_images_second");

                if(mCallbackStereo && (!mCameraParams.contains("color_images_second") || !mCameraParams.contains("depth_images"))){
                        std::cout << cTextRed << "Can't use stereo callback with dataset without stereo or depth images" << cTextReset << std::endl;
                        registerLog("config", "Can't use stereo callback with dataset without stereo or depth images");
                        return false;
                }

                return mCamera && mCamera->init(config);
        }


        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::ckeckCom(){

        }

        //-------------------------------------------------------------------------------------------------------------
        void VisualControlScheme::VisualControlLoop(){
                while(mRun){
                        mCamera->grab();
                        cv::Mat leftImage, rightImage, depthImage;
                        mCamera->rgb(leftImage, rightImage);

                        if(mHasDepth){ // 666 assuming RGBD camera not stereo
                                mCamera->depth(depthImage);
                                mCallbackStereo(leftImage, depthImage);
                        }else{
                                mCallbackMonocular(leftImage);
                        }
                }
        }


}