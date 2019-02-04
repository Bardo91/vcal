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

#include <vcal/VisualControlScheme.h>

#include <chrono>
#include <algorithm>

#ifdef HAS_ROS
    #include <std_msgs/Float32MultiArray.h>
#endif

std::vector<std::string> splitString(const std::string& s, char delimiter) {
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter)) {
      tokens.push_back(token);
   }
   return tokens;
}


namespace vcal{
        //-------------------------------------------------------------------------------------------------------------
        VisualControlScheme::VisualControlScheme(std::function<Eigen::Vector3f(const cv::Mat &)> &_imageCallback){
                initLogFile();
                mCallbackMonocular = _imageCallback;
                #ifdef HAS_ROS
                        if(!ros::isInitialized()){
                                std::cout << cTextRed << "PLEASE INITIALIZE ROS IN YOUR MAIN APPLICATION" << cTextReset <<std::endl;
                                assert(false);
                        }
                #endif
                mControllerX = new PID(0,0,0);
                mControllerY = new PID(0,0,0);
                mControllerZ = new PID(0,0,0);
        }

        //-------------------------------------------------------------------------------------------------------------
        VisualControlScheme::VisualControlScheme(std::function<Eigen::Vector3f(const cv::Mat &, const cv::Mat &)> &_imageCallback){
                initLogFile();
                mCallbackStereo = _imageCallback;
                #ifdef HAS_ROS
                        if(!ros::isInitialized()){
                                std::cout << cTextRed << "PLEASE INITIALIZE ROS IN YOUR MAIN APPLICATION" << cTextReset <<std::endl;
                                assert(false);
                        }
                #endif
                mControllerX = new PID(0,0,0);
                mControllerY = new PID(0,0,0);
                mControllerZ = new PID(0,0,0);
        } 

        //-------------------------------------------------------------------------------------------------------------
        void VisualControlScheme::paramsPID(ePID _pid, ePIDParam _param, float _value){
                PID *mController;
                switch (_pid) {
                        case ePID::X:
                                mController = mControllerX;
                                break;
                        case ePID::Y:
                                mController = mControllerY;
                                break;
                        case ePID::Z:
                                mController = mControllerZ;
                                break;
                }

                 switch (_param) {
                        case ePIDParam::KP:
                                mController->kp(_value);
                                break;
                        case ePIDParam::KI:
                                mController->ki(_value);
                                break;
                        case ePIDParam::KD:
                                mController->kd(_value);
                                break;
                        case ePIDParam::SAT:
                                mController->setSaturations(-_value,_value);
                                break;
                        case ePIDParam::WINDUP:
                                mController->setWindupTerms(-_value,_value);
                                break;
                }
        }

        //-------------------------------------------------------------------------------------------------------------
        VisualControlScheme::~VisualControlScheme(){
                stopPipe();
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureInterface(   const eModules _module, 
                                                        const eComTypes _comType, 
                                                        std::unordered_map<std::string, std::string> _params){
                
                switch (_module) {
                        case eModules::PID:
                                return configureInterfacePID(_comType, _params);
                                break;
                        case eModules::REFERENCE:
                                return configureInterfaceRefence(_comType, _params);
                                break;
                        case eModules::VISUALIZATION:
                                return configureInterfaceVisualization(_comType, _params);
                                break;
                        default:
                                return false;
                }
        
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureImageStream(const eCamerasType _cameraType){
                configureImageStream(_cameraType, {});
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureImageStream(const eCamerasType _cameraType, const std::unordered_map<std::string, std::string> _params){
                mCameraType = _cameraType;
                mCameraParams = _params;
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::startPipe(){
                if(configureCamera()){
                        registerLog("config", "Starting pipeline");
                        std::cout << cTextGreen << "Starting pipeline" << cTextReset << std::endl;
                        mRun = true;
                        mLoopThread = std::thread(&VisualControlScheme::VisualControlLoop, this);

                        return true;       
                }else{
                        registerLog("config", "Can't start pipeline");
                        std::cout << cTextRed << "Can't start pipeline" << cTextReset << std::endl;
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
        void VisualControlScheme::initLogFile(){
                using std::chrono::system_clock;
                std::time_t tt = system_clock::to_time_t (system_clock::now());
                struct std::tm * ptm = std::localtime(&tt);
                auto timeFormat = std::put_time(ptm,"%H-%M-%S");
                std::stringstream timeStr;
                timeStr << timeFormat;
                mLogFile.open(timeStr.str()+"_vcal_log.txt");
        }

        //-------------------------------------------------------------------------------------------------------------
        void VisualControlScheme::registerLog(const std::string &_tag, const  std::string &_register){
                using std::chrono::system_clock;
                std::time_t tt = system_clock::to_time_t (system_clock::now());
                struct std::tm * ptm = std::localtime(&tt);
                auto timeFormat = std::put_time(ptm,"%H-%M-%S");
                std::stringstream timeStr;
                timeStr << timeFormat;
                std::string fullRegister = "["+ timeStr.str() + "] ["+ _tag + "]\t"+_register+"\n";
                mLogFile << fullRegister;
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureCamera(){
                if(mCameraType == eCamerasType::NONE){
                        std::cout << cTextRed << "Error, camera type not configured" << cTextReset << std::endl;
                        registerLog("config", "Error, camera type not configured");
                        return false;
                }else{
                        bool result = false;
                        switch(mCameraType){
                        case eCamerasType::KINECT:
                                result = configureCameraKinect();
                                mHasDepth = true;
                                break;
                        case eCamerasType::REALSENSE:
                                result = configureCameraRealsense();
                                mHasDepth = true;
                                break;
                        case eCamerasType::MONOCULAR:
                                result = configureCameraMonocular();
                                mHasDepth = false;
                                break;
                        case eCamerasType::DATASET:
                                result = configureCameraDataset();
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
        bool VisualControlScheme::configureCameraKinect(){
                mCamera = rgbd::StereoCamera::create(rgbd::StereoCamera::eModel::Kinect);
                cjson::Json config;
                return mCamera && mCamera->init(config);
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureCameraRealsense(){
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
        bool VisualControlScheme::configureCameraMonocular(){
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
        bool VisualControlScheme::configureCameraDataset(){
                mCamera = rgbd::StereoCamera::create(rgbd::StereoCamera::eModel::Virtual);
                cjson::Json config;
                config["input"]["pointCloud"]="";
                config["loop_dataset"] = true;

                if(mCameraParams.find("color_images")!= mCameraParams.end())
                        config["input"]["left"]= mCameraParams["color_images"];
                else
                        return false;    
                
                if(mCameraParams.find("color_images_second") != mCameraParams.end())
                        config["input"]["right"]= mCameraParams["color_images_second"];
                else
                        config["input"]["right"]="";
                
                if(mCameraParams.find("depth_images")!= mCameraParams.end())
                        config["input"]["depth"]= mCameraParams["depth_images"];
                else
                        config["input"]["depth"]="";


                mHasDepth =     (mCameraParams.find("depth_images")!= mCameraParams.end()) || 
                                (mCameraParams.find("color_images_second") != mCameraParams.end());

                if(mCallbackStereo && !mHasDepth){
                        std::cout << cTextRed << "Can't use stereo callback with dataset without stereo or depth images" << cTextReset << std::endl;
                        registerLog("config", "Can't use stereo callback with dataset without stereo or depth images");
                        return false;
                }

                return mCamera && mCamera->init(config);
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureInterfacePID(const eComTypes _comType, std::unordered_map<std::string, std::string> _params){
                if(_comType == eComTypes::ROS){
                        if(_params.find("output_topic") != _params.end()){
                                if(mFastComPubPIDOut) delete mFastComPubPIDOut;
                                mFastComPubPIDOut = nullptr;
                                mRosPubPIDOut = mNH.advertise<std_msgs::Float32MultiArray>(_params["output_topic"], 1);
                        }
                        if(_params.find("param_topic_out") != _params.end()){
                                mControllerX->enableRosPublisher(_params["param_topic_out"]+"/X");
                                mControllerY->enableRosPublisher(_params["param_topic_out"]+"/Y");
                                mControllerZ->enableRosPublisher(_params["param_topic_out"]+"/Z");
                        }
                        if(_params.find("param_topic_in") != _params.end()){
                                mControllerX->enableRosSubscriber(_params["param_topic_in"]+"/X");
                                mControllerY->enableRosSubscriber(_params["param_topic_in"]+"/Y");
                                mControllerZ->enableRosSubscriber(_params["param_topic_in"]+"/Z");
                        }
                }else if(_comType == eComTypes::FASTCOM){
                        if(_params.find("output_topic") != _params.end()){
                                if(mRosPubPIDOut) mRosPubPIDOut = ros::Publisher();
                                mFastComPubPIDOut = new fastcom::Publisher<ControlSignal>(atoi(_params["output_topic"].c_str()));
                        }
                        if(_params.find("param_topic_out") != _params.end()){
                                auto ports = splitString(_params["param_topic_out"], ':');
                                mControllerX->enableFastcomPublisher(atoi(ports[0].c_str()));
                                mControllerY->enableFastcomPublisher(atoi(ports[1].c_str()));
                                mControllerZ->enableFastcomPublisher(atoi(ports[2].c_str()));
                        }
                        if(_params.find("param_topic_in") != _params.end()){
                                auto ports = splitString(_params["param_topic_in"], ':');
                                mControllerX->enableFastcomSubscriber(atoi(ports[0].c_str()));
                                mControllerY->enableFastcomSubscriber(atoi(ports[1].c_str()));
                                mControllerZ->enableFastcomSubscriber(atoi(ports[2].c_str()));
                        }
                }else{
                        return false;
                }
                return true;
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureInterfaceVisualization(const eComTypes _comType,  std::unordered_map<std::string, std::string> _params){
                if(_comType == eComTypes::ROS){
                        if(_params.find("stream_topic") != _params.end()){
                                if(mFastComPubImageStream) delete mFastComPubImageStream;
                                mFastComPubPIDOut = nullptr;
                                image_transport::ImageTransport it(mNH);
                                mRosPubImageStream = it.advertise(_params["stream_topic"], 1);
                        }
                }else if(_comType == eComTypes::FASTCOM){
                        if(_params.find("stream_topic") != _params.end()){
                                if(mRosPubImageStream) mRosPubImageStream = image_transport::Publisher();
                                mFastComPubImageStream = new fastcom::ImagePublisher(atoi(_params["stream_topic"].c_str()));
                        }
                }else{
                        return false;
                }
                return true;
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureInterfaceRefence(const eComTypes _comType,  std::unordered_map<std::string, std::string> _params){
                if(_comType == eComTypes::ROS){
                        if(_params.find("reference_topic") != _params.end()){
                                if(mFastcomSubPIDRef) delete mFastcomSubPIDRef;
                                mFastcomSubPIDRef = nullptr;
                                mRosSubPIDRef = mNH.subscribe<std_msgs::Float32MultiArray>(_params["reference_topic"], 1,[&](const std_msgs::Float32MultiArray::ConstPtr &_msg){
                                        if(fabs(mControllerX->reference() - _msg->data[0])> 0.05){
                                                mControllerX->reference(_msg->data[0]);
                                        }
                                        if(fabs(mControllerY->reference() - _msg->data[1])> 0.05){
                                                mControllerY->reference(_msg->data[1]);
                                        }
                                        if(fabs(mControllerZ->reference() - _msg->data[2])> 0.05){
                                                mControllerZ->reference(_msg->data[2]);
                                        }
                                });
                        }if(_params.find("estimation_topic") != _params.end()){
                                if(mFastComPubEstimation) delete mFastComPubEstimation;
                                mFastComPubEstimation = nullptr;
                                mRosPubEstimation = mNH.advertise<std_msgs::Float32MultiArray>(_params["estimation_topic"], 1);
                        }else{
                                return false;
                        }
                }else if(_comType == eComTypes::FASTCOM){
                        if(_params.find("reference_topic") != _params.end()){
                                if(mRosSubPIDRef) mRosSubPIDRef = ros::Subscriber();
                                mFastcomSubPIDRef = new fastcom::Subscriber<ControlSignal>(atoi(_params["reference_topic"].c_str()));
                                mFastcomSubPIDRef->attachCallback([&](const ControlSignal &_reference){
                                        if(fabs(mControllerX->reference() - _reference.ux)> 0.05){
                                                mControllerX->reference(_reference.ux);
                                        }
                                        if(fabs(mControllerY->reference() - _reference.uy)> 0.05){
                                                mControllerY->reference(_reference.uy);
                                        }
                                        if(fabs(mControllerZ->reference() - _reference.uz)> 0.05){
                                                mControllerZ->reference(_reference.uz);
                                        }
                                });
                        }if(_params.find("estimation_topic") != _params.end()){
                                if(mRosPubEstimation) mRosPubEstimation = ros::Publisher();
                                mFastComPubEstimation = new fastcom::Publisher<ControlSignal>(atoi(_params["estimation_topic"].c_str()));
                        }else {
                                return false;
                        }
                        
                }else{
                        return false;
                }
                return true;
        }

        //-------------------------------------------------------------------------------------------------------------
        void VisualControlScheme::VisualControlLoop(){
                auto t0 = std::chrono::system_clock::now();
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
                while(mRun){
                        auto t1 = std::chrono::system_clock::now();
                        mCamera->grab();
                        cv::Mat leftImage, rightImage, depthImage;
                        mCamera->rgb(leftImage, rightImage);
                        Eigen::Vector3f estimate;
                        if(mHasDepth){ // 666 assuming RGBD camera not stereo
                                mCamera->depth(depthImage);
                                estimate = mCallbackStereo(leftImage, depthImage);
                        }else{
                                estimate = mCallbackMonocular(leftImage);
                        }

                        //std::cout << estimate.transpose() <<std::endl;
                        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count()/1000.0;
                        t0 = t1;
                        float uX = mControllerX->update(estimate[0],diff);
                        float uY = mControllerY->update(estimate[1],diff);
                        float uZ = mControllerZ->update(estimate[2],diff);
                        #ifdef HAS_FASTCOM
                                if(mFastComPubPIDOut){
                                        ControlSignal signal = {uX, uY, uZ};
                                        mFastComPubPIDOut->publish(signal);
                                }
                                if(mFastComPubImageStream){
                                        mFastComPubImageStream->publish(leftImage);
                                }
                                if(mFastComPubEstimation) {
                                        ControlSignal estimation = {estimate[0], estimate[1], estimate[2]};
                                        mFastComPubEstimation->publish(estimation);
                                }
                        #endif
                        #ifdef HAS_ROS
                                if(mRosPubPIDOut){
                                        std_msgs::Float32MultiArray msg;
                                        msg.data = {uX, uY, uZ};
                                        mRosPubPIDOut.publish(msg);
                                }
                                if(mRosPubImageStream){
                                        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", leftImage).toImageMsg();
                                        mRosPubImageStream.publish(msg);
                                }
                                if(mRosPubEstimation){
                                        std_msgs::Float32MultiArray msg;
                                        msg.data = {estimate[0], estimate[1], estimate[2]};
                                        mRosPubEstimation.publish(msg);
                                }
                        #endif
                }
        }


}