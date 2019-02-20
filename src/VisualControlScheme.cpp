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
                callbackMonocular_ = _imageCallback;
                #ifdef HAS_ROS
                        if(!ros::isInitialized()){
                                if(verbose_)
                                        std::cout << cTextRed << "PLEASE INITIALIZE ROS IN YOUR MAIN APPLICATION" << cTextReset <<std::endl;
                                assert(false);
                        }
                #endif
                controllerX_ = new PID(0.2,0,0,-0.3, 0.3, -10,10);
                controllerY_ = new PID(0.2,0,0,-0.3, 0.3, -10,10);
                controllerZ_ = new PID(0.3,0,0,-0.3, 0.3, -10,10);
        }

        //-------------------------------------------------------------------------------------------------------------
        VisualControlScheme::VisualControlScheme(std::function<Eigen::Vector3f(const cv::Mat &, const cv::Mat &)> &_imageCallback){
                initLogFile();
                callbackStereo_ = _imageCallback;
                #ifdef HAS_ROS
                        if(!ros::isInitialized()){
                                if(verbose_)
                                        std::cout << cTextRed << "PLEASE INITIALIZE ROS IN YOUR MAIN APPLICATION" << cTextReset <<std::endl;
                                assert(false);
                        }
                #endif
                controllerX_ = new PID(0.2,0,0,-0.3, 0.3, -10,10);
                controllerY_ = new PID(0.2,0,0,-0.3, 0.3, -10,10);
                controllerZ_ = new PID(0.3,0,0,-0.3, 0.3, -10,10);
        } 


        //-------------------------------------------------------------------------------------------------------------
        void VisualControlScheme::verbose(bool _true){
                verbose_ = _true;
        }

        //-------------------------------------------------------------------------------------------------------------
        void VisualControlScheme::paramsPID(ePID _pid, ePIDParam _param, float _value){
                PID *mController;
                switch (_pid) {
                        case ePID::X:
                                mController = controllerX_;
                                break;
                        case ePID::Y:
                                mController = controllerY_;
                                break;
                        case ePID::Z:
                                mController = controllerZ_;
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
                camera_Type = _cameraType;
                cameraParams_ = _params;
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::startPipe(){
                if(configureCamera()){
                        registerLog("config", "Starting pipeline");
                        if(verbose_)
                                std::cout << cTextGreen << "Starting pipeline" << cTextReset << std::endl;
                        run_ = true;
                        loopThread_ = std::thread(&VisualControlScheme::VisualControlLoop, this);

                        return true;       
                }else{
                        registerLog("config", "Can't start pipeline");
                        if(verbose_)
                                std::cout << cTextRed << "Can't start pipeline" << cTextReset << std::endl;
                        return false;
                }
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::stopPipe(){
                run_ = false;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                if(loopThread_.joinable()){
                        loopThread_.join();
                }
        }

        //-------------------------------------------------------------------------------------------------------------
        void VisualControlScheme::initLogFile(){
                using std::chrono::system_clock;
                std::time_t tt = system_clock::to_time_t (system_clock::now());
                struct std::tm * ptm = std::localtime(&tt);
                auto timeFormat = std::put_time(ptm,"%Y-%m-%d-%H-%M-%S");
                std::stringstream timeStr;
                timeStr << timeFormat;
                mLogFile.open(timeStr.str()+"_vcal_log.txt");
                mEstimateFile.open(timeStr.str()+"_vcal_log_estimate.txt");
                mControlFile.open(timeStr.str()+"_vcal_log_control.txt");
        }

        //-------------------------------------------------------------------------------------------------------------
        void VisualControlScheme::registerLog(const std::string &_tag, const  std::string &_register){
                using std::chrono::system_clock;
                auto now = system_clock::now();
                std::time_t tt = system_clock::to_time_t (now);
                struct std::tm * ptm = std::localtime(&tt);
                auto timeFormat = std::put_time(ptm,"%H-%M-%S");
                std::stringstream timeStr;
                timeStr << timeFormat;
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
                timeStr<< '.' << std::setfill('0') << std::setw(3) << ms.count();
                std::string fullRegister = ""+ timeStr.str() + " ["+ _tag + "]\t"+_register+"\n";
                mLockFile.lock();
                mLogFile << fullRegister;
                mLogFile.flush();
                mLockFile.unlock();
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureCamera(){
                if(camera_Type == eCamerasType::NONE){
                        if(verbose_)
                                std::cout << cTextRed << "Error, camera type not configured" << cTextReset << std::endl;
                        registerLog("config", "Error, camera type not configured");
                        return false;
                }else{
                        bool result = false;
                        switch(camera_Type){
                        case eCamerasType::KINECT:
                                result = configureCameraKinect();
                                hasDepth_ = true;
                                break;
                        case eCamerasType::REALSENSE:
                                result = configureCameraRealsense();
                                hasDepth_ = true;
                                break;
                        case eCamerasType::MONOCULAR:
                                result = configureCameraMonocular();
                                hasDepth_ = false;
                                break;
                        case eCamerasType::DATASET:
                                result = configureCameraDataset();
                                break;
                        }

                        if(!result){
                                if(verbose_)
                                       std::cout << cTextRed << "Failed camera configuration" << cTextReset << std::endl;
                                registerLog("config", "Failed camera configuration");
                                return false;
                        }else{
                                if(verbose_)
                                        std::cout << cTextGreen << "Finished camera configuration" << cTextReset << std::endl;
                                registerLog("config", "Finished camera configuration");
                                return true;
                        }
                }
        }
        
        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureCameraKinect(){
                camera_ = rgbd::StereoCamera::create(rgbd::StereoCamera::eModel::Kinect);
                cjson::Json config;
                return camera_ && camera_->init(config);
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureCameraRealsense(){
                camera_ = rgbd::StereoCamera::create(rgbd::StereoCamera::eModel::RealSense);
                cjson::Json config;
                config["deviceId"] = 0;
                config["syncStreams"] = true;
                config["depth"]["resolution"]=640;
                config["depth"]["fps"]=30;
                config["rgb"]["resolution"]=640;
                config["rgb"]["fps"]=30;
                config["useUncolorizedPoints"] = true;

                return camera_ && camera_->init(config);
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureCameraMonocular(){
                camera_ = rgbd::StereoCamera::create(rgbd::StereoCamera::eModel::Custom);
                cjson::Json config;
                config["device"]["type"] = "opencv";
                config["device"]["left"] = atoi(cameraParams_["device_id"].c_str());
                config["device"]["right"] = "";
                config["cloud"]["type"] = "null";

                if(callbackStereo_){
                        if(verbose_)
                                std::cout << cTextRed << "Can't use stereo callback with monocular devices" << cTextReset << std::endl;
                        registerLog("config", "Can't use stereo callback with monocular devices");
                        return false;
                }

                return camera_ && camera_->init(config);
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureCameraDataset(){
                camera_ = rgbd::StereoCamera::create(rgbd::StereoCamera::eModel::Virtual);
                cjson::Json config;
                config["input"]["pointCloud"]="";
                config["loop_dataset"] = true;

                if(cameraParams_.find("color_images")!= cameraParams_.end())
                        config["input"]["left"]= cameraParams_["color_images"];
                else
                        return false;    
                
                if(cameraParams_.find("color_images_second") != cameraParams_.end())
                        config["input"]["right"]= cameraParams_["color_images_second"];
                else
                        config["input"]["right"]="";
                
                if(cameraParams_.find("depth_images")!= cameraParams_.end())
                        config["input"]["depth"]= cameraParams_["depth_images"];
                else
                        config["input"]["depth"]="";


                hasDepth_ =     (cameraParams_.find("depth_images")!= cameraParams_.end()) || 
                                (cameraParams_.find("color_images_second") != cameraParams_.end());

                if(callbackStereo_ && !hasDepth_){
                        if(verbose_)
                                std::cout << cTextRed << "Can't use stereo callback with dataset without stereo or depth images" << cTextReset << std::endl;
                        registerLog("config", "Can't use stereo callback with dataset without stereo or depth images");
                        return false;
                }

                return camera_ && camera_->init(config);
        }

        //-------------------------------------------------------------------------------------------------------------
        bool VisualControlScheme::configureInterfacePID(const eComTypes _comType, std::unordered_map<std::string, std::string> _params){
                if(_comType == eComTypes::ROS){
                        if(_params.find("output_topic") != _params.end()){
                                if(fastcomPubPIDOut_) delete fastcomPubPIDOut_;
                                fastcomPubPIDOut_ = nullptr;
                                rosPubPIDOut_ = nh_.advertise<std_msgs::Float32MultiArray>(_params["output_topic"], 1);
                        }
                        if(_params.find("param_topic_out") != _params.end()){
                                controllerX_->enableRosPublisher(_params["param_topic_out"]+"/X");
                                controllerY_->enableRosPublisher(_params["param_topic_out"]+"/Y");
                                controllerZ_->enableRosPublisher(_params["param_topic_out"]+"/Z");
                        }
                        if(_params.find("param_topic_in") != _params.end()){
                                controllerX_->enableRosSubscriber(_params["param_topic_in"]+"/X");
                                controllerY_->enableRosSubscriber(_params["param_topic_in"]+"/Y");
                                controllerZ_->enableRosSubscriber(_params["param_topic_in"]+"/Z");
                        }
                }else if(_comType == eComTypes::FASTCOM){
                        if(_params.find("output_topic") != _params.end()){
                                if(rosPubPIDOut_) rosPubPIDOut_ = ros::Publisher();
                                fastcomPubPIDOut_ = new fastcom::Publisher<ControlSignal>(atoi(_params["output_topic"].c_str()));
                        }
                        if(_params.find("param_topic_out") != _params.end()){
                                auto ports = splitString(_params["param_topic_out"], ':');
                                controllerX_->enableFastcomPublisher(atoi(ports[0].c_str()));
                                controllerY_->enableFastcomPublisher(atoi(ports[1].c_str()));
                                controllerZ_->enableFastcomPublisher(atoi(ports[2].c_str()));
                        }
                        if(_params.find("param_topic_in") != _params.end()){
                                auto ports = splitString(_params["param_topic_in"], ':');
                                controllerX_->enableFastcomSubscriber(atoi(ports[0].c_str()));
                                controllerY_->enableFastcomSubscriber(atoi(ports[1].c_str()));
                                controllerZ_->enableFastcomSubscriber(atoi(ports[2].c_str()));
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
                                if(fastcomPubImageStream_) delete fastcomPubImageStream_;
                                fastcomPubPIDOut_ = nullptr;
                                image_transport::ImageTransport it(nh_);
                                rosPubImageStream_ = it.advertise(_params["stream_topic"], 1);
                        }
                }else if(_comType == eComTypes::FASTCOM){
                        if(_params.find("stream_topic") != _params.end()){
                                if(rosPubImageStream_) rosPubImageStream_ = image_transport::Publisher();
                                fastcomPubImageStream_ = new fastcom::ImagePublisher(atoi(_params["stream_topic"].c_str()));
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
                                if(fastcomSubPIDRef_) delete fastcomSubPIDRef_;
                                fastcomSubPIDRef_ = nullptr;

                                rosSubPIDRef_ = nh_.subscribe<std_msgs::Float32MultiArray>(_params["reference_topic"], 1,[&](const std_msgs::Float32MultiArray::ConstPtr &_msg){
					if(fabs(controllerX_->reference() - _msg->data[0])> 0.05){
                                                controllerX_->reference(_msg->data[0]);
                                        }
                                        if(fabs(controllerY_->reference() - _msg->data[1])> 0.05){
                                                controllerY_->reference(_msg->data[1]);
                                        }
                                        if(fabs(controllerZ_->reference() - _msg->data[2])> 0.05){
                                                controllerZ_->reference(_msg->data[2]);
                                        }

                                        registerLog("visual_reference",std::to_string(_msg->data[0])+", "+std::to_string(_msg->data[1])+", "+std::to_string(_msg->data[2]));                       
                                });
                        }if(_params.find("estimation_topic") != _params.end()){
                                if(fastcomPubEstimation_) delete fastcomPubEstimation_;
                                fastcomPubEstimation_ = nullptr;
                                rosPubEstimation_ = nh_.advertise<std_msgs::Float32MultiArray>(_params["estimation_topic"], 1);
                        }else{
                                return false;
                        }
                }else if(_comType == eComTypes::FASTCOM){
                        if(_params.find("reference_topic") != _params.end()){
                                if(rosSubPIDRef_) rosSubPIDRef_ = ros::Subscriber();
                                fastcomSubPIDRef_ = new fastcom::Subscriber<ControlSignal>(atoi(_params["reference_topic"].c_str()));
                                fastcomSubPIDRef_->attachCallback([&](const ControlSignal &_reference){
                                        if(fabs(controllerX_->reference() - _reference.ux)> 0.05){
                                                controllerX_->reference(_reference.ux);
                                        }
                                        if(fabs(controllerY_->reference() - _reference.uy)> 0.05){
                                                controllerY_->reference(_reference.uy);
                                        }
                                        if(fabs(controllerZ_->reference() - _reference.uz)> 0.05){
                                                controllerZ_->reference(_reference.uz);
                                        }
                                });
                        }if(_params.find("estimation_topic") != _params.end()){
                                if(rosPubEstimation_) rosPubEstimation_ = ros::Publisher();
                                fastcomPubEstimation_ = new fastcom::Publisher<ControlSignal>(atoi(_params["estimation_topic"].c_str()));
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
                while(run_){
                        auto t1 = std::chrono::system_clock::now();
                        camera_->grab();
                        registerLog("image", "captured image");
                        cv::Mat leftImage, rightImage, depthImage;
                        camera_->rgb(leftImage, rightImage);
                        Eigen::Vector3f estimate;
                        if(hasDepth_){ // 666 assuming RGBD camera not stereo
				if(callbackStereo_){
		                        camera_->depth(depthImage);
		                        estimate = callbackStereo_(leftImage, depthImage);
				}else{
                                	estimate = callbackMonocular_(leftImage);
				}
                        }else{
                                estimate = callbackMonocular_(leftImage);
                        }
                        registerLog("visual_estimate",std::to_string(estimate[0])+", "+std::to_string(estimate[1])+", "+std::to_string(estimate[2]));
                        mEstimateFile << std::to_string(estimate[0])+", "+std::to_string(estimate[1])+", "+std::to_string(estimate[2]) << std::endl; 
			if(std::isnan(estimate[0]))
				continue;
			
                        //std::cout << estimate.transpose() <<std::endl;

                        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count()/1000.0;
                        t0 = t1;
                        float uX = controllerX_->update(estimate[0],diff);
                        float uY = controllerY_->update(estimate[1],diff);
                        float uZ = controllerZ_->update(estimate[2],diff);
                        registerLog("visual_control",std::to_string(uX)+", "+std::to_string(uY)+", "+std::to_string(uZ));
                        mControlFile << std::to_string(uX)+", "+std::to_string(uY)+", "+std::to_string(uZ) << std::endl;
                        
                        #ifdef HAS_FASTCOM
                                if(fastcomPubPIDOut_){
                                        ControlSignal signal = {uX, uY, uZ};
                                        fastcomPubPIDOut_->publish(signal);
                                }
                                if(fastcomPubImageStream_){
                                        fastcomPubImageStream_->publish(leftImage);
                                }
                                if(fastcomPubEstimation_) {
                                        ControlSignal estimation = {estimate[0], estimate[1], estimate[2]};
                                        fastcomPubEstimation_->publish(estimation);
                                }
                        #endif
                        #ifdef HAS_ROS
                                if(rosPubPIDOut_){
                                        std_msgs::Float32MultiArray msg;
                                        msg.data = {uX, uY, uZ};
                                        rosPubPIDOut_.publish(msg);
                                }
                                if(rosPubImageStream_){
                                        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", leftImage).toImageMsg();
                                        rosPubImageStream_.publish(msg);
                                }
                                if(rosPubEstimation_){
                                        std_msgs::Float32MultiArray msg;
                                        msg.data = {estimate[0], estimate[1], estimate[2]};
                                        rosPubEstimation_.publish(msg);
                                }
                        #endif
                }
        }


}
