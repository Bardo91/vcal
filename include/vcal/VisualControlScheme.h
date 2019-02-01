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

#ifndef VCAL_VISUALCONTROLSCHEME_H_
#define VCAL_VISUALCONTROLSCHEME_H_

#include <functional>
#include <fstream>
#include <thread>

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <rgbd_tools/StereoCamera.h>

#ifdef HAS_ROS
    #include <ros/ros.h>
    #include <image_transport/image_transport.h>
    #include <cv_bridge/cv_bridge.h>
#endif

#ifdef HAS_FASTCOM
    #include <fastcom/fastcom.h>
#endif

namespace vcal{
    class VisualControlScheme{
    public:
        enum eModules {PID, VISUALIZATION, REFERENCE};
        enum eComTypes {ROS, FASTCOM, NONE};
        enum eCamerasType {REALSENSE, KINECT, MONOCULAR, DATASET, NONE};

        /// Creates a visual control scheme that creates a 3D control reference. 
        /// \params _imageCallback: Any function that computes a 3D reference for the control system from an input rgb image
        VisualControlScheme(std::function<Eigen::Vector3f(const cv::Mat &)> &_imageCallback);

        /// Creates a visual control scheme that creates a 3D control reference. 
        /// \params _imageCallback: Any function that computes a 3D reference for the control system from an input rgb image and a depth image
        VisualControlScheme(std::function<Eigen::Vector3f(const cv::Mat &, const cv::Mat &)> &_imageCallback);

        /// Basic destructor. Stops the pipe and kill all threads.
        ~VisualControlScheme();

        /// Configure visual control scheme interfaces.
        /// \params _module: type of module to be configured (PID, visualization or input reference)
        /// \params _comType: communication channel (ROS or fastcom)
        /// \params _names: specific configuration to be applied. It depends on the module to be configured
        ///
        /// ### PARAMETERS LIST
        /// *ROS INTERFACE:
        ///     * PID:
        ///         * "output_topic":"cmd_vel"          Topic where to publish result of PID
        ///         * "param_topic_out":"uav_1_pid"     Topic where the params are published in a list of floats
        ///         * "param_topic_in":"uav_1_pid"      Topic where the params are subscribed in a list of floats
        ///         * "k":"0.723"                       Initial proportional parameter for the PID
        ///         * "ki":"0.0001"                     Initial integral parameter for the PID
        ///         * "kd":"0.3"                        Initial derivative parameter for the PID
        ///         * "wind_up":"10"                    Initial anti windup parameter for the PID
        ///         * "saturation":"1"                  Initial saturation parameter for the PID
        ///     * VISUALIZATION:
        ///         * "stream_topic":"topic_name"       Optionally, enable streaming of image
        ///     * REFERENCE:
        ///         * "input_topic":"topic_name"        Mandatory, topic where to read the 3D reference for the PID
        /// *fastcom INTERFACE:
        ///     * PID:
        ///         * "output_topic":"8888"         Port number where to publish result of PID
        ///         * "param_topic_out":"8889"      Port number where the params are published in a list of floats
        ///         * "param_topic_in":"8890"       Port number where the params are subscribed in a list of floats
        ///         * "k":"0.723"                   Initial proportional parameter for the PID
        ///         * "ki":"0.0001"                 Initial integral parameter for the PID
        ///         * "kd":"0.3"                    Initial derivative parameter for the PID
        ///         * "wind_up":"10"                Initial anti windup parameter for the PID
        ///         * "saturation":"1"              Initial saturation parameter for the PID
        ///     * VISUALIZATION:
        ///         * "stream_topic":"8891"         Optionally, enable port number where to stream the input image
        ///     * REFERENCE:
        ///         * "input_topic":"8892"          Mandatory, port number where to read the 3D reference for the PID 
        ///
        bool configureInterface(    const eModules _module, 
                                    const eComTypes _comType, 
                                    const std::unordered_map<std::string, std::string> &_names);

        /// Basic configure input stream for the visual control scheme.
        /// \param _cameraType: choose input device
        bool configureImageStream(const eCamerasType _cameraType);

        /// Configure input stream for the visual control scheme with custom parameters.
        /// \param _cameraType: choose input device
        /// \param _params: list with parameters
        /// 
        /// ### PARAMETERS LIST
        /// * Monocular:
        ///     * "device_id":"0"       System device's id
        /// * Dataset:
        ///     * "color_images":"template_path_to_files_%d.jpg"            Template filename where the images are located and stored sequentially.
        ///     * "color_images_second":"template_path_to_files_%d.jpg"     Template filename where the images are located and stored sequentially. 
        ///                                                                 This param applies for stereo datasets.
        ///     * "depth_images":"template_path_to_files_%d.png"            Template filename where the images are located and stored sequentially.
        ///                                                                 This param applies for datasets with depth images.
        ///
        bool configureImageStream(const eCamerasType _cameraType, std::unordered_map<std::string, std::string> &_params);

        /// Start/resume processing data
        bool startPipe();

        /// Stop/pause procesingdata
        bool stopPipe();

    private:
        void initLogFile();
        void registerLog(std::string &_register);

        bool checkCamera();
        bool checkCameraKinect();
        bool checkCameraRealsense();
        bool checkCameraMonocular();
        bool checkCameraDataset();

        bool ckeckCom();

        void VisualControlLoop();

    private:
        rgbd::StereoCamera *mCamera = nullptr;
        eCamerasType mCameraType = eCamerasType::NONE;
        std::unordered_map<std::string, std::string> mCameraParams;
        bool mHasDepth = false;

        std::function<Eigen::Vector3f(const cv::Mat &)> mCallbackMonocular;
        std::function<Eigen::Vector3f(const cv::Mat &, const cv::Mat &)> mCallbackStereo;

        eComTypes mSwitchPID    = eComTypes::NONE;
        eComTypes mSwitchRef    = eComTypes::NONE;
        eComTypes mSwitchStream = eComTypes::NONE;

        std::ofstream mLogFile;

        #ifdef HAS_ROS
            ros::Subscriber             mRosSubPIDReference;
            ros::Subscriber             mRosSubPIDParams;
            ros::Publisher              mRosPubPIDParams;
            ros::Publisher              mRosPubPIDOut;
            image_transport::Publisher  mRosPubImageStream;
        #endif

        #ifdef HAS_FASTCOM
            fastcom::Subscriber     mFastComSubPIDReference;
            fastcom::Subscriber     mFastComSubPIDParams;
            fastcom::Publisher      mFastComPubPIDParams;
            fastcom::Publisher      mFastComPubPIDOut;
            fastcom::ImagePublisher mFastComPubImageStream;
        #endif

        std::thread mLoopThread;
        bool mRun = false;

        // some ituls
        const std::string::cTextRed		= "\033[31m";
        const std::string::cTextYellow	= "\033[33m";
        const std::string::cTextBlue	= "\033[34m";
        const std::string::cTextGreen	= "\033[32m";
        const std::string::cTextReset	= "\033[0m";
    }    
}


#endif