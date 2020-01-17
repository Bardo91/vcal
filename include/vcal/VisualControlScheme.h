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
#include <mico/base/StereoCamera.h>

#ifdef HAS_ROS
    #include <ros/ros.h>
    #include <image_transport/image_transport.h>
    #include <cv_bridge/cv_bridge.h>
#endif

#ifdef HAS_FASTCOM
    #include <fastcom/fastcom.h>
#endif

#include <vcal/PID.h>

namespace vcal{
    class VisualControlScheme{
    public:
        /// Public types
        enum class eModules {PID, VISUALIZATION, REFERENCE};
        enum class eComTypes {ROS, FASTCOM, NONE};
        enum class eCamerasType {REALSENSE, KINECT, MONOCULAR, DATASET, NONE};

        enum class ePID {X, Y, Z};
        enum class ePIDParam {KP, KI, KD, SAT, WINDUP};

        struct ControlSignal{
            float ux, uy, uz;
        };

        /// Creates a visual control scheme that creates a 3D control reference. 
        /// \params _imageCallback: Any function that computes a 3D reference for the control system from an input rgb image
        VisualControlScheme(std::function<Eigen::Vector3f(const cv::Mat &)> &_imageCallback);

        /// Creates a visual control scheme that creates a 3D control reference. 
        /// \params _imageCallback: Any function that computes a 3D reference for the control system from an input rgb image and a depth image
        VisualControlScheme(std::function<Eigen::Vector3f(const cv::Mat &, const cv::Mat &)> &_imageCallback);

        /// Enable/disable debugging information
        void verbose(bool _true);

        /// Set PID parameters
        /// \params _pid: pid to be tuned: x, y, or z
        /// \params _param: param to be tuned: 
        /// \params _value: value
        void paramsPID(ePID _pid, ePIDParam _param, float _value);

        /// Basic destructor. Stops the pipe and kill all threads.
        ~VisualControlScheme();

        /// Configure visual control scheme interfaces.
        /// \params _module: type of module to be configured (PID, visualization or input reference)
        /// \params _comType: communication channel (ROS or fastcom)
        /// \params _params: specific configuration to be applied. It depends on the module to be configured
        ///
        /// ### PARAMETERS LIST
        /// *ROS INTERFACE:
        ///     * PID:
        ///         * "output_topic":"cmd_vel"              Topic where to publish result of PID
        ///         * "param_topic_out":"uav_1_pid_out"     Topic where the params are published in a list of floats. This will be used as base for each X Y Z pid
        ///         * "param_topic_in":"uav_1_pid_in"       Topic where the params are subscribed in a list of floats. This will be used as base for each X Y Z pid
        ///     * VISUALIZATION:
        ///         * "stream_topic":"topic_name"       Optionally, enable streaming of image
        ///     * REFERENCE:
        ///         * "reference_topic":"topic_name"            Mandatory, topic where to read the 3D reference for the PID
        ///         * "estimation_topic":"topic_name"       Mandatory, topic where to publish the 3D result of the vision algorithm
        /// *fastcom INTERFACE:
        ///     * PID:
        ///         * "output_topic":"8888"                 Port numbers where to publish result of PID
        ///         * "param_topic_out":"8889:8890::8891"   Port numbers where the params are published in a list of floats, one port per X, Y, Z
        ///         * "param_topic_in":"8892:8893::8894"    Port numbers where the params are subscribed in a list of floats, one port per X, Y, Z
        ///     * VISUALIZATION:
        ///         * "stream_topic":"8891"         Optionally, enable port number where to stream the input image
        ///     * REFERENCE:
        ///         * "reference_topic":"8892"                  Mandatory, port number where to read the 3D reference for the PID 
        ///         * "estimation_topic":"topic_name"       Mandatory, port number where to publish the 3D result of the vision algorithm
        ///
        bool configureInterface(    const eModules _module, 
                                    const eComTypes _comType, 
                                    std::unordered_map<std::string, std::string> _params);

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
        bool configureImageStream(const eCamerasType _cameraType, const std::unordered_map<std::string, std::string> _params);

        /// Start/resume processing data
        bool startPipe();

        /// Stop/pause procesingdata
        bool stopPipe();

    private:
        void initLogFile();
        void registerLog(const std::string & _tag, const std::string &_register);

        bool configureCamera();
        bool configureCameraKinect();
        bool configureCameraRealsense();
        bool configureCameraMonocular();
        bool configureCameraDataset();

        bool configureInterfacePID              (const eComTypes _comType, std::unordered_map<std::string, std::string> _params);
        bool configureInterfaceVisualization    (const eComTypes _comType, std::unordered_map<std::string, std::string> _params);
        bool configureInterfaceRefence          (const eComTypes _comType, std::unordered_map<std::string, std::string> _params);
        
        void VisualControlLoop();

    private:
        bool verbose_ = false;

        mico::StereoCamera *camera_ = nullptr;
        eCamerasType camera_Type = eCamerasType::NONE;
        std::unordered_map<std::string, std::string> cameraParams_;
        bool hasDepth_ = false;

        std::function<Eigen::Vector3f(const cv::Mat &)> callbackMonocular_;
        std::function<Eigen::Vector3f(const cv::Mat &, const cv::Mat &)> callbackStereo_;

        std::ofstream mLogFile;
        std::ofstream mEstimateFile, mControlFile;
        std::mutex mLockFile;
        
        #ifdef HAS_ROS
            ros::NodeHandle             nh_;
            ros::Publisher              rosPubEstimation_;
            ros::Publisher              rosPubPIDOut_;
            ros::Subscriber             rosSubPIDRef_;
            image_transport::Publisher  rosPubImageStream_;
        #endif

        #ifdef HAS_FASTCOM
            fastcom::Publisher<ControlSignal>       *fastcomPubEstimation_ = nullptr;
            fastcom::Publisher<ControlSignal>       *fastcomPubPIDOut_ = nullptr;
            fastcom::Subscriber<ControlSignal>      *fastcomSubPIDRef_ = nullptr;
            fastcom::ImagePublisher                 *fastcomPubImageStream_ = nullptr;

        #endif

        std::thread loopThread_;
        bool run_ = false;

        PID *controllerX_;
        PID *controllerY_;
        PID *controllerZ_;

        // some ituls
        const std::string cTextRed		= "\033[31m";
        const std::string cTextYellow	= "\033[33m";
        const std::string cTextBlue	    = "\033[34m";
        const std::string cTextGreen	= "\033[32m";
        const std::string cTextReset	= "\033[0m";
    };
}


#endif