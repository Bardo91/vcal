//
//
//
//
//
//

#include <QApplication>
#include "PidTuneGui.h"
#include <ros/ros.h>
#include <thread>

int main(int _argc, char **_argv){
    ros::init(_argc, _argv, "pid_tune_gui");
 
    std::thread spinThread([&](){
        ros::spin();
    });

    std::string topicReferencePose;
    std::string topicCurrentPose;
    std::string basePidX;
    std::string basePidY;
    std::string basePidZ;

    ros::param::param<std::string>("reference_pose", topicReferencePose, "/ual_target_pose");
    ros::param::param<std::string>("current_pose", topicCurrentPose, "/uav_1/mavros/local_position/pose");
    ros::param::param<std::string>("base_name_pid_x", basePidX, "pid_x");
    ros::param::param<std::string>("base_name_pid_y", basePidY, "pid_y");
    ros::param::param<std::string>("base_name_pid_z", basePidZ, "pìd_z");

    QApplication a(_argc, _argv);
    PidTuneGui gui( "/aeroarms/crawler_detection/fcu_crawler_pose", 
                    "/aeroarms/mav_controller/ref_fcu_crawler_pose",
                    {   std::pair<std::string, std::string>("X", basePidX),
                        std::pair<std::string, std::string>("Y", basePidY),
                        std::pair<std::string, std::string>("Z", basePidZ)},
                    {
                        {"uav_address",_argv[1]},
                        {"port_pid_x", _argv[2]},
                        {"port_pid_y", _argv[3]},
                        {"port_pid_z", _argv[4]},
                        {"estimate_vision", _argv[5]},
                        {"reference_vision", _argv[6]}
                    });
    gui.show();
    
    return a.exec();
}
