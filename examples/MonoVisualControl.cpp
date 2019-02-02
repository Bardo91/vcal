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

#include <iostream>
#include <vcal/VisualControlScheme.h>


int main(int _argc, char** _argv){

    if(_argc !=2 ){
        std::cout << "Bad Input arguments, provide path to image files template with something red to track" << std::endl;
        return -1; 
    }

    // Create Visual control with mono loop
    float cx = 320;
    float cy = 240;
    float fx = 510;
    float fy = 510;

    std::function<Eigen::Vector3f(const cv::Mat&)> procImage = [&](const cv::Mat &_img) -> Eigen::Vector3f{
        cv::Mat hsv_image;
        cv::cvtColor(_img, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat segmentedImg1, segmentedImg2;
        cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(20, 255, 255), segmentedImg1);
        cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), segmentedImg2);

        cv::Mat red_hue_image;
        cv::addWeighted(segmentedImg1, 1.0, segmentedImg2, 1.0, 0.0, red_hue_image);

        int ptCounter = 0;
        cv::Point2f center(0,0);
        for(unsigned i = 0;  i < red_hue_image.rows; i++){
            for(unsigned j = 0;  j < red_hue_image.cols; j++){
                if(red_hue_image.at<uchar>(i,j) > 200){
                    center += cv::Point2f(j,i);
                    ptCounter++;
                }
            }
        }

        center /= ptCounter;

        cv::Mat display = _img.clone();
        cv::circle(display, center, 30, cv::Scalar(0,255,0),4);
        cv::imshow("img", display);
        cv::waitKey(3);

        float z = 1.0; // 1 meter
        float x = (center.x - cx)/fx*z;
        float y = (center.y - cy)/fy*z;
        return Eigen::Vector3f({x,y,z});
    };
    
    vcal::VisualControlScheme vcs(procImage);

    // Configure camera stream
    vcs.configureImageStream(   vcal::VisualControlScheme::eCamerasType::DATASET, 
                                {{"color_images", _argv[1]}});
    

    // Configure PID out with fastcom
    vcs.configureInterface( vcal::VisualControlScheme::eModules::PID,
                            vcal::VisualControlScheme::eComTypes::FASTCOM,
                            {
                                {"output_topic":"8888"      },
                                {"param_topic_out":"8889"   },
                                {"param_topic_in":"8890"    },
                                {"k":"0.8"                  },
                                {"ki":"0.0001"              },
                                {"kd":"0.3"                 },
                                {"wind_up":"10"             },
                                {"saturation":"1"           }
                            }
                        );

    if(!vcs.startPipe()){
        std::cout << "Error"<<std::endl;
        return -1;
    }

    for(;;)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));


}