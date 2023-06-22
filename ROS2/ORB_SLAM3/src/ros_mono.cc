/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include"../../../include/System.h"

#include <sensor_msgs/msg/image.hpp>

class ImageGrabber: public rclcpp::Node
{
    public:
        ImageGrabber(ORB_SLAM3::System* pSLAM): 
            Node("Mono"),
            mpSLAM(pSLAM)
            {
                sub_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 1, std::bind(&ImageGrabber::GrabImage, this, std::placeholders::_1));
            }

        void GrabImage(const sensor_msgs::msg::Image::ConstPtr& msg);
        ORB_SLAM3::System* mpSLAM;

   private:
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // rclcpp::start();
    //rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("Mono");
    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings" << endl;        
        rclcpp::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    rclcpp::spin(std::make_shared<ImageGrabber>(&SLAM));

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    rclcpp::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::msg::Image::ConstPtr& msg)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.sec);
}


