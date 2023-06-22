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
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"
#include "../include/ImuTypes.h"

class ImuGrabber: public rclcpp::Node
{
public:
    ImuGrabber():
    Node("Mono_Inertial")
    {
      sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("/imu", 1000, std::bind(&ImuGrabber::GrabImu, this, std::placeholders::_1)); 
    };                
    void GrabImu(const sensor_msgs::msg::Imu::ConstPtr &imu_msg);

    queue<sensor_msgs::msg::Imu::ConstPtr> imuBuf;
    std::mutex ImuBufMutex;
private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

};

class ImageGrabber: public ImuGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, const bool bClahe): 
    mpSLAM(pSLAM), 
    mbClahe(bClahe)
    {
      sub_img_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 100, std::bind(&ImageGrabber::GrabImage, this, std::placeholders::_1));
    }

    void GrabImage(const sensor_msgs::msg::Image::ConstPtr& msg);
    cv::Mat GetImage(const sensor_msgs::msg::Image::ConstPtr &img_msg);
    void SyncWithImu();

    queue<sensor_msgs::msg::Image::ConstPtr> img0Buf;
    std::mutex ImageBufMutex;
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber ImuGb;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  bool bEqual = false;
  if(argc < 3 || argc > 4)
  {
    cerr << endl << "Usage: rosrun ORB_SLAM3 Mono_Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
    rclcpp::shutdown();
    return 1;
  }


  if(argc==4)
  {
    std::string sbEqual(argv[3]);
    if(sbEqual == "true")
      bEqual = true;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR,true);

  //ImageGrabber igb(&SLAM,bEqual); // TODO
  auto node = std::make_shared<ImageGrabber>(&SLAM, &bEqual);
  // Maximum delay, 5 seconds
  std::thread sync_thread(&ImageGrabber::SyncWithImu, node);
  rclcpp::spin(node);
  return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::msg::Image::ConstPtr &img_msg)
{
  ImageBufMutex.lock();
  if (!img0Buf.empty())
    img0Buf.pop();
  img0Buf.push(img_msg);
  ImageBufMutex.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::msg::Image::ConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
  
  if(cv_ptr->image.type()==0)
  {
    return cv_ptr->image.clone();
  }
  else
  {
    std::cout << "Error type" << std::endl;
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu()
{
  while(1)
  {
    cv::Mat im;
    double tIm = 0;
    if (!img0Buf.empty()&&imuBuf.empty())
    {
      tIm = img0Buf.front()->header.stamp.sec;
      if(tIm > imuBuf.back()->header.stamp.sec)
          continue;
      {
      ImageBufMutex.lock();
      im = GetImage(img0Buf.front());
      img0Buf.pop();
      ImageBufMutex.unlock();
      }

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      ImuBufMutex.lock();
      if(!imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!imuBuf.empty() && imuBuf.front()->header.stamp.sec<=tIm)
        {
          double t = imuBuf.front()->header.stamp.sec;
          cv::Point3f acc(imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y, imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y, imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          imuBuf.pop();
        }
      }
      ImuBufMutex.unlock();
      if(mbClahe)
        mClahe->apply(im,im);

      mpSLAM->TrackMonocular(im,tIm,vImuMeas);
    }

    std::chrono::milliseconds tSleep(1);
    std::this_thread::sleep_for(tSleep);
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::msg::Imu::ConstPtr  &imu_msg)
{
  ImuBufMutex.lock();
  imuBuf.push(imu_msg);
  ImuBufMutex.unlock();
  return;
}


