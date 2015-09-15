/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Kei Okada.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Kei Okada nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ShapeDescriptors/moments_demo.cpp
/**
 * @function moments_demo.cpp
 * @brief Demo code to calculate moments
 * @author OpenCV team
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/ContourMomentsConfig.h"
#include "opencv_apps/Moment.h"
#include "opencv_apps/MomentArray.h"
#include "opencv_apps/MomentArrayStamped.h"

namespace contour_moments {
class ContourMomentsNodelet : public nodelet::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;
  ros::NodeHandle nh_, local_nh_;

  contour_moments::ContourMomentsConfig config_;
  dynamic_reconfigure::Server<contour_moments::ContourMomentsConfig> srv;

  bool debug_view_;
  int subscriber_count_;
  ros::Time prev_stamp_;

  int low_threshold_;

  std::string window_name_;
  static bool need_config_update_;

  void reconfigureCallback(contour_moments::ContourMomentsConfig &new_config, uint32_t level)
  {
    config_ = new_config;
    low_threshold_ = config_.canny_low_threshold;
    if (subscriber_count_)
    { // @todo Could do this without an interruption at some point.
      unsubscribe();
      subscribe();
    }
  }

  const std::string &frameWithDefault(const std::string &frame, const std::string &image_frame)
  {
    if (frame.empty())
      return image_frame;
    return frame;
  }

  void imageCallbackWithInfo(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    do_work(msg, cam_info->header.frame_id);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    do_work(msg, msg->header.frame_id);
  }

  static void trackbarCallback( int, void* )
  {
    need_config_update_ = true;
  }

  void do_work(const sensor_msgs::ImageConstPtr& msg, const std::string input_frame_from_msg)
  {
    // Work on the image.
    try
    {
      // Convert the image into something opencv can handle.
      cv::Mat frame = cv_bridge::toCvShare(msg, msg->encoding)->image;

      // Messages
      opencv_apps::MomentArrayStamped moments_msg;
      moments_msg.header = msg->header;

      // Do the work
      cv::Mat src_gray;
      /// Convert image to gray and blur it
      if ( frame.channels() > 1 ) {
        cv::cvtColor( frame, src_gray, cv::COLOR_BGR2GRAY );
      } else {
        src_gray = frame;
      }
      cv::blur( src_gray, src_gray, cv::Size(3,3) );

      /// Create window
      if( debug_view_) {
        cv::namedWindow( window_name_, cv::WINDOW_AUTOSIZE );
      }

      cv::Mat canny_output;
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::RNG rng(12345);

      /// Detect edges using canny
      cv::Canny( src_gray, canny_output, low_threshold_ , low_threshold_ *2, 3 );
      /// Find contours
      cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

      /// Get the moments
      std::vector<cv::Moments> mu(contours.size() );
      for( size_t i = 0; i < contours.size(); i++ )
      { mu[i] = moments( contours[i], false ); }

      ///  Get the mass centers:
      std::vector<cv::Point2f> mc( contours.size() );
      for( size_t i = 0; i < contours.size(); i++ )
      { mc[i] = cv::Point2f( static_cast<float>(mu[i].m10/mu[i].m00) , static_cast<float>(mu[i].m01/mu[i].m00) ); }

      /// Draw contours
      cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC3 );
      for( size_t i = 0; i< contours.size(); i++ )
      {
        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        cv::drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, cv::Point() );
        cv::circle( drawing, mc[i], 4, color, -1, 8, 0 );
      }

      /// Calculate the area with the moments 00 and compare with the result of the OpenCV function
      printf("\t Info: Area and Contour Length \n");
      for( size_t i = 0; i< contours.size(); i++ )
      {
        printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", (int)i, mu[i].m00, cv::contourArea(contours[i]), cv::arcLength( contours[i], true ) );
        cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        cv::drawContours( drawing, contours, (int)i, color, 2, 8, hierarchy, 0, cv::Point() );
        cv::circle( drawing, mc[i], 4, color, -1, 8, 0 );

        opencv_apps::Moment moment_msg;
        moment_msg.m00 = mu[i].m00;
        moment_msg.m10 = mu[i].m10;
        moment_msg.m01 = mu[i].m01;
        moment_msg.m20 = mu[i].m20;
        moment_msg.m11 = mu[i].m11;
        moment_msg.m02 = mu[i].m02;
        moment_msg.m30 = mu[i].m30;
        moment_msg.m21 = mu[i].m21;
        moment_msg.m12 = mu[i].m12;
        moment_msg.m03 = mu[i].m03;
        moment_msg.mu20 = mu[i].mu20;
        moment_msg.mu11 = mu[i].mu11;
        moment_msg.mu02 = mu[i].mu02;
        moment_msg.mu30 = mu[i].mu30;
        moment_msg.mu21 = mu[i].mu21;
        moment_msg.mu12 = mu[i].mu12;
        moment_msg.mu03 = mu[i].mu03;
        moment_msg.nu20 = mu[i].nu20;
        moment_msg.nu11 = mu[i].nu11;
        moment_msg.nu02 = mu[i].nu02;
        moment_msg.nu30 = mu[i].nu30;
        moment_msg.nu21 = mu[i].nu21;
        moment_msg.nu12 = mu[i].nu12;
        moment_msg.nu03 = mu[i].nu03;
        opencv_apps::Point2D center_msg;
        center_msg.x = mc[i].x;
        center_msg.y = mc[i].y;
        moment_msg.center = center_msg;
        moment_msg.area = cv::contourArea(contours[i]);
        moment_msg.length = cv::arcLength(contours[i], true);
        moments_msg.moments.push_back(moment_msg);
      }

      if( debug_view_) {
        cv::imshow( window_name_, drawing );
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, msg->encoding, drawing).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_.publish(moments_msg);
    }
    catch (cv::Exception &e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe()
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info)
      cam_sub_ = it_->subscribeCamera("image", 3, &ContourMomentsNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", 3, &ContourMomentsNodelet::imageCallback, this);
  }

  void unsubscribe()
  {
    NODELET_DEBUG("Unsubscribing from image topic.");
    img_sub_.shutdown();
    cam_sub_.shutdown();
  }

  void img_connectCb(const image_transport::SingleSubscriberPublisher& ssp)
  {
    if (subscriber_count_++ == 0) {
      subscribe();
    }
  }

  void img_disconnectCb(const image_transport::SingleSubscriberPublisher&)
  {
    subscriber_count_--;
    if (subscriber_count_ == 0) {
      unsubscribe();
    }
  }

  void msg_connectCb(const ros::SingleSubscriberPublisher& ssp)
  {
    if (subscriber_count_++ == 0) {
      subscribe();
    }
  }

  void msg_disconnectCb(const ros::SingleSubscriberPublisher&)
  {
    subscriber_count_--;
    if (subscriber_count_ == 0) {
      unsubscribe();
    }
  }

public:
  virtual void onInit()
  {
    nh_ = getNodeHandle();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nh_));
    local_nh_ = ros::NodeHandle("~");

    local_nh_.param("debug_view", debug_view_, false);
    subscriber_count_ = 0;
    prev_stamp_ = ros::Time(0, 0);

    window_name_ = "Edge Detection Demo";
    low_threshold_ = 100; // only for canny

    image_transport::SubscriberStatusCallback img_connect_cb    = boost::bind(&ContourMomentsNodelet::img_connectCb, this, _1);
    image_transport::SubscriberStatusCallback img_disconnect_cb = boost::bind(&ContourMomentsNodelet::img_disconnectCb, this, _1);
    ros::SubscriberStatusCallback msg_connect_cb    = boost::bind(&ContourMomentsNodelet::msg_connectCb, this, _1);
    ros::SubscriberStatusCallback msg_disconnect_cb = boost::bind(&ContourMomentsNodelet::msg_disconnectCb, this, _1);
    img_pub_ = image_transport::ImageTransport(local_nh_).advertise("image", 1, img_connect_cb, img_disconnect_cb);
    msg_pub_ = local_nh_.advertise<opencv_apps::MomentArrayStamped>("moments", 1, msg_connect_cb, msg_disconnect_cb);

    if( debug_view_ ) {
      subscriber_count_++;
    }

    dynamic_reconfigure::Server<contour_moments::ContourMomentsConfig>::CallbackType f =
      boost::bind(&ContourMomentsNodelet::reconfigureCallback, this, _1, _2);
    srv.setCallback(f);
  }
};
bool ContourMomentsNodelet::need_config_update_ = false;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(contour_moments::ContourMomentsNodelet, nodelet::Nodelet);
