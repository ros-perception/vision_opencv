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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/segment_objects.cpp
/**
 * This program demonstrated a simple method of connected components clean up of background subtraction
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/background_segm.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/SegmentObjectsConfig.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Float64.h"
#include "opencv_apps/Contour.h"
#include "opencv_apps/ContourArray.h"
#include "opencv_apps/ContourArrayStamped.h"

namespace segment_objects {
class SegmentObjectsNodelet : public nodelet::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_, area_pub_;
  ros::ServiceServer update_bg_model_service_;

  boost::shared_ptr<image_transport::ImageTransport> it_;
  ros::NodeHandle nh_, local_nh_;

  segment_objects::SegmentObjectsConfig config_;
  dynamic_reconfigure::Server<segment_objects::SegmentObjectsConfig> srv;

  bool debug_view_;
  int subscriber_count_;
  ros::Time prev_stamp_;

  std::string window_name_;
  static bool need_config_update_;

#ifndef CV_VERSION_EPOCH
  cv::Ptr<cv::BackgroundSubtractorMOG2> bgsubtractor;
#else
  cv::BackgroundSubtractorMOG2 bgsubtractor;
#endif
  bool update_bg_model;

  void reconfigureCallback(segment_objects::SegmentObjectsConfig &new_config, uint32_t level)
  {
    config_ = new_config;
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
      opencv_apps::ContourArrayStamped contours_msg;
      contours_msg.header = msg->header;

      // Do the work
      cv::Mat bgmask, out_frame;

      if( debug_view_) {
        /// Create Trackbars for Thresholds
        cv::namedWindow( window_name_, cv::WINDOW_AUTOSIZE );
        if (need_config_update_) {
          srv.updateConfig(config_);
          need_config_update_ = false;
        }
      }

#ifndef CV_VERSION_EPOCH
      bgsubtractor->apply(frame, bgmask, update_bg_model ? -1 : 0);
#else
      bgsubtractor(frame, bgmask, update_bg_model ? -1 : 0);
#endif
      //refineSegments(tmp_frame, bgmask, out_frame);
      int niters = 3;

      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;

      cv::Mat temp;

      cv::dilate(bgmask, temp, cv::Mat(), cv::Point(-1,-1), niters);
      cv::erode(temp, temp, cv::Mat(), cv::Point(-1,-1), niters*2);
      cv::dilate(temp, temp, cv::Mat(), cv::Point(-1,-1), niters);

      cv::findContours( temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );

      out_frame = cv::Mat::zeros(frame.size(), CV_8UC3);

      if( contours.size() == 0 )
        return;

      // iterate through all the top-level contours,
      // draw each connected component with its own random color
      int idx = 0, largestComp = 0;
      double maxArea = 0;

      for( ; idx >= 0; idx = hierarchy[idx][0] )
      {
        const std::vector<cv::Point>& c = contours[idx];
        double area = fabs(cv::contourArea(cv::Mat(c)));
        if( area > maxArea )
        {
          maxArea = area;
          largestComp = idx;
        }
      }
      cv::Scalar color( 0, 0, 255 );
      cv::drawContours( out_frame, contours, largestComp, color, CV_FILLED, 8, hierarchy );

      std_msgs::Float64 area_msg;
      area_msg.data = maxArea;
      for( size_t i = 0; i< contours.size(); i++ ) {
        opencv_apps::Contour contour_msg;
        for ( size_t j = 0; j < contours[i].size(); j++ ) {
          opencv_apps::Point2D point_msg;
          point_msg.x = contours[i][j].x;
          point_msg.y = contours[i][j].y;
          contour_msg.points.push_back(point_msg);
        }
        contours_msg.contours.push_back(contour_msg);
      }

      //-- Show what you got
      if( debug_view_) {
        cv::imshow( window_name_, out_frame );
        int keycode = cv::waitKey(1);
        //if( keycode == 27 )
        //    break;
        if( keycode == ' ' )
        {
            update_bg_model = !update_bg_model;
            NODELET_INFO("Learn background is in state = %d",update_bg_model);
        }
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, msg->encoding, out_frame).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_.publish(contours_msg);
      area_pub_.publish(area_msg);
    }
    catch (cv::Exception &e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }
  
  bool update_bg_model_cb(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
    update_bg_model = !update_bg_model;
    NODELET_INFO("Learn background is in state = %d",update_bg_model);
    return true;
  }

  void subscribe()
  {
    NODELET_DEBUG("Subscribing to image topic.");
    if (config_.use_camera_info)
      cam_sub_ = it_->subscribeCamera("image", 3, &SegmentObjectsNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", 3, &SegmentObjectsNodelet::imageCallback, this);
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

    window_name_ = "segmented";
    update_bg_model = true;

#ifndef CV_VERSION_EPOCH
    bgsubtractor = cv::createBackgroundSubtractorMOG2();
#else
    bgsubtractor.set("noiseSigma", 10);
#endif

    image_transport::SubscriberStatusCallback img_connect_cb    = boost::bind(&SegmentObjectsNodelet::img_connectCb, this, _1);
    image_transport::SubscriberStatusCallback img_disconnect_cb = boost::bind(&SegmentObjectsNodelet::img_disconnectCb, this, _1);
    ros::SubscriberStatusCallback msg_connect_cb    = boost::bind(&SegmentObjectsNodelet::msg_connectCb, this, _1);
    ros::SubscriberStatusCallback msg_disconnect_cb = boost::bind(&SegmentObjectsNodelet::msg_disconnectCb, this, _1);
    img_pub_ = image_transport::ImageTransport(local_nh_).advertise("segmented", 1, img_connect_cb, img_disconnect_cb);
    msg_pub_ = local_nh_.advertise<opencv_apps::ContourArrayStamped>("contours", 1, msg_connect_cb, msg_disconnect_cb);
    area_pub_ = local_nh_.advertise<std_msgs::Float64>("area", 1, msg_connect_cb, msg_disconnect_cb);
    update_bg_model_service_ = local_nh_.advertiseService("update_bg_model", &SegmentObjectsNodelet::update_bg_model_cb, this);
        
    if( debug_view_ ) {
      subscriber_count_++;
    }

    dynamic_reconfigure::Server<segment_objects::SegmentObjectsConfig>::CallbackType f =
      boost::bind(&SegmentObjectsNodelet::reconfigureCallback, this, _1, _2);
    srv.setCallback(f);
  }
};
bool SegmentObjectsNodelet::need_config_update_ = false;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(segment_objects::SegmentObjectsNodelet, nodelet::Nodelet);
