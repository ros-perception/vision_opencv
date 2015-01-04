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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/simpleflow_demo.cpp
/**
 * This is a demo of SimpleFlow optical flow algorithm
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/SimpleFlowConfig.h"
#include "opencv_apps/FlowArrayStamped.h"

namespace simple_flow {
class SimpleFlowNodelet : public nodelet::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;
  ros::NodeHandle nh_, local_nh_;

  simple_flow::SimpleFlowConfig config_;
  dynamic_reconfigure::Server<simple_flow::SimpleFlowConfig> srv;

  bool debug_view_;
  int subscriber_count_;
  ros::Time prev_stamp_;

  std::string window_name_;
  static bool need_config_update_;
  int scale_;
  
  cv::Mat gray, prevGray;

  void reconfigureCallback(simple_flow::SimpleFlowConfig &new_config, uint32_t level)
  {
    config_ = new_config;
    scale_ = config_.scale;
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

      cv::resize(frame, gray, cv::Size(frame.cols/(double)MAX(1,scale_), frame.rows/(double)MAX(1,scale_)), 0, 0, CV_INTER_AREA);
      if(prevGray.empty())
        gray.copyTo(prevGray);

      if (gray.rows != prevGray.rows && gray.cols != prevGray.cols) {
        NODELET_WARN("Images should be of equal sizes");
        gray.copyTo(prevGray);
      }

      if (frame.type() != 16) {
        NODELET_ERROR("Images should be of equal type CV_8UC3");
      }
      // Messages
      opencv_apps::FlowArrayStamped flows_msg;
      flows_msg.header = msg->header;

      // Do the work
      cv::Mat flow;

      if( debug_view_) {
        cv::namedWindow( window_name_, cv::WINDOW_AUTOSIZE );
        cv::createTrackbar( "Scale", window_name_, &scale_, 24, trackbarCallback);
        if (need_config_update_) {
          config_.scale = scale_;
          srv.updateConfig(config_);
          need_config_update_ = false;
        }
      }

      float start = (float)cv::getTickCount();
      cv::calcOpticalFlowSF(gray, prevGray,
                            flow,
                            3, 2, 4, 4.1, 25.5, 18, 55.0, 25.5, 0.35, 18, 55.0, 25.5, 10);
      NODELET_INFO("calcOpticalFlowSF : %lf sec", (cv::getTickCount() - start) / cv::getTickFrequency());

      //writeOpticalFlowToFile(flow, file);
      int cols = flow.cols;
      int rows = flow.rows;
      double scale_col = frame.cols/(double)flow.cols;
      double scale_row = frame.rows/(double)flow.rows;


      for (int i= 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
          cv::Vec2f flow_at_point = flow.at<cv::Vec2f>(i, j);
          cv::line(frame, cv::Point(scale_col*j, scale_row*i), cv::Point(scale_col*(j+flow_at_point[0]), scale_row*(i+flow_at_point[1])), cv::Scalar::all(255), 1, 8, 0 );

          opencv_apps::Flow flow_msg;
          opencv_apps::Point2D point_msg;
          opencv_apps::Point2D velocity_msg;
          point_msg.x = scale_col*j;
          point_msg.y = scale_row*i;
          velocity_msg.x = scale_col*flow_at_point[0];
          velocity_msg.y = scale_row*flow_at_point[1];
          flow_msg.point = point_msg;
          flow_msg.velocity = velocity_msg;
          flows_msg.flow.push_back(flow_msg);
        }
      }

      //cv::cvtColor( frame, src_gray, cv::COLOR_BGR2GRAY );
      /// Apply Canny edge detector
      //Canny( src_gray, edges, 50, 200, 3 );

      //-- Show what you got
      if ( debug_view_) {
        cv::imshow( window_name_, frame );
        int c = cv::waitKey(1);
      }

      cv::swap(prevGray, gray);
      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, msg->encoding, frame).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_.publish(flows_msg);
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
      cam_sub_ = it_->subscribeCamera("image", 3, &SimpleFlowNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", 3, &SimpleFlowNodelet::imageCallback, this);
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

    window_name_ = "simpleflow_demo";
    scale_ = 4.0;

    image_transport::SubscriberStatusCallback img_connect_cb    = boost::bind(&SimpleFlowNodelet::img_connectCb, this, _1);
    image_transport::SubscriberStatusCallback img_disconnect_cb = boost::bind(&SimpleFlowNodelet::img_disconnectCb, this, _1);
    ros::SubscriberStatusCallback msg_connect_cb    = boost::bind(&SimpleFlowNodelet::msg_connectCb, this, _1);
    ros::SubscriberStatusCallback msg_disconnect_cb = boost::bind(&SimpleFlowNodelet::msg_disconnectCb, this, _1);
    img_pub_ = image_transport::ImageTransport(local_nh_).advertise("image", 1, img_connect_cb, img_disconnect_cb);
    msg_pub_ = local_nh_.advertise<opencv_apps::FlowArrayStamped>("flows", 1, msg_connect_cb, msg_disconnect_cb);
        
    if( debug_view_ ) {
      subscriber_count_++;
    }

    dynamic_reconfigure::Server<simple_flow::SimpleFlowConfig>::CallbackType f =
      boost::bind(&SimpleFlowNodelet::reconfigureCallback, this, _1, _2);
    srv.setCallback(f);
  }
};
bool SimpleFlowNodelet::need_config_update_ = false;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(simple_flow::SimpleFlowNodelet, nodelet::Nodelet);
