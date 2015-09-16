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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code/ImgTrans/HoughLines_Demo.cpp
/**
 * @file HoughLines_Demo.cpp
 * @brief Demo code for Hough Transform
 * @author OpenCV team
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/HoughLinesConfig.h"
#include "opencv_apps/Line.h"
#include "opencv_apps/LineArray.h"
#include "opencv_apps/LineArrayStamped.h"

namespace hough_lines {
class HoughLinesNodelet : public nodelet::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;
  ros::NodeHandle nh_, local_nh_;

  hough_lines::HoughLinesConfig config_;
  dynamic_reconfigure::Server<hough_lines::HoughLinesConfig> srv;

  bool debug_view_;
  int subscriber_count_;
  ros::Time prev_stamp_;

  std::string window_name_;
  static bool need_config_update_;

  int min_threshold_;
  int max_threshold_;
  int threshold_;

  void reconfigureCallback(hough_lines::HoughLinesConfig &new_config, uint32_t level)
  {
    config_ = new_config;
    threshold_ = config_.threshold;
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
      opencv_apps::LineArrayStamped lines_msg;
      lines_msg.header = msg->header;

      // Do the work
      std::vector<cv::Rect> faces;
      cv::Mat src_gray, edges;

      if ( frame.channels() > 1 ) {
        cv::cvtColor( frame, src_gray, cv::COLOR_BGR2GRAY );
      } else {
        src_gray = frame;
      }
      /// Apply Canny edge detector
      Canny( src_gray, edges, 50, 200, 3 );

      if( debug_view_) {
        /// Create Trackbars for Thresholds
        char thresh_label[50];
        sprintf( thresh_label, "Thres: %d + input", min_threshold_ );

        cv::namedWindow( window_name_, cv::WINDOW_AUTOSIZE );
        cv::createTrackbar( thresh_label, window_name_, &threshold_, max_threshold_, trackbarCallback);
        if (need_config_update_) {
          config_.threshold = threshold_;
          srv.updateConfig(config_);
          need_config_update_ = false;
        }
      }

      /// Initialize
      cv::cvtColor( edges, frame, cv::COLOR_GRAY2BGR );

      switch (config_.hough_type) {
        case hough_lines::HoughLines_Standard_Hough_Transform:
          {
            std::vector<cv::Vec2f> s_lines;

            /// 1. Use Standard Hough Transform
            cv::HoughLines( edges, s_lines, 1, CV_PI/180, min_threshold_ + threshold_, 0, 0 );

            /// Show the result
            for( size_t i = 0; i < s_lines.size(); i++ )
            {
              float r = s_lines[i][0], t = s_lines[i][1];
              double cos_t = cos(t), sin_t = sin(t);
              double x0 = r*cos_t, y0 = r*sin_t;
              double alpha = 1000;

              cv::Point pt1( cvRound(x0 + alpha*(-sin_t)), cvRound(y0 + alpha*cos_t) );
              cv::Point pt2( cvRound(x0 - alpha*(-sin_t)), cvRound(y0 - alpha*cos_t) );
#ifndef CV_VERSION_EPOCH
              cv::line( frame, pt1, pt2, cv::Scalar(255,0,0), 3, cv::LINE_AA);
#else
              cv::line( frame, pt1, pt2, cv::Scalar(255,0,0), 3, CV_AA);
#endif
              opencv_apps::Line line_msg;
              line_msg.pt1.x = pt1.x;
              line_msg.pt1.y = pt1.y;
              line_msg.pt2.x = pt2.x;
              line_msg.pt2.y = pt2.y;
              lines_msg.lines.push_back(line_msg);
            }

            break;
          }
        case hough_lines::HoughLines_Probabilistic_Hough_Transform:
        default:
          {
            std::vector<cv::Vec4i> p_lines;

            /// 2. Use Probabilistic Hough Transform
            cv::HoughLinesP( edges, p_lines, 1, CV_PI/180, min_threshold_ + threshold_, 30, 10 );

            /// Show the result
            for( size_t i = 0; i < p_lines.size(); i++ )
            {
              cv::Vec4i l = p_lines[i];
#ifndef CV_VERSION_EPOCH
              cv::line( frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,0), 3, cv::LINE_AA);
#else
              cv::line( frame, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(255,0,0), 3, CV_AA);
#endif
              opencv_apps::Line line_msg;
              line_msg.pt1.x = l[0];
              line_msg.pt1.y = l[1];
              line_msg.pt2.x = l[2];
              line_msg.pt2.y = l[3];
              lines_msg.lines.push_back(line_msg);
            }

            break;
          }
      }

      //-- Show what you got
      if( debug_view_) {
        cv::imshow( window_name_, frame );
        int c = cv::waitKey(1);
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, msg->encoding,frame).toImageMsg();
      img_pub_.publish(out_img);
      msg_pub_.publish(lines_msg);
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
      cam_sub_ = it_->subscribeCamera("image", 3, &HoughLinesNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", 3, &HoughLinesNodelet::imageCallback, this);
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

    window_name_ = "Hough Lines Demo";
    min_threshold_ = 50;
    max_threshold_ = 150;
    threshold_ = max_threshold_;

    image_transport::SubscriberStatusCallback img_connect_cb    = boost::bind(&HoughLinesNodelet::img_connectCb, this, _1);
    image_transport::SubscriberStatusCallback img_disconnect_cb = boost::bind(&HoughLinesNodelet::img_disconnectCb, this, _1);
    ros::SubscriberStatusCallback msg_connect_cb    = boost::bind(&HoughLinesNodelet::msg_connectCb, this, _1);
    ros::SubscriberStatusCallback msg_disconnect_cb = boost::bind(&HoughLinesNodelet::msg_disconnectCb, this, _1);
    img_pub_ = image_transport::ImageTransport(local_nh_).advertise("image", 1, img_connect_cb, img_disconnect_cb);
    msg_pub_ = local_nh_.advertise<opencv_apps::LineArrayStamped>("lines", 1, msg_connect_cb, msg_disconnect_cb);

    if( debug_view_ ) {
      subscriber_count_++;
    }

    dynamic_reconfigure::Server<hough_lines::HoughLinesConfig>::CallbackType f =
      boost::bind(&HoughLinesNodelet::reconfigureCallback, this, _1, _2);
    srv.setCallback(f);
  }
};
bool HoughLinesNodelet::need_config_update_ = false;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hough_lines::HoughLinesNodelet, nodelet::Nodelet);
