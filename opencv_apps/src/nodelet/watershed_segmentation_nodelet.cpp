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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/watershed.cpp
/**
 * This program demonstrates the famous watershed segmentation algorithm in OpenCV: watershed()
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/WatershedSegmentationConfig.h"

namespace watershed_segmentation {
class WatershedSegmentationNodelet : public nodelet::Nodelet
{
  image_transport::Publisher img_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;
  ros::NodeHandle nh_, local_nh_;

  watershed_segmentation::WatershedSegmentationConfig config_;
  dynamic_reconfigure::Server<watershed_segmentation::WatershedSegmentationConfig> srv;

  bool debug_view_;
  int subscriber_count_;
  ros::Time prev_stamp_;

  std::string window_name_, segment_name_;
  static bool need_config_update_;
  static bool on_mouse_update_;
  static int on_mouse_event_;
  static int on_mouse_x_;
  static int on_mouse_y_;
  static int on_mouse_flags_;

  cv::Mat markerMask;
  cv::Point prevPt;

  static void onMouse( int event, int x, int y, int flags, void* )
  {
    on_mouse_update_ = true;
    on_mouse_event_ = event;
    on_mouse_x_ = x;
    on_mouse_y_ = y;
    on_mouse_flags_ = flags;
  }

  void reconfigureCallback(watershed_segmentation::WatershedSegmentationConfig &new_config, uint32_t level)
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
      //opencv_apps::LineArrayStamped lines_msg;
      //lines_msg.header = msg->header;

      // Do the work
      //std::vector<cv::Rect> faces;
      cv::Mat imgGray;

      /// Initialize
      if ( markerMask.empty() )  {
        cv::cvtColor(frame, markerMask, cv::COLOR_BGR2GRAY);
        cv::cvtColor(markerMask, imgGray, cv::COLOR_GRAY2BGR);
        markerMask = cv::Scalar::all(0);
      }

      if( debug_view_) {
        cv::imshow( window_name_, frame);
        cv::setMouseCallback( window_name_, onMouse, 0 );
        if (need_config_update_) {
          srv.updateConfig(config_);
          need_config_update_ = false;
        }

        if ( on_mouse_update_ ) {
          int event = on_mouse_event_;
          int x = on_mouse_x_;
          int y = on_mouse_y_;
          int flags = on_mouse_flags_;

          if( x < 0 || x >= frame.cols || y < 0 || y >= frame.rows )
            return;
          if( event == cv::EVENT_LBUTTONUP || !(flags & cv::EVENT_FLAG_LBUTTON) )
            prevPt = cv::Point(-1,-1);
          else if( event == cv::EVENT_LBUTTONDOWN )
            prevPt = cv::Point(x,y);
          else if( event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON) )
          {
            cv::Point pt(x, y);
            if( prevPt.x < 0 )
              prevPt = pt;
            cv::line( markerMask, prevPt, pt, cv::Scalar::all(255), 5, 8, 0 );
            cv::line( frame, prevPt, pt, cv::Scalar::all(255), 5, 8, 0 );
            prevPt = pt;
            cv::imshow(window_name_, markerMask);
          }
        }
        cv::waitKey(1);
      }
      
      int i, j, compCount = 0;
      std::vector<std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hierarchy;

      cv::findContours(markerMask, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

      if( contours.empty() ) {
        NODELET_WARN("contnorus is empty");
        return; //continue;
      }
      cv::Mat markers(markerMask.size(), CV_32S);
      markers = cv::Scalar::all(0);
      int idx = 0;
      for( ; idx >= 0; idx = hierarchy[idx][0], compCount++ )
        cv::drawContours(markers, contours, idx, cv::Scalar::all(compCount+1), -1, 8, hierarchy, INT_MAX);

      if( compCount == 0 ) {
        NODELET_WARN("compCount is 0");
        return; //continue;
      }

      std::vector<cv::Vec3b> colorTab;
      for( i = 0; i < compCount; i++ )
      {
        int b = cv::theRNG().uniform(0, 255);
        int g = cv::theRNG().uniform(0, 255);
        int r = cv::theRNG().uniform(0, 255);

        colorTab.push_back(cv::Vec3b((uchar)b, (uchar)g, (uchar)r));
      }

      double t = (double)cv::getTickCount();
      cv::watershed( frame, markers );
      t = (double)cv::getTickCount() - t;
      NODELET_INFO( "execution time = %gms", t*1000./cv::getTickFrequency() );

      cv::Mat wshed(markers.size(), CV_8UC3);

      // paint the watershed image
      for( i = 0; i < markers.rows; i++ )
        for( j = 0; j < markers.cols; j++ )
        {
          int index = markers.at<int>(i,j);
          if( index == -1 )
            wshed.at<cv::Vec3b>(i,j) = cv::Vec3b(255,255,255);
          else if( index <= 0 || index > compCount )
            wshed.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
          else
            wshed.at<cv::Vec3b>(i,j) = colorTab[index - 1];
        }

      wshed = wshed*0.5 + imgGray*0.5;

      //-- Show what you got
      if( debug_view_) {
        cv::imshow( segment_name_, wshed );

        int c = cv::waitKey(1);
        //if( (char)c == 27 )
        //    break;
        if( (char)c == 'r' )
        {
          markerMask = cv::Scalar::all(0);
        }
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, msg->encoding, wshed).toImageMsg();
      img_pub_.publish(out_img);
      //msg_pub_.publish(lines_msg);
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
      cam_sub_ = it_->subscribeCamera("image", 3, &WatershedSegmentationNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", 3, &WatershedSegmentationNodelet::imageCallback, this);
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

    window_name_ = "input";
    segment_name_ = "watershed transform";
    prevPt.x = -1;
    prevPt.y = -1;

    image_transport::SubscriberStatusCallback img_connect_cb    = boost::bind(&WatershedSegmentationNodelet::img_connectCb, this, _1);
    image_transport::SubscriberStatusCallback img_disconnect_cb = boost::bind(&WatershedSegmentationNodelet::img_disconnectCb, this, _1);
    ros::SubscriberStatusCallback msg_connect_cb    = boost::bind(&WatershedSegmentationNodelet::msg_connectCb, this, _1);
    ros::SubscriberStatusCallback msg_disconnect_cb = boost::bind(&WatershedSegmentationNodelet::msg_disconnectCb, this, _1);
    img_pub_ = image_transport::ImageTransport(local_nh_).advertise("image", 1, img_connect_cb, img_disconnect_cb);
    //msg_pub_ = local_nh_.advertise<opencv_apps::LineArrayStamped>("lines", 1, msg_connect_cb, msg_disconnect_cb);
        
    if( debug_view_ ) {
      subscriber_count_++;
    }

    dynamic_reconfigure::Server<watershed_segmentation::WatershedSegmentationConfig>::CallbackType f =
      boost::bind(&WatershedSegmentationNodelet::reconfigureCallback, this, _1, _2);
    srv.setCallback(f);

    NODELET_INFO("This program demonstrates the famous watershed segmentation algorithm in OpenCV: watershed()");
    NODELET_INFO("Hot keys: ");
    NODELET_INFO("\tESC - quit the program");
    NODELET_INFO("\tr - restore the original image");
    NODELET_INFO("\tw or SPACE - run watershed segmentation algorithm");
    NODELET_INFO("\t\t(before running it, *roughly* mark the areas to segment on the image)");
    NODELET_INFO("\t  (before that, roughly outline several markers on the image)");
  }
};
bool WatershedSegmentationNodelet::need_config_update_ = false;
bool WatershedSegmentationNodelet::on_mouse_update_ = false;
int WatershedSegmentationNodelet::on_mouse_event_ = 0;
int WatershedSegmentationNodelet::on_mouse_x_ = 0;
int WatershedSegmentationNodelet::on_mouse_y_ = 0;
int WatershedSegmentationNodelet::on_mouse_flags_ = 0;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(watershed_segmentation::WatershedSegmentationNodelet, nodelet::Nodelet);
