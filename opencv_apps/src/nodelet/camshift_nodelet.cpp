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

// https://github.com/Itseez/opencv/blob/2.4/samples/cpp/camshiftdemo.cpp
/**
 * This is a demo that shows mean-shift based tracking
 * You select a color objects such as your face and it tracks it.
 * This reads from video camera (0 by default, or the camera number the user enters
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <ctype.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include "opencv_apps/CamShiftConfig.h"
#include "opencv_apps/RotatedRectStamped.h"

namespace camshift {
class CamShiftNodelet : public nodelet::Nodelet
{
  image_transport::Publisher img_pub_, bproj_pub_;
  image_transport::Subscriber img_sub_;
  image_transport::CameraSubscriber cam_sub_;
  ros::Publisher msg_pub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;
  ros::NodeHandle nh_, local_nh_;

  camshift::CamShiftConfig config_;
  dynamic_reconfigure::Server<camshift::CamShiftConfig> srv;

  bool debug_view_;
  int subscriber_count_;
  ros::Time prev_stamp_;

  std::string window_name_, histogram_name_;
  static bool need_config_update_;
  static bool on_mouse_update_;
  static int on_mouse_event_;
  static int on_mouse_x_;
  static int on_mouse_y_;

  int vmin_, vmax_, smin_;
  bool backprojMode;
  bool selectObject;
  int trackObject;
  bool showHist;
  cv::Point origin;
  cv::Rect selection;
  bool paused;

  cv::Rect trackWindow;
  int hsize;
  float hranges[2];
  const float* phranges;
  cv::Mat hist, histimg;
  //cv::Mat hsv;
  
  static void onMouse( int event, int x, int y, int, void* )
  {
    on_mouse_update_ = true;
    on_mouse_event_ = event;
    on_mouse_x_ = x;
    on_mouse_y_ = y;
  }

  void reconfigureCallback(camshift::CamShiftConfig &new_config, uint32_t level)
  {
    config_ = new_config;
    vmin_ = config_.vmin;
    vmax_ = config_.vmax;
    smin_ = config_.smin;
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
      cv::Mat backproj;
      
      // Messages
      opencv_apps::RotatedRectStamped rect_msg;
      rect_msg.header = msg->header;

      // Do the work
      
      if( debug_view_) {
        /// Create Trackbars for Thresholds

        cv::namedWindow( window_name_, cv::WINDOW_AUTOSIZE );

        cv::setMouseCallback( window_name_, onMouse, 0 );
        cv::createTrackbar( "Vmin", window_name_, &vmin_, 256, trackbarCallback);
        cv::createTrackbar( "Vmax", window_name_, &vmax_, 256, trackbarCallback);
        cv::createTrackbar( "Smin", window_name_, &smin_, 256, trackbarCallback);

        if (need_config_update_) {
          config_.vmin = vmin_;
          config_.vmax = vmax_;
          config_.smin = smin_;
          srv.updateConfig(config_);
          need_config_update_ = false;
        }
      }

      if ( on_mouse_update_ ) {
        int event = on_mouse_event_;
        int x = on_mouse_x_;
        int y = on_mouse_y_;

        if( selectObject )
        {
          selection.x = MIN(x, origin.x);
          selection.y = MIN(y, origin.y);
          selection.width = std::abs(x - origin.x);
          selection.height = std::abs(y - origin.y);

          selection &= cv::Rect(0, 0, frame.cols, frame.rows);
        }

        switch( event )
        {
          case cv::EVENT_LBUTTONDOWN:
            origin = cv::Point(x,y);
            selection = cv::Rect(x,y,0,0);
            selectObject = true;
            break;
          case cv::EVENT_LBUTTONUP:
            selectObject = false;
            if( selection.width > 0 && selection.height > 0 )
              trackObject = -1;
            break;
        }
        on_mouse_update_ = false;
      }

      if( !paused )
      {
        cv::Mat hsv, hue, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        if( trackObject )
        {
          int _vmin = vmin_, _vmax = vmax_;

          cv::inRange(hsv, cv::Scalar(0, smin_, MIN(_vmin,_vmax)),
                      cv::Scalar(180, 256, MAX(_vmin, _vmax)), mask);
          int ch[] = {0, 0};
          hue.create(hsv.size(), hsv.depth());
          cv::mixChannels(&hsv, 1, &hue, 1, ch, 1);

          if( trackObject < 0 )
          {
            cv::Mat roi(hue, selection), maskroi(mask, selection);
            cv::calcHist(&roi, 1, 0, maskroi, hist, 1, &hsize, &phranges);
            cv::normalize(hist, hist, 0, 255, cv::NORM_MINMAX);
            std::vector<float> hist_value;
            hist_value.resize(hsize);
            for(int i = 0; i < hsize; i ++) { hist_value[i] = hist.at<float>(i);}
            local_nh_.setParam("histogram", hist_value);

            trackWindow = selection;
            trackObject = 1;

            histimg = cv::Scalar::all(0);
            int binW = histimg.cols / hsize;
            cv::Mat buf(1, hsize, CV_8UC3);
            for( int i = 0; i < hsize; i++ )
              buf.at<cv::Vec3b>(i) = cv::Vec3b(cv::saturate_cast<uchar>(i*180./hsize), 255, 255);
            cv::cvtColor(buf, buf, cv::COLOR_HSV2BGR);

            for( int i = 0; i < hsize; i++ )
            {
              int val = cv::saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
              cv::rectangle( histimg, cv::Point(i*binW,histimg.rows),
                             cv::Point((i+1)*binW,histimg.rows - val),
                             cv::Scalar(buf.at<cv::Vec3b>(i)), -1, 8 );
            }
          }

          cv::calcBackProject(&hue, 1, 0, hist, backproj, &phranges);
          backproj &= mask;
          cv::RotatedRect trackBox = cv::CamShift(backproj, trackWindow,
                                                  cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 10, 1 ));
          if( trackWindow.area() <= 1 )
          {
            int cols = backproj.cols, rows = backproj.rows, r = (MIN(cols, rows) + 5)/6;
            //trackWindow = cv::Rect(trackWindow.x - r, trackWindow.y - r,
            //                       trackWindow.x + r, trackWindow.y + r) &
            trackWindow = cv::Rect(cols/2 - r, rows/2 - r,
                                   cols/2 + r, rows/2 + r) &
              cv::Rect(0, 0, cols, rows);
          }

          if( backprojMode )
            cv::cvtColor( backproj, frame, cv::COLOR_GRAY2BGR );
#ifndef CV_VERSION_EPOCH
          cv::ellipse( frame, trackBox, cv::Scalar(0,0,255), 3, cv::LINE_AA );
#else
          cv::ellipse( frame, trackBox, cv::Scalar(0,0,255), 3, CV_AA );
#endif
          
          rect_msg.rect.angle = trackBox.angle;
          opencv_apps::Point2D point_msg;
          opencv_apps::Size size_msg;
          point_msg.x = trackBox.center.x;
          point_msg.y = trackBox.center.y;
          size_msg.width = trackBox.size.width;
          size_msg.height = trackBox.size.height;
          rect_msg.rect.center = point_msg;
          rect_msg.rect.size = size_msg;
        }
      }
      else if( trackObject < 0 )
        paused = false;

      if( selectObject && selection.width > 0 && selection.height > 0 )
      {
        cv::Mat roi(frame, selection);
        bitwise_not(roi, roi);
      }

      if( debug_view_ ) {
        cv::imshow( window_name_, frame );
        cv::imshow( histogram_name_, histimg );

        char c = (char)cv::waitKey(1);
        //if( c == 27 )
        //  break;
        switch(c)
        {
          case 'b':
            backprojMode = !backprojMode;
            break;
          case 'c':
            trackObject = 0;
            histimg = cv::Scalar::all(0);
            break;
          case 'h':
            showHist = !showHist;
            if( !showHist )
              cv::destroyWindow( histogram_name_ );
            else
              cv::namedWindow( histogram_name_, 1 );
            break;
          case 'p':
            paused = !paused;
            break;
          default:
            ;
        }
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_img1 = cv_bridge::CvImage(msg->header, msg->encoding, frame).toImageMsg();
      sensor_msgs::Image::Ptr out_img2 = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::MONO8, backproj).toImageMsg();
      img_pub_.publish(out_img1);
      bproj_pub_.publish(out_img2);
      if( trackObject )
        msg_pub_.publish(rect_msg);
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
      cam_sub_ = it_->subscribeCamera("image", 3, &CamShiftNodelet::imageCallbackWithInfo, this);
    else
      img_sub_ = it_->subscribe("image", 3, &CamShiftNodelet::imageCallback, this);
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

    window_name_ = "CamShift Demo";
    histogram_name_ = "Histogram";

    vmin_ = 10; vmax_ = 256; smin_ = 30;
    backprojMode = false;
    selectObject = false;
    trackObject = 0;
    showHist = true;
    paused = false;

    hsize = 16;
    hranges[0] = 0;
    hranges[1] = 180;
    phranges = hranges;
    histimg = cv::Mat::zeros(200, 320, CV_8UC3);

    image_transport::SubscriberStatusCallback img_connect_cb    = boost::bind(&CamShiftNodelet::img_connectCb, this, _1);
    image_transport::SubscriberStatusCallback img_disconnect_cb = boost::bind(&CamShiftNodelet::img_disconnectCb, this, _1);
    ros::SubscriberStatusCallback msg_connect_cb    = boost::bind(&CamShiftNodelet::msg_connectCb, this, _1);
    ros::SubscriberStatusCallback msg_disconnect_cb = boost::bind(&CamShiftNodelet::msg_disconnectCb, this, _1);
    img_pub_ = image_transport::ImageTransport(local_nh_).advertise("image", 1, img_connect_cb, img_disconnect_cb);
    bproj_pub_ = image_transport::ImageTransport(local_nh_).advertise("back_project", 1, img_connect_cb, img_disconnect_cb);
    msg_pub_ = local_nh_.advertise<opencv_apps::RotatedRectStamped>("track_box", 1, msg_connect_cb, msg_disconnect_cb);
        
    if( debug_view_ ) {
      subscriber_count_++;
    }

    dynamic_reconfigure::Server<camshift::CamShiftConfig>::CallbackType f =
      boost::bind(&CamShiftNodelet::reconfigureCallback, this, _1, _2);
    srv.setCallback(f);

    NODELET_INFO("Hot keys: ");
    NODELET_INFO("\tESC - quit the program");
    NODELET_INFO("\tc - stop the tracking");
    NODELET_INFO("\tb - switch to/from backprojection view");
    NODELET_INFO("\th - show/hide object histogram");
    NODELET_INFO("\tp - pause video");
    NODELET_INFO("To initialize tracking, select the object with mouse");

    std::vector<float> hist_value;
    local_nh_.getParam("histogram", hist_value);
    if ( hist_value.size() == hsize ) {
      hist.create(hsize, 1, CV_32F);
      for(int i = 0; i < hsize; i ++) { hist.at<float>(i) = hist_value[i];}
      trackObject = 1;
      trackWindow = cv::Rect(0, 0, 640, 480); //
      

      histimg = cv::Scalar::all(0);
      int binW = histimg.cols / hsize;
      cv::Mat buf(1, hsize, CV_8UC3);
      for( int i = 0; i < hsize; i++ )
        buf.at<cv::Vec3b>(i) = cv::Vec3b(cv::saturate_cast<uchar>(i*180./hsize), 255, 255);
      cv::cvtColor(buf, buf, cv::COLOR_HSV2BGR);
      
      for( int i = 0; i < hsize; i++ )
      {
        int val = cv::saturate_cast<int>(hist.at<float>(i)*histimg.rows/255);
        cv::rectangle( histimg, cv::Point(i*binW,histimg.rows),
                       cv::Point((i+1)*binW,histimg.rows - val),
                       cv::Scalar(buf.at<cv::Vec3b>(i)), -1, 8 );
      }
    }

  }
};
bool CamShiftNodelet::need_config_update_ = false;
bool CamShiftNodelet::on_mouse_update_ = false;
int CamShiftNodelet::on_mouse_event_ = 0;
int CamShiftNodelet::on_mouse_x_ = 0;
int CamShiftNodelet::on_mouse_y_ = 0;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(camshift::CamShiftNodelet, nodelet::Nodelet);
