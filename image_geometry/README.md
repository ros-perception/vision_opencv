## Overview

image_geometry contains Python and C++
libraries that simplifies interpreting images geometrically using the
parameters from
[sensor_msgs/msg/CameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CameraInfo.html).
Although `CameraInfo` contains all the information required to rectify
a raw image and project points onto it, **it is highly recommended that
you use this library**, since performing these operations correctly over
the space of all camera options can be non-trivial.

The camera parameters in `CameraInfo` are for a full-resolution image;
region-of-interest alone significantly complicates the creation of
rectification maps and requires adjusting the projection matrix. Adding
options such as subsampling (binning) to `CameraInfo` would further
complicate the correct interpretation of the corresponding Images. Using
`image_geometry` simplifies and future-proofs imaging code.

The `image_geometry` classes are written to be used in an
`Image`/`CameraInfo` message callback similar to
[cv_bridge](https://index.ros.org/p/cv_bridge).

In order to maintain invariance, the `CameraModel` classes offer
read-only access to specific parameters and matrices. Setting a
`CameraModel` can only be performed with full information using the
`fromCameraInfo()` functions.

These classes use the OpenCV camera model (
[OpenCV functions for 3d reconstruction](http://opencv.willowgarage.com/documentation/camera_calibration_and_3d_reconstruction.html)).
Accessors to camera matrices in the format expected by OpenCV are
provided for easy integration.

## Tutorials

- [Projecting a TF frame onto an image (C++)](#projecting-a-tf-frame-onto-an-image-c):
  This tutorial shows how to use
  image_geometry together with
  [tf](https://index.ros.org/p/tf) to project a tf frame onto a live
  image stream.

## C++ API

Please see the
[image_geometry C++ API](http://www.ros.org/doc/api/image_geometry/html/c++/)
documentation.

## Python API

Please see the
[image_geometry Python API](http://www.ros.org/doc/api/image_geometry/html/python/)
documentation.

## Projecting a TF frame onto an image (C++)

- **Note:** This tutorial assumes that you have completed the previous
  tutorials:
  [Writing a Simple Image Publisher (C++)](https://index.ros.org/p/image_transport),
  [Writing a Simple Image Subscriber (C++)](https://index.ros.org/p/image_transport),
  [Learning about tf and time (C++)](https://index.ros.org/p/tf),
  [Recording and playing back data](https://index.ros.org/p/rosbag).
- **Description:** This tutorial shows how to use
  image_geometry together with
  [tf](https://index.ros.org/p/tf) to project a tf frame onto a live
  image stream.
- **Tutorial Level:** INTERMEDIATE

In this tutorial we will write a node that projects one or more tf
frames to image coordinates and draws them on the image:

![attachment:draw_frames.png](doc/draw_frames.png)

Although this is a mathematically simple operation, this tutorial is
complex in that it integrates several different software components:
[OpenCV](https://index.ros.org/p/opencv2),
[tf](https://index.ros.org/p/tf),
[bag files](https://index.ros.org/p/rosbag),
[image_transport](https://index.ros.org/p/image_transport) and of
course(!) image_geometry. If you write
vision code using ROS, you will be using these packages frequently. Make
sure that you understand the recommended tutorials at the top before
diving in.

### Preliminaries

First begin downloading the sample data:
[plug_on_base.bag](http://download.ros.org/data/image_geometry/Tutorial/plug_on_base.bag)
(750 MB).

Create a scratch package to work in and manipulate the example code.
Create a sandbox package `learning_image_geometry` with the following
dependencies: 

```
{#!wiki buildsystem rosbuild
{{{
$ roscreate-pkg learning_image_geometry image_geometry image_transport opencv2 cv_bridge tf image_view
```

 }}}}



```
{#!wiki buildsystem catkin
{{{
$ catkin_create_pkg learning_image_geometry image_geometry image_transport opencv2 cv_bridge tf image_view
```

 }}}} Make sure that the
`learning_image_geometry` directory is included in your
`ROS_PACKAGE_PATH`.

### Writing the processing node

Here we'll create a node that subscribes to a camera image stream and
tf information, draws the requested frames on each incoming image, and
publishes the annotated images on an output topic.

Change to the directory you've created for these tutorials:



```
$ roscd learning_image_geometry
```



#### The Code

Open a new file `draw_frames.cpp` with your favorite editor and place
the following inside it: (Code for ROS Groovy) 

```cplusplus
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>

class FrameDrawer
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber sub_;
  image_transport::Publisher pub_;
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  std::vector<std::string> frame_ids_;
  CvFont font_;

public:
  FrameDrawer(const std::vector<std::string>& frame_ids)
    : it_(nh_), frame_ids_(frame_ids)
  {
    std::string image_topic = nh_.resolveName("image");
    sub_ = it_.subscribeCamera(image_topic, 1, &FrameDrawer::imageCb, this);
    pub_ = it_.advertise("image_out", 1);
    cvInitFont(&font_, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    cv::Mat image;
    cv_bridge::CvImagePtr input_bridge;
    try {
      input_bridge = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      image = input_bridge->image;
    }
    catch (cv_bridge::Exception& ex){
      ROS_ERROR("[draw_frames] Failed to convert image");
      return;
    }

    cam_model_.fromCameraInfo(info_msg);

    BOOST_FOREACH(const std::string& frame_id, frame_ids_) {
      tf::StampedTransform transform;
      try {
        ros::Time acquisition_time = info_msg->header.stamp;
        ros::Duration timeout(1.0 / 30);
        tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id,
                                      acquisition_time, timeout);
        tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id,
                                     acquisition_time, transform);
      }
      catch (tf::TransformException& ex) {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return;
      }

      tf::Point pt = transform.getOrigin();
      cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_cv);

      static const int RADIUS = 3;
      cv::circle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
      CvSize text_size;
      int baseline;
      cvGetTextSize(frame_id.c_str(), &font_, &text_size, &baseline);
      CvPoint origin = cvPoint(uv.x - text_size.width / 2,
                               uv.y - RADIUS - baseline - 3);
    cv:putText(image, frame_id.c_str(), origin, cv::FONT_HERSHEY_SIMPLEX, 12, CV_RGB(255,0,0));
    }

    pub_.publish(input_bridge->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_frames");
  std::vector<std::string> frame_ids(argv + 1, argv + argc);
  FrameDrawer drawer(frame_ids);
  ros::spin();
}
```



#### The Code Explained

Now, let's break down the code piece by piece. Most of these headers
should already be familiar to you.
`image_geometry/pinhole_camera_model.h` defines the
`PinholeCameraModel` class which we will use to project a 3d point
into image coordinates.

Let's briefly detour to the `main()` function: Here we do the usual
ROS setup. `ros::init()` removes any ROS command-line remapping
arguments. The remaining arguments are the frame IDs we want drawn; we
copy those into a `vector`.

`FrameDrawer` is our workhorse class for this example. Let's see what
it does. These members are for all our ROS communication needs. These
two objects are our bridge from the ROS message types
[sensor_msgs/msg/Image](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.html)
and
[sensor_msgs/msg/CameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CameraInfo.html)
into the world of OpenCV. We will use them in our image callback.
Finally we have the list of frame IDs to draw and a `CvFont` object
we'll use for rendering the text. The constructor. Our node handle
`nh_` is constructed first and initializes the node; we then pass it
to the `ImageTransport` constructor. `tf_listener_` automatically
connects to the `/tf` topic. We subscribe to the sychronized
`Image` + `CameraInfo` stream.
[image_transport::CameraSubscriber](http://www.ros.org/doc/api/image_transport/html/classimage__transport_1_1CameraSubscriber.html)
subscribes to the image topic and its sibling `camera_info` topic.
Review
[Writing a Simple Image Subscriber](https://index.ros.org/p/image_transport)
for more information on
[image_transport](https://index.ros.org/p/image_transport)
subscriptions. We advertise our annotated output image topic; see
[Writing a Simple Image Publisher](https://index.ros.org/p/image_transport).
Finally, we initialize our font object. Now we get to the image
callback. When processing data from a calibrated camera, we need both
the image and the `CameraInfo` message which contains the calibration
parameters. We convert the `Image` message to an OpenCV `IplImage`.
Review
[Using CvBridge to Convert Between ROS Images and OpenCV Images](https://index.ros.org/p/cv_bridge)
for details. Here we fill our `PinholeCameraModel` using the
information from the `CameraInfo` message. Note that `cam_model_`
is a member variable rather than a local one. In the common case that
the camera parameters do not change, updating a previously filled camera
model is very cheap. For each requested frame ID, we query
[tf](https://index.ros.org/p/tf) for the position of that frame in the
optical frame of the camera (`cam_model_.tfFrame()`, pulled out of
the `CameraInfo` header). The operations here are described in
[Learning about tf and time](https://index.ros.org/p/tf).

Notice that for maximal accuracy, we look up the transform at the time
the image was acquired by the camera rather than `ros::Time::now()` or
`ros::Time(0)` (i.e. the most recent transform). For this example we
assume the camera is streaming at 30fps, so we are willing to wait no
more than 1/30s for the transform to become available. We extract the
origin of the requested frame in the camera optical frame and store it
in a `cv::Point3d`. Then we use our
`image_geometry::PinholeCameraModel` to project that point into image
coordinates.

We assume here that we are listening on an *undistorted* image topic.
The raw images from the camera driver are typically distorted,
preventing a clean projective mapping from 3D world coordinates to 2D
image coordinates; characterizing this distortion is an important part
of [camera calibration](https://index.ros.org/p/camera_calibration).
image_geometry includes methods for
undistorting images, but in ROS this is typically performed by an
[image_proc](https://index.ros.org/p/image_proc) or
[stereo_image_proc](https://index.ros.org/p/stereo_image_proc) node
sitting between the camera driver and your vision processing node. See
the [image_pipeline](https://index.ros.org/p/image_pipeline) for
details. Finally, we draw a filled red circle at the position of the
frame and print its ID centered above it. For more information see the
[OpenCV drawing functions](http://opencv.willowgarage.com/documentation/c/drawing_functions.html).
Finally, we convert the OpenCV `IplImage` to a ROS `Image` message
and publish it.

### Writing the launch file

For convenience, we'll make a
[launch file](https://index.ros.org/p/roslaunch) to start our processing
node and a couple instances of
[image_view](https://index.ros.org/p/image_view).

#### The launch file



```wiki
'''PR2 Beta Workshop Participants'''

Make sure to use the PR2 launch file below to run off live data from your robot.

If you want to try out the bag provided in this tutorial, ''you must use a separate roscore from your robot's''. Be careful!
```



Open a new file `draw.launch` and place inside it:

<<SeeSaw("+bag/-robot", Bag)>>`   ` <<SeeSaw("+robot/-bag",
PR2, show=True)>> 

```
{#!wiki seesaw bag
'''For use with the example bag file'''
{{{
#!xml block=launch
<launch>
  <param name="/use_sim_time" value="True" />

  <node name="draw_frames" pkg="learning_image_geometry" type="draw_frames" args="/base_link /base_laser_link /r_gripper_tool_frame" output="screen">
    <remap from="image" to="forearm_camera_r/image_rect_color" />
  </node>

  <node name="input_viewer" pkg="image_view" type="image_view">
    <remap from="image" to="forearm_camera_r/image_rect_color" />
    <param name="autosize" value="True" />
  </node>

  <node name="output_viewer" pkg="image_view" type="image_view">
    <remap from="image" to="image_out" />
    <param name="autosize" value="True" />
  </node>
</launch>
```

 }}}}



```
{#!wiki seesaw robot
{{{
'''For use with a PR2 robot:'''
#!xml block=launch_robot
<launch>
  <node name="draw_frames" pkg="learning_image_geometry" type="draw_frames" args="/base_link /base_laser_link /r_gripper_tool_frame" output="screen">
    <remap from="image" to="r_forearm_cam/image_rect_color" />
  </node>

  <node name="input_viewer" pkg="image_view" type="image_view">
    <remap from="image" to="r_forearm_cam/image_rect_color" />
    <param name="autosize" value="True" />
  </node>

  <node name="output_viewer" pkg="image_view" type="image_view">
    <remap from="image" to="image_out" />
    <param name="autosize" value="True" />
  </node>
</launch>
```

 }}}}

#### The launch file explained

This is a tricky bit. On a live system, ROS time is considered the same
as wall-clock time. For this test, however, we are running from a bag
file; the timestamps of the bagged messages are far in the past relative
to the current time. Therefore we need to tell all of our nodes to use
*simulated* time. At startup, each node checks the `/use_sim_time`
parameter; if it is set to true, they will listen on the `/clock`
topic (which [rosbag](https://index.ros.org/p/rosbag) will publish) for
the current ROS time. See [Clock](Clock) for details.

Especially when using [tf](https://index.ros.org/p/tf), trying to run a
node against a bag file without forcing simulated time is a common
source of issues. If you do this, tf will complain about timestamps far
in the past. Launch the `draw_frames` node we just wrote, remapping
`image` to the rectified image stream. It comes from the right forearm
camera on a PR2 robot. On a live system, the `image_rect_color` topic
is published by [image_proc](https://index.ros.org/p/image_proc) from
the `image_raw` topic published by the camera driver. Launch a viewer
for the unprocessed image stream. Launch a viewer for the annotated
output stream `/image_out` produced by our processing node.

### Building your node

Open your [CMakeLists.txt](https://index.ros.org/p/rosbuild) file and
append the following line at the bottom: 

```
rosbuild_add_executable(draw_frames draw_frames.cpp)
```



Then 

```
$ rosmake learning_image_geometry
```



### Trying it out

Make sure you have a `roscore` running, then launch your processing
and visualization nodes: 

```
$ roslaunch draw.launch
```



Now play back the recorded data: 

```
$ rosbag play plug_on_base.bag --clock
```





```wiki
This bag file is not compatible with ROS Groovy because the /camera_info topic has changed since the bag file was generated. You should get an error saying "Client [/draw_frames] wants topic /forearm_camera_r/camera_info to have datatype/md5sum"
```



The `\--clock` option tells [rosbag](https://index.ros.org/p/rosbag)
to publish simulated time on the `/clock` topic. Recall that this is
necessary to avoid confusing time-aware components like
[tf](https://index.ros.org/p/tf) when using bag files.

You should now see the annotated image in the "/image_out" window.

## Tutorials

- [Projecting a TF frame onto an image (C++)](#projecting-a-tf-frame-onto-an-image-c):
  This tutorial shows how to use
  image_geometry together with
  [tf](https://index.ros.org/p/tf) to project a tf frame onto a live
  image stream.
