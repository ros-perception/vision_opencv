Converting between ROS images and OpenCV images (C++)
-----------------------------------------------------

Overview
~~~~~~~~

This tutorial demonstrates how to interface ROS 2 and OpenCV by converting ROS images into OpenCV images, and vice versa, using ``cv_bridge``.
It includes a sample node that can be used as a template for your own node.

Prerequisites
~~~~~~~~~~~~~

This tutorial assumes you have a working knowledge of ROS 2.

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_.

1. Write the node
~~~~~~~~~~~~~~~~~

Here is a node that listens to an image topic, converts the images into ``cv::Mat``, draws a circle on them and publishes the result.

Let's assume you have already created an empty ROS 2 package for C++ named ``cv_bridge_tutorial``.
Create a new file ``src/image_converter.cpp`` inside your package, with the following content:

.. code-block:: C++

  #include <memory>

  #include "cv_bridge/cv_bridge.hpp"
  #include "opencv2/imgproc.hpp"
  #include "rclcpp/rclcpp.hpp"
  #include "sensor_msgs/image_encodings.hpp"
  #include "sensor_msgs/msg/image.hpp"

  class ImageConverter : public rclcpp::Node
  {
  public:
    ImageConverter()
    : Node("image_converter")
    {
      publisher_ = create_publisher<sensor_msgs::msg::Image>("image_out", 10);
      subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "image_in", 10,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {image_callback(msg);});
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      // Draw a red circle on the image
      if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) {
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0), 2);
      }

      // Convert the OpenCV image back to a ROS message and publish it
      publisher_->publish(*cv_ptr->toImageMsg());
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  };

  int main(int argc, char ** argv)
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConverter>());
    rclcpp::shutdown();
    return 0;
  }

2. Examine the code
~~~~~~~~~~~~~~~~~~~

.. code-block:: C++

  #include "cv_bridge/cv_bridge.hpp"
  #include "opencv2/imgproc.hpp"

``cv_bridge/cv_bridge.hpp`` declares the ``CvImage`` class and the conversion functions.
The OpenCV image processing module is included to draw on the image.

.. code-block:: C++

      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

Convert the ROS image message to a ``CvImage``, which contains the OpenCV image.

The node is going to draw on the image, so it needs a mutable copy of the data: this is what ``toCvCopy()`` returns.
The second argument is the encoding the node wants, ``bgr8``.
If the message has a different encoding, for example ``rgb8`` or ``mono8``, the image is converted.

You should always catch ``cv_bridge::Exception``, which is thrown when the conversion is not possible.
Without it, a message with an unsupported encoding would bring down the node.

.. code-block:: C++

      if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) {
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0), 2);
      }

``cv_ptr->image`` is a regular ``cv::Mat``, so any OpenCV function can be used with it.

.. code-block:: C++

      publisher_->publish(*cv_ptr->toImageMsg());

Convert the ``CvImage`` back to a ROS image message and publish it.
``toImageMsg()`` uses the ``header`` and the ``encoding`` of the ``CvImage``, which ``toCvCopy()`` filled in from the original message.

3. Update package.xml and CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add the dependencies to your ``package.xml``:

.. code-block:: xml

  <depend>cv_bridge</depend>
  <depend>libopencv-dev</depend>
  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>

Add the executable to your ``CMakeLists.txt``:

.. code-block:: cmake

  find_package(ament_cmake REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)

  add_executable(image_converter src/image_converter.cpp)
  target_compile_features(image_converter PUBLIC cxx_std_17)
  target_link_libraries(image_converter PRIVATE
    cv_bridge::cv_bridge
    opencv_core
    opencv_imgproc
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS})

  install(TARGETS image_converter
    DESTINATION lib/${PROJECT_NAME})

  ament_package()

4. Build and run
~~~~~~~~~~~~~~~~

Build your package from the root of your workspace and source it:

.. code-block:: console

  $ colcon build --packages-select cv_bridge_tutorial
  $ source install/setup.bash

To run the node, you need an image stream.
In a terminal, publish some images with ``cam2image`` from the ``image_tools`` package.
In burger mode it publishes synthetic images in the ``/image`` topic, so no camera is needed:

.. code-block:: console

  $ ros2 run image_tools cam2image --ros-args -p burger_mode:=true

In a second terminal, run the node, remapping its topics:

.. code-block:: console

  $ ros2 run cv_bridge_tutorial image_converter --ros-args -r image_in:=/image -r image_out:=/image_circle

In a third terminal, check that the node publishes the images:

.. code-block:: console

  $ ros2 topic list -t
  /image [sensor_msgs/msg/Image]
  /image_circle [sensor_msgs/msg/Image]
  ...

You can see the result, with the circle in the top left corner, with ``rqt_image_view``.
Select the ``/image_circle`` topic in its window:

.. code-block:: console

  $ ros2 run rqt_image_view rqt_image_view

5. Share the image data
~~~~~~~~~~~~~~~~~~~~~~~

The node above copies every image, because it modifies it.
A node that only reads the image can avoid the copy with ``toCvShare()``.

Create a new file ``src/image_brightness.cpp``, with a node that prints the mean value of the images:

.. code-block:: C++

  #include <memory>

  #include "cv_bridge/cv_bridge.hpp"
  #include "opencv2/core.hpp"
  #include "rclcpp/rclcpp.hpp"
  #include "sensor_msgs/image_encodings.hpp"
  #include "sensor_msgs/msg/image.hpp"

  class ImageBrightness : public rclcpp::Node
  {
  public:
    ImageBrightness()
    : Node("image_brightness")
    {
      subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "image_in", 10,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) {image_callback(msg);});
    }

  private:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
    {
      namespace enc = sensor_msgs::image_encodings;

      cv_bridge::CvImageConstPtr cv_ptr;
      try {
        if (enc::isColor(msg->encoding)) {
          cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        } else {
          cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
        }
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
        return;
      }

      // The image is read-only: it may share its data with the message
      const cv::Scalar mean = cv::mean(cv_ptr->image);
      RCLCPP_INFO(
        get_logger(), "%dx%d %s image, mean of the first channel: %.1f",
        cv_ptr->image.cols, cv_ptr->image.rows, cv_ptr->encoding.c_str(), mean[0]);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  };

  int main(int argc, char ** argv)
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageBrightness>());
    rclcpp::shutdown();
    return 0;
  }

.. code-block:: C++

      cv_bridge::CvImageConstPtr cv_ptr;
      try {
        if (enc::isColor(msg->encoding)) {
          cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        } else {
          cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
        }
      } catch (const cv_bridge::Exception & e) {

The node wants color if it is available, and it falls back to grayscale otherwise.
If the message is already ``bgr8`` or ``mono8``, the returned image points to the data of the message and nothing is copied.
If it has a different but convertible encoding, a new buffer is allocated and the image is converted.

``toCvShare()`` returns a pointer to a constant ``CvImage``: the image must not be modified, because its data may belong to the message.

Add the executable to your ``CMakeLists.txt`` in the same way as the first one, build the package and run it:

.. code-block:: console

  $ ros2 run cv_bridge_tutorial image_brightness --ros-args -r image_in:=/image
  [INFO] [image_brightness]: 320x240 bgr8 image, mean of the first channel: 28.5
  [INFO] [image_brightness]: 320x240 bgr8 image, mean of the first channel: 27.9

The time stamps of the log messages are omitted here.
