Publishing cv::Mat images with type adaptation (C++)
----------------------------------------------------

Overview
~~~~~~~~

With `type adaptation <https://docs.ros.org/en/rolling/How-To-Guides/Defining-custom-type-adaptations.html>`_, a node can publish and subscribe to its own data type instead of a ROS message.
``cv_bridge`` provides an adapter between ``cv::Mat`` images and ``sensor_msgs/msg/Image``:

* The node works with ``cv::Mat`` directly, and the conversion to a message is done by ROS 2 only when it is needed.
* When the publisher and the subscriber are in the same process, with intra-process communication enabled, the image is passed without converting or copying it.
* For the rest of the system the topic is a regular ``sensor_msgs/msg/Image`` topic.

This tutorial demonstrates how to use the adapter with a publisher and a subscriber that run in the same process.

Prerequisites
~~~~~~~~~~~~~

This tutorial assumes you have completed the :doc:`Converting-Images-Cpp` tutorial, and it reuses its ``cv_bridge_tutorial`` package.

1. Write the code
~~~~~~~~~~~~~~~~~

Create a new file ``src/type_adapter_example.cpp`` inside your package, with the following content:

.. code-block:: C++

  #include <chrono>
  #include <memory>
  #include <utility>

  #include "cv_bridge/cv_mat_sensor_msgs_image_type_adapter.hpp"
  #include "opencv2/core.hpp"
  #include "opencv2/imgproc.hpp"
  #include "rclcpp/rclcpp.hpp"
  #include "sensor_msgs/msg/image.hpp"
  #include "std_msgs/msg/header.hpp"

  RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
    cv_bridge::ROSCvMatContainer,
    sensor_msgs::msg::Image);

  class CvMatPublisher : public rclcpp::Node
  {
  public:
    explicit CvMatPublisher(const rclcpp::NodeOptions & options)
    : Node("cv_mat_publisher", options)
    {
      publisher_ = create_publisher<cv_bridge::ROSCvMatContainer>("image", 10);
      timer_ = create_wall_timer(std::chrono::milliseconds(500), [this]() {publish_frame();});
    }

  private:
    void publish_frame()
    {
      // A blue image with a white circle that moves with every frame
      cv::Mat frame(240, 320, CV_8UC3, cv::Scalar(255, 0, 0));
      cv::circle(frame, cv::Point(20 + (count_++ * 10) % 280, 120), 15, cv::Scalar::all(255), -1);

      std_msgs::msg::Header header;
      header.stamp = now();
      header.frame_id = "camera";

      auto container = std::make_unique<cv_bridge::ROSCvMatContainer>(std::move(frame), header);
      RCLCPP_INFO(
        get_logger(), "Publishing an image with data at %p",
        static_cast<const void *>(container->cv_mat().data));
      publisher_->publish(std::move(container));
    }

    int count_ {0};
    rclcpp::Publisher<cv_bridge::ROSCvMatContainer>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
  };

  class CvMatSubscriber : public rclcpp::Node
  {
  public:
    explicit CvMatSubscriber(const rclcpp::NodeOptions & options)
    : Node("cv_mat_subscriber", options)
    {
      subscription_ = create_subscription<cv_bridge::ROSCvMatContainer>(
        "image", 10,
        [this](std::unique_ptr<cv_bridge::ROSCvMatContainer> container) {
          const cv::Mat & frame = container->cv_mat();
          RCLCPP_INFO(
            get_logger(), "Received a %dx%d image with data at %p",
            frame.cols, frame.rows, static_cast<const void *>(frame.data));
        });
    }

  private:
    rclcpp::Subscription<cv_bridge::ROSCvMatContainer>::SharedPtr subscription_;
  };

  int main(int argc, char ** argv)
  {
    rclcpp::init(argc, argv);

    // Both nodes run in the same process, with intra-process communication enabled
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    auto publisher = std::make_shared<CvMatPublisher>(options);
    auto subscriber = std::make_shared<CvMatSubscriber>(options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(publisher);
    executor.add_node(subscriber);
    executor.spin();

    rclcpp::shutdown();
    return 0;
  }

2. Examine the code
~~~~~~~~~~~~~~~~~~~

.. code-block:: C++

  #include "cv_bridge/cv_mat_sensor_msgs_image_type_adapter.hpp"

This header declares ``cv_bridge::ROSCvMatContainer``, the type that is published, and its adapter.

.. code-block:: C++

  RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
    cv_bridge::ROSCvMatContainer,
    sensor_msgs::msg::Image);

Declare that ``ROSCvMatContainer`` is used in place of ``sensor_msgs::msg::Image``.
After this, publishers and subscriptions can be created with ``ROSCvMatContainer`` as their type.

.. code-block:: C++

      publisher_ = create_publisher<cv_bridge::ROSCvMatContainer>("image", 10);

The publisher is created with the container type.
The topic it advertises is a ``sensor_msgs/msg/Image`` topic.

.. code-block:: C++

      auto container = std::make_unique<cv_bridge::ROSCvMatContainer>(std::move(frame), header);
      ...
      publisher_->publish(std::move(container));

A ``ROSCvMatContainer`` holds a ``cv::Mat`` and the header of the message.
Here the image is moved into the container, and the container is moved into ``publish()``, so the data is never copied.

The encoding of the message is deduced from the type of the ``cv::Mat``: ``mono8`` for ``CV_8UC1`` and ``bgr8`` for ``CV_8UC3``.
For any other type, or to use a different encoding, give the encoding as the fourth argument of the constructor (the third one is the endianness of the data):

.. code-block:: C++

  cv_bridge::ROSCvMatContainer container(depth_image, header, false, "32FC1");

.. code-block:: C++

      subscription_ = create_subscription<cv_bridge::ROSCvMatContainer>(
        "image", 10,
        [this](std::unique_ptr<cv_bridge::ROSCvMatContainer> container) {
          const cv::Mat & frame = container->cv_mat();

The subscription is also created with the container type, and its callback receives the container.
``cv_mat()`` gives access to the image.

.. note::

   The ``cv::Mat`` may reference data that is owned by the container.
   If you need the image after the callback has returned, keep the container, or make a deep copy of the image with ``cv::Mat::clone()``.

.. code-block:: C++

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);

Intra-process communication has to be enabled in the nodes to pass the images without copying them.

3. Add the executable to CMakeLists.txt
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add the dependency on ``std_msgs`` to your ``package.xml``:

.. code-block:: xml

  <depend>std_msgs</depend>

And the new executable to your ``CMakeLists.txt``:

.. code-block:: cmake

  find_package(std_msgs REQUIRED)

  add_executable(type_adapter_example src/type_adapter_example.cpp)
  target_compile_features(type_adapter_example PUBLIC cxx_std_17)
  target_link_libraries(type_adapter_example PRIVATE
    cv_bridge::cv_bridge
    opencv_core
    opencv_imgproc
    rclcpp::rclcpp
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS})

  install(TARGETS type_adapter_example
    DESTINATION lib/${PROJECT_NAME})

4. Build and run
~~~~~~~~~~~~~~~~

Build your package from the root of your workspace, source it and run the executable:

.. code-block:: console

  $ colcon build --packages-select cv_bridge_tutorial
  $ source install/setup.bash
  $ ros2 run cv_bridge_tutorial type_adapter_example
  [INFO] [cv_mat_publisher]: Publishing an image with data at 0x7b9c0414a040
  [INFO] [cv_mat_subscriber]: Received a 320x240 image with data at 0x7b9c0414a040
  [INFO] [cv_mat_publisher]: Publishing an image with data at 0x5a817756e800
  [INFO] [cv_mat_subscriber]: Received a 320x240 image with data at 0x5a817756e800

The time stamps of the log messages are omitted here, and the addresses are different in every run.
The subscriber receives the same address that the publisher printed: the image has not been copied.

From another terminal the topic is a regular image topic, which any other node can subscribe to.
For those subscribers the image is converted to a message:

.. code-block:: console

  $ ros2 topic list -t
  /image [sensor_msgs/msg/Image]
  ...
  $ ros2 topic echo --once /image --field encoding
  bgr8
  ---
