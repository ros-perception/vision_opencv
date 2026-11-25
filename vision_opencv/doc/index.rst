vision_opencv
=============

``vision_opencv`` is a metapackage that installs the packages used to interface ROS 2 with `OpenCV <https://opencv.org/>`_, the computer vision library.
It contains no code itself; it only depends on:

.. list-table::
   :header-rows: 1
   :widths: 20 50 30

   * - Package
     - Use it to
     - Documentation
   * - ``cv_bridge``
     - Convert between ``sensor_msgs/msg/Image`` (and ``CompressedImage``) messages and OpenCV images (``cv::Mat`` in C++, NumPy arrays in Python).
     - `cv_bridge <https://docs.ros.org/en/rolling/p/cv_bridge/>`_
   * - ``image_geometry``
     - Interpret ``sensor_msgs/msg/CameraInfo`` messages: project 3D points to pixels and back, rectify images and points, and work with stereo pairs.
     - `image_geometry <https://docs.ros.org/en/rolling/p/image_geometry/>`_

.. toctree::
   :maxdepth: 2

   self
   Package information </standards>

Which package do I need?
------------------------

* You want the **pixels** of an image, to process them with OpenCV or to publish an image you computed: use ``cv_bridge``.
* You want to relate pixels to the **3D world** with the camera calibration, for example to draw a 3D point on an image or to compute the ray behind a pixel: use ``image_geometry``.
* Many nodes need both, for example to draw on an image the position of a TF frame.

Installation
------------

Install both packages with the metapackage:

.. code-block:: console

   $ sudo apt install ros-rolling-vision-opencv

or install only the one you need, for example ``ros-rolling-cv-bridge``.

.. note::

   In the ``package.xml`` of your own packages, depend on ``cv_bridge`` and/or ``image_geometry`` directly rather than on ``vision_opencv``, so you only pull in what you use.

Quick example
-------------

A Python callback that receives an image and its camera calibration, converts the image to OpenCV, and draws the point that is 1 m in front of the camera:

.. code-block:: python

   import cv2
   from cv_bridge import CvBridge
   from image_geometry import PinholeCameraModel

   bridge = CvBridge()
   camera_model = PinholeCameraModel()


   def callback(image_msg, camera_info_msg):
       image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
       camera_model.from_camera_info(camera_info_msg)

       u, v = camera_model.project_3d_to_pixel((0.0, 0.0, 1.0))
       cv2.circle(image, (int(u), int(v)), 5, (0, 0, 255), -1)

       return bridge.cv2_to_imgmsg(image, encoding='bgr8', header=image_msg.header)

The equivalent C++ API is ``cv_bridge::toCvCopy()`` (in ``cv_bridge/cv_bridge.hpp``) and ``image_geometry::PinholeCameraModel`` (in ``image_geometry/pinhole_camera_model.hpp``).
See the documentation of each package for the details and for tutorials.

Links
-----

* `Source code and issue tracker <https://github.com/ros-perception/vision_opencv>`_
* `OpenCV documentation <https://docs.opencv.org/4.x/>`_
