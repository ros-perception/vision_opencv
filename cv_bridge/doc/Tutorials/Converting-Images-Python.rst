Converting between ROS images and OpenCV images (Python)
--------------------------------------------------------

Overview
~~~~~~~~

This tutorial demonstrates how to interface ROS 2 and OpenCV by converting ROS images into OpenCV images, and vice versa, using ``cv_bridge``.
It includes a sample node that can be used as a template for your own node.

Prerequisites
~~~~~~~~~~~~~

This tutorial assumes you have a working knowledge of ROS 2.

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/ROS-Framework/client-libraries/Working-with-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/ROS-Framework/client-libraries/Working-with-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_.

1. Write the node
~~~~~~~~~~~~~~~~~

Here is a node that listens to an image topic, converts the images into OpenCV images, draws a circle on them and publishes the result.

Let's assume you have already created an empty ROS 2 package for Python named ``cv_bridge_tutorial``.
Create a new file ``cv_bridge_tutorial/image_converter.py`` inside your package, with the following content:

.. code-block:: python

  import cv2
  from cv_bridge import CvBridge, CvBridgeError
  import rclpy
  from rclpy.node import Node
  from sensor_msgs.msg import Image


  class ImageConverter(Node):

      def __init__(self):
          super().__init__('image_converter')
          self.bridge = CvBridge()
          self.publisher = self.create_publisher(Image, 'image_out', 10)
          self.subscription = self.create_subscription(
              Image, 'image_in', self.image_callback, 10)

      def image_callback(self, msg):
          try:
              cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
          except CvBridgeError as e:
              self.get_logger().error(f'cv_bridge exception: {e}')
              return

          # Draw a red circle on the image
          rows, cols, _ = cv_image.shape
          if rows > 60 and cols > 60:
              cv2.circle(cv_image, (50, 50), 10, (0, 0, 255), 2)

          # Convert the OpenCV image back to a ROS message and publish it
          try:
              out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8', header=msg.header)
          except CvBridgeError as e:
              self.get_logger().error(f'cv_bridge exception: {e}')
              return
          self.publisher.publish(out_msg)


  def main():
      rclpy.init()
      rclpy.spin(ImageConverter())
      rclpy.shutdown()


  if __name__ == '__main__':
      main()

2. Examine the code
~~~~~~~~~~~~~~~~~~~

.. code-block:: python

  from cv_bridge import CvBridge, CvBridgeError

``CvBridge`` is the class that does the conversions, and ``CvBridgeError`` is the exception it raises when a conversion is not possible.

.. code-block:: python

          self.bridge = CvBridge()

Create one ``CvBridge`` object and use it for all the images.

.. code-block:: python

          try:
              cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
          except CvBridgeError as e:
              self.get_logger().error(f'cv_bridge exception: {e}')
              return

Convert the ROS image message to an OpenCV image, which is a NumPy array in Python.

``desired_encoding`` is the encoding the node wants, ``bgr8``.
If the message has a different encoding, for example ``rgb8`` or ``mono8``, the image is converted.
With the default value, ``'passthrough'``, the image keeps the encoding of the message.

.. code-block:: python

          rows, cols, _ = cv_image.shape
          if rows > 60 and cols > 60:
              cv2.circle(cv_image, (50, 50), 10, (0, 0, 255), 2)

``cv_image`` is a regular NumPy array, so any OpenCV function can be used with it.
A ``bgr8`` image has three dimensions: rows, columns and channels.

.. code-block:: python

              out_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8', header=msg.header)

Convert the OpenCV image back to a ROS image message.

``cv2_to_imgmsg()`` does not convert the image.
The ``encoding`` argument tells the subscribers of the message how to interpret the data, so that they know whether the image is RGB or BGR.
It must be compatible with the type of the array: a three-channel 8-bit array can be ``bgr8`` or ``rgb8``, but not ``mono8``.

The ``header`` argument copies the time stamp and the frame of the original message.

3. Update package.xml and setup.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add the dependencies to your ``package.xml``:

.. code-block:: xml

  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>python3-opencv</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>

Update the ``console_scripts`` section of your ``setup.py``:

.. code-block:: python

    entry_points={
        'console_scripts': [
            'image_converter = cv_bridge_tutorial.image_converter:main',
        ],
    },

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
