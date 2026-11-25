Using the camera model (Python)
-------------------------------

Overview
~~~~~~~~

This tutorial demonstrates the main operations of the ``image_geometry`` camera models: projecting 3D points to pixels, projecting pixels to rays, removing the lens distortion and computing depth from stereo disparity.

It does not need a camera: the calibration is created in the code.
In a real node it comes from the ``camera_info`` topic of the camera.

Prerequisites
~~~~~~~~~~~~~

This tutorial assumes you have a working knowledge of ROS 2 and of Python.

If you have not done so already `create a workspace <https://docs.ros.org/en/rolling/ROS-Framework/client-libraries/Working-with-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html>`_ and `create a package <https://docs.ros.org/en/rolling/ROS-Framework/client-libraries/Working-with-Client-Libraries/Creating-Your-First-ROS2-Package.html>`_.

1. Write the script
~~~~~~~~~~~~~~~~~~~

Let's assume you have already created an empty ROS 2 package for Python named ``image_geometry_tutorial_py``.
Create a new file ``image_geometry_tutorial_py/camera_model_example.py`` inside your package, with the following content:

.. code-block:: python

  import cv2
  from image_geometry import PinholeCameraModel, StereoCameraModel
  import numpy as np
  from sensor_msgs.msg import CameraInfo


  def make_camera_info(tx=0.0):
      """Return the calibration of a 640x480 camera with some radial distortion."""
      info = CameraInfo()
      info.header.frame_id = 'camera_optical_frame'
      info.width, info.height = 640, 480
      info.distortion_model = 'plumb_bob'
      info.d = [-0.2, 0.05, 0.0, 0.0, 0.0]
      info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
      info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
      info.p = [500.0, 0.0, 320.0, tx, 0.0, 500.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
      return info


  def main():
      model = PinholeCameraModel()
      model.from_camera_info(make_camera_info())

      print('Frame:', model.get_tf_frame())
      print('Resolution:', model.full_resolution())
      print(f'fx = {model.fx()}, fy = {model.fy()}, cx = {model.cx()}, cy = {model.cy()}')
      print(f'Field of view: {np.degrees(model.fov_x()):.1f} x {np.degrees(model.fov_y()):.1f} deg')

      # Project a 3D point, in the optical frame of the camera, to a pixel
      point = (0.2, -0.1, 1.0)
      u, v = model.project_3d_to_pixel(point)
      print(f'Point {point} -> pixel ({u:.1f}, {v:.1f})')

      # Project the pixel back to a ray: a unit vector in the direction of the point
      ray = model.project_pixel_to_3d_ray((u, v))
      print('Ray through the pixel:', [round(float(c), 4) for c in ray])
      print('Ray scaled to z = 1:', [round(float(c / ray[2]), 4) for c in ray])

      # Remove the lens distortion from a pixel of the raw image
      raw = (600.0, 450.0)
      rect = model.rectify_point(raw)
      print(f'Raw pixel {raw} -> rectified pixel ({rect[0]:.1f}, {rect[1]:.1f})')

      # Remove the lens distortion from a whole image
      raw_image = np.zeros((480, 640), np.uint8)
      cv2.circle(raw_image, (600, 450), 5, 255, -1)
      rectified_image = np.zeros_like(raw_image)
      model.rectify_image(raw_image, rectified_image)
      ys, xs = np.nonzero(rectified_image > 128)
      print(f'Spot in the rectified image at ({xs.mean():.1f}, {ys.mean():.1f})')

      # Size in pixels of an object 0.1 m wide at 2 m from the camera
      print(f'0.1 m at 2 m is {model.get_delta_u(0.1, 2.0):.1f} pixels wide')

      # A stereo pair with a baseline of 0.1 m
      stereo = StereoCameraModel()
      stereo.from_camera_info(make_camera_info(), make_camera_info(tx=-500.0 * 0.1))
      disparity = stereo.get_disparity(2.0)
      depth = stereo.get_z(disparity)
      print(f'Disparity at 2 m: {disparity:.1f} pixels, depth for it: {depth:.1f} m')
      xyz = stereo.project_pixel_to_3d((420.0, 190.0), disparity)
      print('Pixel (420, 190) with that disparity ->', [round(float(c), 3) for c in xyz])


  if __name__ == '__main__':
      main()

2. Examine the code
~~~~~~~~~~~~~~~~~~~

.. code-block:: python

  def make_camera_info(tx=0.0):

This function creates the calibration of a 640x480 camera with a focal length of 500 pixels, the principal point in the center of the image and some radial distortion (``d``).
A calibrated camera driver publishes the same information in its ``camera_info`` topic.

.. code-block:: python

      model = PinholeCameraModel()
      model.from_camera_info(make_camera_info())

Create a ``PinholeCameraModel`` and fill it with the calibration.
In a node, call ``from_camera_info()`` with every ``CameraInfo`` message you receive, in case the calibration changes.

.. code-block:: python

      point = (0.2, -0.1, 1.0)
      u, v = model.project_3d_to_pixel(point)

``project_3d_to_pixel()`` projects a 3D point to the pixel where it appears in the *rectified* image.
The point is in the optical frame of the camera, in meters: 0.2 m to the right, 0.1 m up (``y`` points down) and 1 m in front of the camera.

.. code-block:: python

      ray = model.project_pixel_to_3d_ray((u, v))

``project_pixel_to_3d_ray()`` does the opposite: it returns the direction of the ray that passes through the pixel, as a unit vector.
A pixel does not determine the depth of a point, only its direction: every point of the ray projects to the same pixel.
Scaling the ray to ``z = 1`` gives back the original point, which was at a depth of 1 m.

.. code-block:: python

      raw = (600.0, 450.0)
      rect = model.rectify_point(raw)

``rectify_point()`` converts a pixel of the *raw* image, which contains the lens distortion, to the rectified image.
Use it with the features you detect in a raw image before projecting them to 3D.

.. code-block:: python

      rectified_image = np.zeros_like(raw_image)
      model.rectify_image(raw_image, rectified_image)

``rectify_image()`` removes the distortion from a whole image.
It writes the result into the second argument, which must be an array of the same size and type as the input.

To demonstrate it, the raw image has a white spot at the pixel ``(600, 450)``.
In the rectified image the spot moves to the same position that ``rectify_point()`` computed.

.. code-block:: python

      print(f'0.1 m at 2 m is {model.get_delta_u(0.1, 2.0):.1f} pixels wide')

``get_delta_u()`` returns the width in pixels of an object of a given width at a given depth.
``get_delta_x()`` does the opposite, and ``get_delta_v()`` and ``get_delta_y()`` do the same in the vertical direction.

.. code-block:: python

      stereo = StereoCameraModel()
      stereo.from_camera_info(make_camera_info(), make_camera_info(tx=-500.0 * 0.1))

A ``StereoCameraModel`` is created from the calibration of the left and the right camera.
The fourth element of the projection matrix of the right camera, ``Tx``, encodes the baseline: it is ``-fx * baseline``, here for a baseline of 0.1 m.

``get_disparity()`` and ``get_z()`` convert between depth and disparity, and ``project_pixel_to_3d()`` computes the 3D point of a pixel of the left image with a given disparity.

3. Update package.xml and setup.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add the dependencies to your ``package.xml``:

.. code-block:: xml

  <exec_depend>image_geometry</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <exec_depend>python3-opencv</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>

Update the ``console_scripts`` section of your ``setup.py``:

.. code-block:: python

    entry_points={
        'console_scripts': [
            'camera_model_example = image_geometry_tutorial_py.camera_model_example:main',
        ],
    },

4. Build and run
~~~~~~~~~~~~~~~~

Build your package from the root of your workspace, source it and run the script:

.. code-block:: console

  $ colcon build --packages-select image_geometry_tutorial_py
  $ source install/setup.bash
  $ ros2 run image_geometry_tutorial_py camera_model_example
  Frame: camera_optical_frame
  Resolution: (640, 480)
  fx = 500.0, fy = 500.0, cx = 320.0, cy = 240.0
  Field of view: 65.2 x 51.3 deg
  Point (0.2, -0.1, 1.0) -> pixel (420.0, 190.0)
  Ray through the pixel: [0.1952, -0.0976, 0.9759]
  Ray scaled to z = 1: [0.2, -0.1, 1.0]
  Raw pixel (600.0, 450.0) -> rectified pixel (632.3, 474.2)
  Spot in the rectified image at (632.2, 474.1)
  0.1 m at 2 m is 25.0 pixels wide
  Disparity at 2 m: 25.0 pixels, depth for it: 2.0 m
  Pixel (420, 190) with that disparity -> [0.4, -0.2, 2.0]
