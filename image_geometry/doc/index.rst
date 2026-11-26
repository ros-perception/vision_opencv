image_geometry
==============

``image_geometry`` contains C++ and Python libraries for interpreting images geometrically, using the calibration of the camera from a ``sensor_msgs/msg/CameraInfo`` message.
With it you can project 3D points onto the image, find the ray that corresponds to a pixel, remove the lens distortion from images and points, and compute depth from stereo disparity.

``CameraInfo`` contains all the information required to rectify a raw image and to project points onto it, but **it is highly recommended that you use this library**, since performing these operations correctly over the space of all camera options can be non-trivial.
For example, the camera parameters in ``CameraInfo`` are for the full-resolution image, so a region of interest or binning require adjusting the projection matrix and the rectification maps.
``image_geometry`` handles them for you.

The classes use the `OpenCV camera model <https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html>`_, and they give access to the camera matrices in the format expected by OpenCV.

.. toctree::
   :maxdepth: 2

   self
   tutorials
   references
   Package information </standards>

1. Camera models
----------------

``PinholeCameraModel``
  A monocular camera, modelled as an ideal pinhole camera plus lens distortion.

``StereoCameraModel``
  A stereo pair of pinhole cameras, made from the ``CameraInfo`` of the left and the right camera.

A camera model is filled from the ``CameraInfo`` messages with ``fromCameraInfo()`` (``from_camera_info()`` in Python), and it gives read-only access to the camera parameters.
It is meant to be updated in the callback of every image, like ``cv_bridge``:

.. code-block:: C++

   image_geometry::PinholeCameraModel model_;  // a member of the node

   void image_callback(
     const sensor_msgs::msg::Image::ConstSharedPtr & image,
     const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info)
   {
     // Update the camera model (usually a no-op)
     model_.fromCameraInfo(info);

     // Do processing...
   }

Updating the model is cheap when the calibration does not change.
In C++, ``fromCameraInfo()`` returns ``true`` only if the calibration, the binning or the region of interest changed, and the rectification maps are computed again only in that case.

2. Coordinate frames
--------------------

The 3D points are expressed in the *optical* frame of the camera, whose name is given by the ``frame_id`` of the ``CameraInfo`` (``tfFrame()``, or ``get_tf_frame()`` in Python):

* ``x`` points to the right of the image,
* ``y`` points down,
* ``z`` points forward, along the optical axis.

Use ``tf2`` to transform points from or to the other frames of the robot, see the :doc:`Tutorials/Projecting-A-TF-Frame-Cpp` tutorial.

The pixel coordinates are ``(u, v)``, where ``u`` is the column and ``v`` is the row, starting at the top left corner of the image.

3. Raw and rectified images
---------------------------

Camera drivers publish *raw* images, which contain the distortion of the lens.
Most operations of ``image_geometry`` work with *rectified* pixel coordinates, where the distortion has been removed and the camera behaves as an ideal pinhole camera:

* ``project3dToPixel()`` returns rectified coordinates, and ``projectPixelTo3dRay()`` expects them.
* ``rectifyPoint()`` and ``unrectifyPoint()`` convert single pixels between raw and rectified coordinates.
* ``rectifyImage()`` removes the distortion from a whole image.

In a ROS 2 system the images are usually rectified by an ``image_proc`` node from the `image_pipeline <https://github.com/ros-perception/image_pipeline>`_ between the camera driver and your node, so your node can subscribe to the rectified image topic (like ``image_rect``) and project points onto it directly.

The supported distortion models are ``plumb_bob``, ``rational_polynomial`` and ``equidistant`` (fisheye).
If all the distortion coefficients are zero, the raw and the rectified images are the same.
In C++, rectifying with any other model throws ``image_geometry::Exception``.

.. warning::

   The Python ``PinholeCameraModel`` does not support the ``equidistant`` model: it treats it as ``plumb_bob``, which gives wrong results for ``rectify_point()`` and ``rectify_image()``.
   Use the C++ library for fisheye cameras.

4. Main operations
------------------

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - C++
     - Python
     - Description
   * - ``project3dToPixel()``
     - ``project_3d_to_pixel()``
     - Project a 3D point to rectified pixel coordinates.
   * - ``projectPixelTo3dRay()``
     - ``project_pixel_to_3d_ray()``
     - Return the ray through a rectified pixel. See the note below.
   * - ``rectifyPoint()``, ``unrectifyPoint()``
     - ``rectify_point()``
     - Convert a pixel between raw and rectified coordinates.
   * - ``rectifyImage()``, ``unrectifyImage()``
     - ``rectify_image()``
     - Remove the distortion from an image, or apply it.
   * - ``getDeltaU()``, ``getDeltaV()``
     - ``get_delta_u()``, ``get_delta_v()``
     - Size in pixels of a length at a given depth.
   * - ``getDeltaX()``, ``getDeltaY()``
     - ``get_delta_x()``, ``get_delta_y()``
     - Length at a given depth of a size in pixels.
   * - ``fx()``, ``fy()``, ``cx()``, ``cy()``, ``Tx()``, ``Ty()``
     - ``fx()``, ``fy()``, ``cx()``, ``cy()``, ``tx()``, ``ty()``
     - Parameters of the projection matrix.
   * - ``fovX()``, ``fovY()``
     - ``fov_x()``, ``fov_y()``
     - Field of view, in radians.
   * - ``intrinsicMatrix()``, ``distortionCoeffs()``, ``rotationMatrix()``, ``projectionMatrix()``
     - ``intrinsic_matrix()``, ``distortion_coeffs()``, ``rotation_matrix()``, ``projection_matrix()``
     - Matrices ``K``, ``D``, ``R`` and ``P`` in the format expected by OpenCV.

.. note::

   ``projectPixelTo3dRay()`` returns a ray scaled to ``z = 1`` in C++, but ``project_pixel_to_3d_ray()`` returns a unit vector in Python.
   Both point in the same direction.

For a stereo camera, ``StereoCameraModel`` also provides:

* ``getDisparity()`` and ``getZ()`` (``get_disparity()`` and ``get_z()`` in Python) to convert between depth and disparity,
* ``projectDisparityTo3d()`` (``project_pixel_to_3d()`` in Python) to compute the 3D point of a pixel of the left image with a given disparity,
* ``projectDisparityImageTo3d()`` (C++ only) to convert a whole disparity image to a point cloud.

5. Binning and region of interest
---------------------------------

When the ``CameraInfo`` has binning or a region of interest, the images are smaller than the full resolution at which the camera was calibrated.
The camera models adjust their projection matrix, so all the operations work with the pixel coordinates of the images that are actually published.

In C++, ``fullResolution()`` is the calibrated resolution, ``reducedResolution()`` is the resolution of the published images, and ``toFullResolution()`` and ``toReducedResolution()`` convert pixels between both.

6. Using the libraries
----------------------

In C++, add the dependency to your ``package.xml``:

.. code-block:: xml

   <depend>image_geometry</depend>

and link against the exported target in your ``CMakeLists.txt``:

.. code-block:: cmake

   find_package(image_geometry REQUIRED)

   add_executable(my_node src/my_node.cpp)
   target_link_libraries(my_node PRIVATE image_geometry::image_geometry)

The classes are declared in ``image_geometry/pinhole_camera_model.hpp`` and ``image_geometry/stereo_camera_model.hpp``.

In Python, add ``<exec_depend>image_geometry</exec_depend>`` to your ``package.xml`` and import the classes from the ``image_geometry`` module:

.. code-block:: python

   from image_geometry import PinholeCameraModel, StereoCameraModel

The Python methods with camelCase names, like ``fromCameraInfo()``, are deprecated: use the snake_case names, like ``from_camera_info()``.
