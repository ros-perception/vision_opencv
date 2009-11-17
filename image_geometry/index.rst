image_geometry
==============

image_geometry simplifies interpreting images geometrically using the
parameters from sensor_msgs/CameraInfo. Although CameraInfo contains all
the information required to rectify a raw image and project points onto
it, actually performing these operations correctly over the space of all
camera options can be non-trivial. The camera parameters in CameraInfo
are for a full-resolution image; region-of-interest alone significantly
complicates the creation of rectification maps and requires adjusting
the projection matrix. Adding options such as subsampling (binning) to
CameraInfo would further complicate the correct interpretation of the
corresponding Images. By using image_geometry, users will both simplify
and future-proof their code.

Indices and tables
==================

* :ref:`genindex`
* :ref:`search`

