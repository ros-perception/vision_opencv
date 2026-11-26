# image_geometry

`image_geometry` contains C++ and Python libraries for interpreting images geometrically, using the calibration of the camera from a [sensor_msgs/msg/CameraInfo](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/CameraInfo.html) message.
It does for the calibration what `cv_bridge` does for the images: it interfaces `CameraInfo` with OpenCV.

`CameraInfo` contains all the information required to rectify a raw image and to project points onto it, but performing these operations correctly over the space of all camera options (distortion models, binning, region of interest) is non-trivial, so it is highly recommended that you use this library.

## Usage

### C++

```cpp
#include "image_geometry/pinhole_camera_model.hpp"

image_geometry::PinholeCameraModel model;
model.fromCameraInfo(camera_info_msg);  // cheap if the calibration did not change

// 3D point in the optical frame of the camera -> rectified pixel
cv::Point2d uv = model.project3dToPixel(cv::Point3d(0.2, -0.1, 1.0));

// Rectified pixel -> ray, scaled to z = 1
cv::Point3d ray = model.projectPixelTo3dRay(uv);

// Raw (distorted) pixel -> rectified pixel
cv::Point2d uv_rect = model.rectifyPoint(cv::Point2d(600, 450));
```

```cmake
find_package(image_geometry REQUIRED)
target_link_libraries(my_target PRIVATE image_geometry::image_geometry)
```

### Python

```python
from image_geometry import PinholeCameraModel

model = PinholeCameraModel()
model.from_camera_info(camera_info_msg)

u, v = model.project_3d_to_pixel((0.2, -0.1, 1.0))
ray = model.project_pixel_to_3d_ray((u, v))  # a unit vector
u_rect, v_rect = model.rectify_point((600.0, 450.0))
```

`StereoCameraModel` does the same for a stereo pair, and converts between disparity and depth.

## Documentation

The full documentation, including tutorials and the API reference, is in the [doc](https://github.com/ros-perception/vision_opencv/tree/rolling/image_geometry/doc) directory, and it is published at [docs.ros.org](https://docs.ros.org/en/rolling/p/image_geometry/).
