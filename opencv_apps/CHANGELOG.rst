^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package opencv_apps
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.11 (2016-01-31)
--------------------
* check if opencv_contrib is available
* Use respawn instead of watch
* Contributors: Kei Okada, trainman419

1.11.10 (2016-01-16)
--------------------
* enable simple_flow on opencv3, https://github.com/ros-perception/vision_opencv/commit/8ed5ff5c48b4c3d270cd8216175cf6a8441cb046 can revert https://github.com/ros-perception/vision_opencv/commit/89a933aef7c11acdb75a17c46bfcb60389b25baa
* lk_flow_nodeletcpp, fback_flow_nodelet.cpp: need to copy input image to gray
* opencv_apps: add test programs, this will generate images for wiki
* fix OpenCV3 build
* phase_corr: fix display, bigger circle and line
* goodfeature_track_nodelet.cpp: publish good feature points as corners
* set image encoding to bgr8
* convex_hull: draw hull with width 4
* watershed_segmentatoin_nodelet.cpp: output segmented area as contours and suppot add_seed_points as input of segmentatoin seed
* src/nodelet/segment_objects_nodelet.cpp: change output image topic name from segmented to image
* Convert rgb image to bgr in lk_flow
* [oepncv_apps] fix bug for segment_objects_nodelet.cpp
* Contributors: Kei Okada, Kentaro Wada, Shingo Kitagawa, Vincent Rabaud

1.11.9 (2015-11-29)
-------------------
* Accept grayscale images as input as well
* Add format enum for easy use and choose format.
* Contributors: Felix Mauch, talregev

1.11.8 (2015-07-15)
-------------------
* simplify the OpenCV3 compatibility
* fix image output of fback_flow
* fix error: ISO C++ forbids initialization of member for gcc 4.6
* add std_srvs
* add std_srvs
* fix error: ISO C++ forbids initialization of member for gcc 4.6
* get opencv_apps to compile with OpenCV3
* fix licenses for Kei
* add opencv_apps, proposed in `#40 <https://github.com/ros-perception/vision_opencv/issues/40>`_
* Contributors: Kei Okada, Vincent Rabaud, Yuto Inagaki
