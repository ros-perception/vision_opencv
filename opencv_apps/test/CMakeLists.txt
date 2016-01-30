# Tests simple calibration dataset
catkin_download_test_data(face_detector_withface_test_diamondback.bag http://download.ros.org/data/face_detector/face_detector_withface_test_diamondback.bag
  DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
  MD5 59126117e049e69d577b7ee27251a6f8
  )
catkin_download_test_data(vslam_tutorial.bag http://download.ros.org/data/vslam_system/vslam_tutorial.bag
  DESTINATION .
  MD5 f5aece448b7af00a38a993eb71400806
  )
add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/vslam_tutorial.bag
  DEPENDS vslam_tutorial.bag
  COMMAND rosbag reindex vslam_tutorial.bag --output-dir ${CMAKE_CURRENT_SOURCE_DIR}
)
add_custom_target(vslam_tutorial_bag DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/vslam_tutorial.bag)
add_dependencies(tests vslam_tutorial_bag)

#add_rostest(test-apps.test)
add_rostest(test-edge_detection.test ARGS gui:=false)
add_rostest(test-hough_lines.test ARGS gui:=false)
add_rostest(test-hough_circles.test ARGS gui:=false)
add_rostest(test-find_contours.test ARGS gui:=false)
add_rostest(test-convex_hull.test ARGS gui:=false)
add_rostest(test-general_contours.test ARGS gui:=false)
add_rostest(test-contour_moments.test ARGS gui:=false)
add_rostest(test-face_detection.test ARGS gui:=false)
add_rostest(test-goodfeature_track.test ARGS gui:=false)
add_rostest(test-camshift.test ARGS gui:=false)
add_rostest(test-fback_flow.test ARGS gui:=false)
add_rostest(test-lk_flow.test ARGS gui:=false)
add_rostest(test-people_detect.test ARGS gui:=false)
add_rostest(test-phase_corr.test ARGS gui:=false)
add_rostest(test-segment_objects.test ARGS gui:=false)
# simple flow requires opencv-contrib https://github.com/ros-perception/vision_opencv/issues/108
if(OPENCV_HAVE_OPTFLOW)
  add_rostest(test-simple_flow.test ARGS gui:=false)
endif()
add_rostest(test-watershed_segmentation.test ARGS gui:=false)


