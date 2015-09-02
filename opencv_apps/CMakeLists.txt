cmake_minimum_required(VERSION 2.8.3)
project(opencv_apps)

find_package(catkin REQUIRED COMPONENTS cv_bridge dynamic_reconfigure message_generation image_transport nodelet roscpp)

find_package(OpenCV REQUIRED)
if (OpenCV_VERSION VERSION_EQUAL "3")
  set(${PROJECT_NAME}_EXTRA_CFG)
  set(${PROJECT_NAME}_EXTRA_FILES)
else()
  set(${PROJECT_NAME}_EXTRA_CFG cfg/SimpleFlow.cfg)
  set(${PROJECT_NAME}_EXTRA_FILES src/nodelet/simple_flow_nodelet.cpp)
endif()

# generate the dynamic_reconfigure config file
generate_dynamic_reconfigure_options(
  cfg/EdgeDetection.cfg cfg/HoughLines.cfg cfg/HoughCircles.cfg
  cfg/FindContours.cfg cfg/ConvexHull.cfg cfg/GeneralContours.cfg cfg/ContourMoments.cfg
  cfg/FaceDetection.cfg
  cfg/GoodfeatureTrack.cfg
  #
  cfg/CamShift.cfg
  cfg/FBackFlow.cfg
  cfg/LKFlow.cfg
  cfg/PeopleDetect.cfg
  cfg/PhaseCorr.cfg
  cfg/SegmentObjects.cfg
  cfg/WatershedSegmentation.cfg
  ${${PROJECT_NAME}_EXTRA_CFG}
  )

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Point2D.msg
   Point2DStamped.msg
   Point2DArray.msg
   Point2DArrayStamped.msg
   Rect.msg
   RectArray.msg
   RectArrayStamped.msg
   Flow.msg
   FlowStamped.msg
   FlowArray.msg
   FlowArrayStamped.msg
   RectArrayStamped.msg
   Size.msg
   Face.msg
   FaceArray.msg
   FaceArrayStamped.msg
   Line.msg
   LineArray.msg
   LineArrayStamped.msg
   RotatedRect.msg
   RotatedRectStamped.msg
   RotatedRectArray.msg
   RotatedRectArrayStamped.msg
   Circle.msg
   CircleArray.msg
   CircleArrayStamped.msg
   Moment.msg
   MomentArray.msg
   MomentArrayStamped.msg
   Contour.msg
   ContourArray.msg
   ContourArrayStamped.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package()

include_directories( ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

## Declare a cpp library
add_library(${PROJECT_NAME} SHARED
  # https://github.com/Itseez/opencv/blob/2.4/samples/cpp/tutorial_code
  # ImgTrans
  src/nodelet/edge_detection_nodelet.cpp
  src/nodelet/hough_lines_nodelet.cpp
  src/nodelet/hough_circles_nodelet.cpp
  # ShapeDescriptors
  src/nodelet/find_contours_nodelet.cpp
  src/nodelet/convex_hull_nodelet.cpp
  src/nodelet/general_contours_nodelet.cpp
  src/nodelet/contour_moments_nodelet.cpp
  # objectDetection
  src/nodelet/face_detection_nodelet.cpp
  # TrackingMotion
  src/nodelet/goodfeature_track_nodelet.cpp
  # https://github.com/Itseez/opencv/blob/2.4/samples/cpp
  # 3calibration.cpp
  # bagofwords_classification.cpp
  # bgfg_gmg.cpp
  # bgfg_segm.cpp
  # brief_match_test.cpp
  # build3dmodel.cpp
  # calibration_artificial.cpp
  # calibration.cpp
  src/nodelet/camshift_nodelet.cpp
  src/nodelet/simple_example_nodelet.cpp
  src/nodelet/simple_compressed_example_nodelet.cpp
  # chamfer.cpp
  # connected_components.cpp
  # contours2.cpp
  # convexhull.cpp
  # cout_mat.cpp
  # delaunay2.cpp
  # demhist.cpp
  # descriptor_extractor_matcher.cpp
  # detection_based_tracker_sample.cpp
  # detector_descriptor_evaluation.cpp
  # detector_descriptor_matcher_evaluation.cpp
  # dft.cpp
  # distrans.cpp
  # drawing.cpp
  # edge.cpp
  # em.cpp
  # fabmap_sample.cpp
  # facerec_demo.cpp
  # facial_features.cpp
  src/nodelet/fback_flow_nodelet.cpp
  # ffilldemo.cpp
  # filestorage.cpp
  # fitellipse.cpp
  # freak_demo.cpp
  # gencolors.cpp
  # generic_descriptor_match.cpp
  # grabcut.cpp
  # houghcircles.cpp
  # houghlines.cpp
  # hybridtrackingsample.cpp
  # image.cpp
  # imagelist_creator.cpp
  # image_sequence.cpp
  # inpaint.cpp
  # intelperc_capture.cpp
  # kalman.cpp
  # kmeans.cpp
  # laplace.cpp
  # latentsvm_multidetect.cpp
  # letter_recog.cpp
  # linemod.cpp
  src/nodelet/lk_flow_nodelet.cpp
  # logpolar_bsm.cpp
  # matcher_simple.cpp
  # matching_to_many_images.cpp
  # meanshift_segmentation.cpp
  # minarea.cpp
  # morphology2.cpp
  # opencv_version.cpp
  # OpenEXRimages_HDR_Retina_toneMapping.cpp
  # OpenEXRimages_HDR_Retina_toneMapping_video.cpp
  # openni_capture.cpp
  # pca.cpp
  src/nodelet/people_detect_nodelet.cpp
  src/nodelet/phase_corr_nodelet.cpp
  # points_classifier.cpp
  # retinaDemo.cpp
  # rgbdodometry.cpp
  src/nodelet/segment_objects_nodelet.cpp
  ## select3dobj.cpp
  # src/nodelet/simple_flow_nodelet.cpp
  # squares.cpp
  # starter_imagelist.cpp
  # starter_video.cpp
  # stereo_calib.cpp
  # stereo_match.cpp
  # stitching.cpp
  # stitching_detailed.cpp
  # tvl1_optical_flow.cpp
  ## videocapture_pvapi.cpp
  ## video_dmtx.cpp
  # video_homography.cpp
  # videostab.cpp
  src/nodelet/watershed_segmentation_nodelet.cpp
  ${${PROJECT_NAME}_EXTRA_FILES}
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_executable(edge_detection_exe src/node/edge_detection.cpp)
SET_TARGET_PROPERTIES(edge_detection_exe PROPERTIES OUTPUT_NAME edge_detection)
target_link_libraries(edge_detection_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(hough_lines_exe src/node/hough_lines.cpp)
SET_TARGET_PROPERTIES(hough_lines_exe PROPERTIES OUTPUT_NAME hough_lines)
target_link_libraries(hough_lines_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(hough_circles_exe src/node/hough_circles.cpp)
SET_TARGET_PROPERTIES(hough_circles_exe PROPERTIES OUTPUT_NAME hough_circles)
target_link_libraries(hough_circles_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(find_contours_exe src/node/find_contours.cpp)
SET_TARGET_PROPERTIES(find_contours_exe PROPERTIES OUTPUT_NAME find_contours)
target_link_libraries(find_contours_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(convex_hull_exe src/node/convex_hull.cpp)
SET_TARGET_PROPERTIES(convex_hull_exe PROPERTIES OUTPUT_NAME convex_hull)
target_link_libraries(convex_hull_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(general_contours_exe src/node/general_contours.cpp)
SET_TARGET_PROPERTIES(general_contours_exe PROPERTIES OUTPUT_NAME general_contours)
target_link_libraries(general_contours_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(contour_moments_exe src/node/contour_moments.cpp)
SET_TARGET_PROPERTIES(contour_moments_exe PROPERTIES OUTPUT_NAME contour_moments)
target_link_libraries(contour_moments_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(face_detection_exe src/node/face_detection.cpp)
SET_TARGET_PROPERTIES(face_detection_exe PROPERTIES OUTPUT_NAME face_detection)
target_link_libraries(face_detection_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(goodfeature_track_exe src/node/goodfeature_track.cpp)
SET_TARGET_PROPERTIES(goodfeature_track_exe PROPERTIES OUTPUT_NAME goodfeature_track)
target_link_libraries(goodfeature_track_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

  # bgfg_gmg.cpp
  # bgfg_segm.cpp
  # calibration.cpp

add_executable(camshift_exe src/node/camshift.cpp)
SET_TARGET_PROPERTIES(camshift_exe PROPERTIES OUTPUT_NAME camshift)
target_link_libraries(camshift_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(simple_example_exe src/node/simple_example.cpp)
SET_TARGET_PROPERTIES(simple_example_exe PROPERTIES OUTPUT_NAME simple_example)
target_link_libraries(simple_example_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(simple_compressed_example_exe src/node/simple_compressed_example.cpp)
SET_TARGET_PROPERTIES(simple_compressed_example_exe PROPERTIES OUTPUT_NAME simple_compressed_example)
target_link_libraries(simple_compressed_example_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(fback_flow_exe src/node/fback_flow.cpp)
SET_TARGET_PROPERTIES(fback_flow_exe PROPERTIES OUTPUT_NAME fback_flow)
target_link_libraries(fback_flow_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

  # fback.cpp
  # hybridtrackingsample.cpp
  # image_sequence.cpp
  # intelperc_capture.cpp
  # laplace.cpp
  # linemod.cpp
  # lkdemo.cpp

add_executable(lk_flow_exe src/node/lk_flow.cpp)
SET_TARGET_PROPERTIES(lk_flow_exe PROPERTIES OUTPUT_NAME lk_flow)
target_link_libraries(lk_flow_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(people_detect_exe src/node/people_detect.cpp)
SET_TARGET_PROPERTIES(people_detect_exe PROPERTIES OUTPUT_NAME people_detect)
target_link_libraries(people_detect_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(phase_corr_exe src/node/phase_corr.cpp)
SET_TARGET_PROPERTIES(phase_corr_exe PROPERTIES OUTPUT_NAME phase_corr)
target_link_libraries(phase_corr_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

  # phase_corr.cpp
  # retinaDemo.cpp
  # segment_objects.cpp

add_executable(segment_objects_exe src/node/segment_objects.cpp)
SET_TARGET_PROPERTIES(segment_objects_exe PROPERTIES OUTPUT_NAME segment_objects)
target_link_libraries(segment_objects_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

  # select3dobj.cpp

add_executable(simple_flow_exe src/node/simple_flow.cpp)
SET_TARGET_PROPERTIES(simple_flow_exe PROPERTIES OUTPUT_NAME simple_flow)
target_link_libraries(simple_flow_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

  # starter_video.cpp
  # videocapture_pvapi.cpp
  # video_dmtx.cpp
  # video_homography.cpp

add_executable(watershed_segmentation_exe src/node/watershed_segmentation.cpp)
SET_TARGET_PROPERTIES(watershed_segmentation_exe PROPERTIES OUTPUT_NAME watershed_segmentation)
target_link_libraries(watershed_segmentation_exe ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


install(TARGETS edge_detection_exe hough_lines_exe hough_circles_exe face_detection_exe goodfeature_track_exe
  camshift_exe simple_example_exe simple_compressed_example_exe fback_flow_exe lk_flow_exe people_detect_exe phase_corr_exe segment_objects_exe watershed_segmentation_exe
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
