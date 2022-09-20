^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cv_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.2.1 (2022-09-21)
------------------
* silence dperecation warnings using boost macros (`#492 <https://github.com/ros-perception/vision_opencv/issues/492>`_)
* Contributors: Kenji Brameld

3.2.0 (2022-09-14)
------------------
* Add apache license and bsd license, because both are used. (`#483 <https://github.com/ros-perception/vision_opencv/issues/483>`_)
* Fix 16U encoding type (`#463 <https://github.com/ros-perception/vision_opencv/issues/463>`_)
* Reorganize author tag (`#469 <https://github.com/ros-perception/vision_opencv/issues/469>`_)
* Update maintainers (`#451 <https://github.com/ros-perception/vision_opencv/issues/451>`_)
* Fix ModuleNotFoundError: No module named 'cv_bridge' error (`#444 <https://github.com/ros-perception/vision_opencv/issues/444>`_)
* Make python3-opencv from test_depend to depend tag in package.xml (`#439 <https://github.com/ros-perception/vision_opencv/issues/439>`_)
* Contributors: Daisuke Nishimatsu, Kenji Brameld, RachelRen05

3.0.3 (2022-04-01)
------------------
* Add missing dependency on rcpputils (`#427 <https://github.com/ros-perception/vision_opencv/issues/427>`_)
* Contributors: Shane Loretz

3.0.2 (2022-01-27)
------------------
* Minor cleanups to the ROS 2 branch (`#418 <https://github.com/ros-perception/vision_opencv/issues/418>`_)
* Find Python3 if not on Android (`#423 <https://github.com/ros-perception/vision_opencv/issues/423>`_)
* Forward ports from noetic (`#420 <https://github.com/ros-perception/vision_opencv/issues/420>`_)
  * Add header arg to cv2_to_imgmsg (`#326 <https://github.com/ros-perception/vision_opencv/issues/326>`_)
  * prevent conversion of single channel 16bit integer images to/from colour (`#412 <https://github.com/ros-perception/vision_opencv/issues/412>`_)
* Contributors: Chris Lalancette, Christian Rauch, Michael Carroll, Naoya Yamaguchi, Shane Loretz

3.0.1 (2022-01-25)
------------------
* Use Boost::python3 if Boost < 1.67 (`#422 <https://github.com/ros-perception/vision_opencv/issues/422>`_)
* Use Boost::pythonXY target (`#421 <https://github.com/ros-perception/vision_opencv/issues/421>`_)
* Contributors: Shane Loretz

3.0.0 (2022-01-19)
------------------
* Export Modern CMake targets and install includes to another folder(`#419 <https://github.com/ros-perception/vision_opencv/issues/419>`_)
* Handle padded img msg in Python (`#400 <https://github.com/ros-perception/vision_opencv/issues/400>`_)
* Remove boost endian (`#399 <https://github.com/ros-perception/vision_opencv/issues/399>`_)
* Add conversions from YUV422 YUY2 (`#396 <https://github.com/ros-perception/vision_opencv/issues/396>`_)
* fix endianness comparison (`#397 <https://github.com/ros-perception/vision_opencv/issues/397>`_)
* Export sensor_msgs dependency (`#392 <https://github.com/ros-perception/vision_opencv/issues/392>`_)
* Fix multiple undefined references (`#370 <https://github.com/ros-perception/vision_opencv/issues/370>`_)
* Updated `cv_bridge.dll` install location. (`#356 <https://github.com/ros-perception/vision_opencv/issues/356>`_)
* Contributors: Homalozoa X, Jacob Perron, Patrick Musau, Sean Yen, Shane Loretz

2.2.1 (2020-07-16)
------------------
* Align module.hpp with noetic (`#342 <https://github.com/ros-perception/vision_opencv/issues/342>`_)
* Contributors: Michael Carroll

2.2.0 (2020-05-27)
------------------
* Disable portable image tests (`#337 <https://github.com/ros-perception/vision_opencv/issues/337>`_)
* use more specific boost rosdep keys (`#319 <https://github.com/ros-perception/vision_opencv/issues/319>`_)
* add opencv4 into the dependency list for ROS2 (`#324 <https://github.com/ros-perception/vision_opencv/issues/324>`_)
* Corrected the typo of Boost_INCLUDE_DIRS (`#315 <https://github.com/ros-perception/vision_opencv/issues/315>`_)
* use target include directories (`#313 <https://github.com/ros-perception/vision_opencv/issues/313>`_)
* Contributors: Karsten Knese, Lewis Liu, Michael Carroll, Mikael Arguedas, Sean Yen

2.1.3 (2019-10-23)
------------------
* populate array.array directly instead of converting to and from a string (`#305 <https://github.com/ros-perception/vision_opencv/issues/305>`_)
* include Boost to fix Windows build (`#290 <https://github.com/ros-perception/vision_opencv/issues/290>`_)
* Export interfaces for Win32 Shared Lib (`#301 <https://github.com/ros-perception/vision_opencv/issues/301>`_)
* Contributors: Dirk Thomas, Jonathan Noyola, Sean Yen

2.1.2 (2019-05-30)
------------------
* Suppress Boost Python warning. (`#279 <https://github.com/ros-perception/vision_opencv/issues/279>`_)
* silence unused return value warnings (`#276 <https://github.com/ros-perception/vision_opencv/issues/276>`_)
* Contributors: Karsten Knese, Michael Carroll

2.1.1 (2019-04-26)
------------------
* Replace ament commands with colcon (`#263 <https://github.com/ros-perception/vision_opencv/issues/263>`_)
  Latest ros2 repo does not include ament_tools. Executing ament command results in error.
* Contributors: Lalit Begani

2.1.0 (2018-11-26)
------------------
* find boost_python on boost 1.58 (<1.65) (`#246 <https://github.com/ros-perception/vision_opencv/issues/246>`_)
* Contributors: Mikael Arguedas

2.0.5 (2018-08-17)
------------------
* remove redundant ament_auto_lint dependency to release package
* fix test_encode_decode_cv2_compressed runtime error with JPEG2000
* change python3-numpy as build and execution dependency to fix building
  error while releasing package
* Contributor: Ethan Gao

2.0.4 (2018-08-14)
------------------
* Add libopencv-dev as dependency of cv_bridge
* Replace assert_equal with assert to remove dependency to nose package
* Contributor: Ethan Gao

2.0.3 (2018-08-07)
------------------
* fix the build warning with colcon building
* optimize buffer type of imgmsg_to_cv2()
* add python3_opencv and python3_numpy as dependency
* uncrustify 0.67 coding style alignment
* Contributors: Mikael Arguedas, Lars Berscheid, Ethan Gao

2.0.2 (2018-06-29)
------------------
* fix rosdep keys issue of opencv3


2.0.1 (2018-06-28)
------------------

2.0.0 (2018-06-06)
-------------------
* Port cv_bridge to ros2
* Remove the old inherent boost files
* Replace boost code with newer C++ standard higher than C++11
* Add a new readme
* Adapt message definition like sensor_msg/Image etc in ROS2
* Coding style alignment with ROS2 using cppcheck/cpplint and python flake8/pep257
* Enable all the C++/Python unit test with gtest and unittest with ament tools
* Contributors: Ethan Gao

1.12.7 (2017-11-12)
-------------------

1.12.6 (2017-11-11)
-------------------
* fix endianness issues
* Contributors: Vincent Rabaud

1.12.5 (2017-11-05)
-------------------
* add version_gte for opencv3
  @vrabaud If you'll update opencv3 version as discussed in https://discourse.ros.org/t/opencv-3-3/2674/4, I think we'd better to add 'version_gte' tag so that apt-get install ros-kinetic-cv-bridge also pulls openv3.3 from repository, to avoid API breaking issue between opencv2 and opencv3.
* Simplify the dependency components of cv_bridge
  Fixes `#183 <https://github.com/ros-perception/vision_opencv/issues/183>`_
* Fixes `#177 <https://github.com/ros-perception/vision_opencv/issues/177>`_
  The Python bridge was wrong on OpenCV2 with mono8 (and any Mat
  with only two dimensions btw). Took the official Python bridge
  from OpenCV.
* Add missing test file
  This fixes `#171 <https://github.com/ros-perception/vision_opencv/issues/171>`_
* Properly deal with alpha in image compression.
  That fixes `#169 <https://github.com/ros-perception/vision_opencv/issues/169>`_
* Silence warnings about un-used variables
* export OpenCV variables
* Contributors: Kei Okada, Victor Lamoine, Vincent Rabaud

1.12.4 (2017-01-29)
-------------------
* properly find Boost Python 2 or 3
  This fixes `#158 <https://github.com/ros-perception/vision_opencv/issues/158>`_
* Contributors: Vincent Rabaud

1.12.3 (2016-12-04)
-------------------
* Use api in sensor_msgs to get byte_depth and num channels
* Implement cpp conversion of N channel image
  This is cpp version of https://github.com/ros-perception/vision_opencv/pull/141,
  which is one for python.
* Fill black color to depth nan region
* address gcc6 build error in cv_bridge and tune
  With gcc6, compiling fails with `stdlib.h: No such file or directory`,
  as including '-isystem /usr/include' breaks with gcc6, cf.,
  https://gcc.gnu.org/bugzilla/show_bug.cgi?id=70129
  This commit addresses this issue for cv_bridge in the same way
  it was done in the commit ead421b8 [1] for image_geometry.
  This issue was also addressed in various other ROS packages.
  A list of related commits and pull requests is at:
  https://github.com/ros/rosdistro/issues/12783
  [1] https://github.com/ros-perception/vision_opencv/commit/ead421b85eeb750cbf7988657015296ed6789bcf
  Signed-off-by: Lukas Bulwahn <lukas.bulwahn@oss.bmw-carit.de>
* cv_bridge: Add missing test_depend on numpy
* Contributors: Kentaro Wada, Lukas Bulwahn, Maarten de Vries

1.12.2 (2016-09-24)
-------------------
* Specify background label when colorizing label image
* Adjust to arbitrary image channels like 32FC40
  Proper fix for `#141 <https://github.com/ros-perception/vision_opencv/issues/141>`_
* Remove unexpectedly included print statement
* Contributors: Kentaro Wada, Vincent Rabaud

1.12.1 (2016-07-11)
-------------------
* split the conversion tests out of enumerants
* support is_bigendian in Python
  Fixes `#114 <https://github.com/ros-perception/vision_opencv/issues/114>`_
  Also fixes mono16 test
* Support compressed Images messages in python for indigo
  - Add cv2_to_comprssed_imgmsg: Convert from cv2 image to compressed image ros msg.
  - Add comprssed_imgmsg_to_cv2:   Convert the compress message to a new image.
  - Add compressed image tests.
  - Add time to msgs (compressed and regular).
  add enumerants test for compressed image.
  merge the compressed tests with the regular ones.
  better comment explanation. I will squash this commit.
  Fix indentation
  fix typo mistage: from .imgmsg_to_compressed_cv2 to .compressed_imgmsg_to_cv2.
  remove cv2.CV_8UC1
  remove rospy and time depndency.
  change from IMREAD_COLOR to IMREAD_ANYCOLOR.
  - make indentaion of 4.
  - remove space trailer.
  - remove space from empty lines.
  - another set of for loops, it will make things easier to track. In that new set,  just have the number of channels in ([],1,3,4) (ignore two for jpg). from: https://github.com/ros-perception/vision_opencv/pull/132#discussion_r66721943
  - keep the OpenCV error message. from: https://github.com/ros-perception/vision_opencv/pull/132#discussion_r66721013
  add debug print for test.
  add case for 4 channels in test.
  remove 4 channels case from compressed test.
  add debug print for test.
  change typo of format.
  fix typo in format. change from dip to dib.
  change to IMREAD_ANYCOLOR as python code. (as it should).
  rename TIFF to tiff
  Sperate the tests one for regular images and one for compressed.
  update comment
* Add CvtColorForDisplayOptions with new colormap param
* fix doc jobs
* Add python binding for cv_bridge::cvtColorForDisplay
* Don't colorize float image as label image
  This is a bug and image whose encoding is other than 32SC1 should not be
  colorized. (currently, depth images with 32FC1 is also colorized.)
* Fix compilation of cv_bridge with opencv3 and python3.
* Contributors: Kentaro Wada, Maarten de Vries, Vincent Rabaud, talregev

1.12.0 (2016-03-18)
-------------------
* depend on OpenCV3 only
* Contributors: Vincent Rabaud

1.11.12 (2016-03-10)
--------------------
* Fix my typo
* Remove another eval
  Because `cvtype2_to_dtype_with_channels('8UCimport os; os.system("rm -rf /")')` should never have a chance of happening.
* Remove eval, and other fixes
  Also, extend from object, so as not to get a python 2.2-style class, and use the new-style raise statement
* Contributors: Eric Wieser

1.11.11 (2016-01-31)
--------------------
* clean up the doc files
* fix a few warnings in doc jobs
* Contributors: Vincent Rabaud

1.11.10 (2016-01-16)
--------------------
* fix OpenCV3 build
* Describe about converting label to bgr image in cvtColorForDisplay
* Convert label to BGR image to display
* Add test for rgb_colors.cpp
* Add rgb_colors util
* Update doc for converting to BGR in cvtColorForDisplay
* Convert to BGR from any encoding
* Refactor: sensor_msgs::image_encodings -> enc
* Contributors: Kentaro Wada, Vincent Rabaud

1.11.9 (2015-11-29)
-------------------
* deal with endianness
* add cvtColorForDisplay
* Improved efficiency by using toCvShare instead of toCvCopy.
* Add format enum for easy use and choose format.
* fix compilation warnings
* start to extend the cv_bridge with cvCompressedImage class, that will convert from cv::Mat opencv images to CompressedImage ros messages and vice versa
* Contributors: Carlos Costa, Vincent Rabaud, talregev

1.11.8 (2015-07-15)
-------------------
* Simplify some OpenCV3 distinction
* fix tests
* fix test under OpenCV3
* Remove Python for Android
* Contributors: Gary Servin, Vincent Rabaud

1.11.7 (2014-12-14)
-------------------
* check that the type is indeed a Numpy one
  This is in response to `#51 <https://github.com/ros-perception/vision_opencv/issues/51>`_
* Contributors: Vincent Rabaud

1.11.6 (2014-11-16)
-------------------
* chnage the behavior when there is only one channel
* cleanup tests
* Contributors: Vincent Rabaud

1.11.5 (2014-09-21)
-------------------
* get code to work with OpenCV3
  actually fixes `#46 <https://github.com/ros-perception/vision_opencv/issues/46>`_ properly
* Contributors: Vincent Rabaud

1.11.4 (2014-07-27)
-------------------
* Fix `#42 <https://github.com/ros-perception/vision_opencv/issues/42>`_
* Contributors: Libor Wagner

1.11.3 (2014-06-08)
-------------------
* Correct dependency from non-existent package to cv_bridge
* Contributors: Isaac Isao Saito

1.11.2 (2014-04-28)
-------------------
* Add depend on python for cv_bridge
* Contributors: Scott K Logan

1.11.1 (2014-04-16)
-------------------
* fixes `#34 <https://github.com/ros-perception/vision_opencv/issues/34>`_
* Contributors: Vincent Rabaud

1.11.0 (2014-02-15)
-------------------
* remove deprecated API and fixes `#33 <https://github.com/ros-perception/vision_opencv/issues/33>`_
* fix OpenCV dependencies
* Contributors: Vincent Rabaud

1.10.15 (2014-02-07)
--------------------
* fix python 3 error at configure time
* Contributors: Dirk Thomas

1.10.14 (2013-11-23 16:17)
--------------------------
* update changelog
* Find NumPy include directory
* Contributors: Brian Jensen, Vincent Rabaud

1.10.13 (2013-11-23 09:19)
--------------------------
* fix compilation on older NumPy
* Contributors: Vincent Rabaud

1.10.12 (2013-11-22)
--------------------
* bump changelog
* Fixed issue with image message step size
* fix crash for non char data
* fix `#26 <https://github.com/ros-perception/vision_opencv/issues/26>`_
* Contributors: Brian Jensen, Vincent Rabaud

1.10.11 (2013-10-23)
--------------------
* fix bad image check and improve it too
* Contributors: Vincent Rabaud

1.10.10 (2013-10-19)
--------------------
* fixes `#25 <https://github.com/ros-perception/vision_opencv/issues/25>`_
* Contributors: Vincent Rabaud

1.10.9 (2013-10-07)
-------------------
* fixes `#20 <https://github.com/ros-perception/vision_opencv/issues/20>`_
* Contributors: Vincent Rabaud

1.10.8 (2013-09-09)
-------------------
* fixes `#22 <https://github.com/ros-perception/vision_opencv/issues/22>`_
* fixes `#17 <https://github.com/ros-perception/vision_opencv/issues/17>`_
* check for CATKIN_ENABLE_TESTING
* fixes `#16 <https://github.com/ros-perception/vision_opencv/issues/16>`_
* update email  address
* Contributors: Lukas Bulwahn, Vincent Rabaud

1.10.7 (2013-07-17)
-------------------

1.10.6 (2013-03-01)
-------------------
* make sure conversion are applied for depth differences
* Contributors: Vincent Rabaud

1.10.5 (2013-02-11)
-------------------

1.10.4 (2013-02-02)
-------------------
* fix installation of the boost package
* Contributors: Vincent Rabaud

1.10.3 (2013-01-17)
-------------------
* Link against PTYHON_LIBRARIES
* Contributors: William Woodall

1.10.2 (2013-01-13)
-------------------
* use CATKIN_DEVEL_PREFIX instead of obsolete CATKIN_BUILD_PREFIX
* Contributors: Dirk Thomas

1.10.1 (2013-01-10)
-------------------
* add licenses
* fixes `#5 <https://github.com/ros-perception/vision_opencv/issues/5>`_ by removing the logic from Python and using wrapped C++ and adding a test for it
* fix a bug discovered when running the opencv_tests
* use some C++ logic
* add a Boost Python module to have the C++ logix used directly in Python
* Contributors: Vincent Rabaud

1.10.0 (2013-01-03)
-------------------
* add conversion from Bayer to gray
* Contributors: Vincent Rabaud

1.9.15 (2013-01-02)
-------------------
* use the reverted isColor behavior
* Contributors: Vincent Rabaud

1.9.14 (2012-12-30)
-------------------

1.9.13 (2012-12-15)
-------------------
* use the catkin macros for the setup.py
* fix `#3 <https://github.com/ros-perception/vision_opencv/issues/3>`_
* Contributors: Vincent Rabaud

1.9.12 (2012-12-14)
-------------------
* buildtool_depend catkin fix
* CMakeLists.txt clean up.
* Contributors: William Woodall

1.9.11 (2012-12-10)
-------------------
* fix issue `#1 <https://github.com/ros-perception/vision_opencv/issues/1>`_
* Cleanup of package.xml
* Contributors: Vincent Rabaud, William Woodall

1.9.10 (2012-10-04)
-------------------
* fix the bad include folder
* Contributors: Vincent Rabaud

1.9.9 (2012-10-01)
------------------
* fix dependencies
* Contributors: Vincent Rabaud

1.9.8 (2012-09-30)
------------------
* fix some dependencies
* add rosconsole as a dependency
* fix missing Python at install and fix some dependencies
* Contributors: Vincent Rabaud

1.9.7 (2012-09-28 21:07)
------------------------
* add missing stuff
* make sure we find catkin
* Contributors: Vincent Rabaud

1.9.6 (2012-09-28 15:17)
------------------------
* move the test to where it belongs
* fix the tests and the API to not handle conversion from CV_TYPE to Color type (does not make sense)
* comply to the new Catkin API
* backport the YUV422 bug fix from Fuerte
* apply patch from https://code.ros.org/trac/ros-pkg/ticket/5556
* Contributors: Vincent Rabaud

1.9.5 (2012-09-15)
------------------
* remove dependencies to the opencv2 ROS package
* Contributors: Vincent Rabaud

1.9.4 (2012-09-13)
------------------
* make sure the include folders are copied to the right place
* Contributors: Vincent Rabaud

1.9.3 (2012-09-12)
------------------

1.9.2 (2012-09-07)
------------------
* be more compliant to the latest catkin
* added catkin_project() to cv_bridge, image_geometry, and opencv_tests
* Contributors: Jonathan Binney, Vincent Rabaud

1.9.1 (2012-08-28 22:06)
------------------------
* remove things that were marked as ROS_DEPRECATED
* Contributors: Vincent Rabaud

1.9.0 (2012-08-28 14:29)
------------------------
* catkinized opencv_tests by Jon Binney
* catkinized cv_bridge package... others disable for now by Jon Binney
* remove the version check, let's trust OpenCV :)
* revert the removal of opencv2
* vision_opencv: Export OpenCV flags in manifests for image_geometry, cv_bridge.
* finally get rid of opencv2 as it is a system dependency now
* bump REQUIRED version of OpenCV to 2.3.2, which is what's in ros-fuerte-opencv
* switch rosdep name to opencv2, to refer to ros-fuerte-opencv2
* added missing header
* Added constructor to CvImage to make converting a cv::Mat to sensor_msgs::Image less verbose.
* cv_bridge: Added unit test for `#5206 <https://github.com/ros-perception/vision_opencv/issues/5206>`_
* cv_bridge: Applied patch from mdesnoyer to fix handling of non-continuous OpenCV images. `#5206 <https://github.com/ros-perception/vision_opencv/issues/5206>`_
* Adding opencv2 to all manifests, so that client packages may
  not break when using them.
* baking in opencv debs and attempting a pre-release
* cv_bridge: Support for new 16-bit encodings.
* cv_bridge: Deprecate old C++ cv_bridge API.
* cv_bridge: Correctly scale for MONO8 <-> MONO16 conversions.
* cv_bridge: Fixed issue where pointer version to toCvCopy would ignore the requested encoding (http://answers.ros.org/question/258/converting-kinect-rgb-image-to-opencv-gives-wrong).
* fixed doc build by taking a static snapshot
* cv_bridge: Marking doc reviewed.
* cv_bridge: Tweaks to make docs look better.
* cv_bridge: Added cvtColor(). License notices. Documented that CvBridge class is obsolete.
* cv_bridge: Added redesigned C++ cv_bridge.
* Doc cleanup
* Trigger doc rebuild
* mono16 -> bgr conversion tested and fixed in C
* Added Ubuntu platform tags to manifest
* Handle mono16 properly
* Raise exception when imgMsgToCv() gets an image encoding it does not recognise, `#3489 <https://github.com/ros-perception/vision_opencv/issues/3489>`_
* Remove use of deprecated rosbuild macros
* Fixed example
* cv_bridge split from opencv2
* Contributors: Vincent Rabaud, ethanrublee, gerkey, jamesb, mihelich, vrabaud, wheeler
