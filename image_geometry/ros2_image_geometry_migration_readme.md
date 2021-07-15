
**This file describes work done and steps to perform test on the image_geometry package for ROS2 (Bouncy & Crystal).**

**Overview:**

	image_geometry is a package for interpreting images geometrically.

**List of changes from ROS1 to ROS2:**

	The changes has been done by following the Migration guide for ROS2( https://index.ros.org/doc/ros2/Migration-Guide/ ).

	1. Migrated directed.py python tests to ROS2
	2. Enabled pytests in image_geometry package.
	3. Migrated cameramodels.py, directed.py, CMakelists.txt & package.xml files for ROS2.
	4. Merged Approved changes from PR #257.
	5. Followed cosmetics rules and updated the files with copyright, pep257 and flake8 rules.

**Pre-requisites:**

	1. System should have installed Crystal/Bouncy distro. Check out installation instructions and tutorials https://index.ros.org/doc/ros2/.
	2. System should have checkout ros2 vision_opencv pkg & build (Refer Steps below).

		Steps to checkout vision_opencv package:
			1. Open the new terminal and run below commands.
			2. mkdir -p vision_opencv_ws/src
			3. cd ~/vision_opencv_ws/src
			4. git clone git@github.com:akhileshmoghe/vision_opencv.git
			#git branch -ra //This command will show different branches of main git on #kinetic
			5. cd vision_opencv/
			6. git checkout -b ros2-devel remotes/origin/ros2-devel

		Steps to Build vision_opencv package:
			1. cd ~/vision_opencv_ws/
			2.  for Crystal:
						source /opt/ros/crystal/setup.bash
					for Bouncy:
						source /opt/ros/bouncy/setup.bash
			3. cd ~/vision_opencv_ws/src/vision_opencv
			4. colcon build

	4. System should have both ROS1 & ROS2. Note: We have verified on ROS1(i.e. melodic) & ROS2 (i.e. Bouncy & Crystal).


**Future Work:-**

	1. Make Cpp code for image_geometry package to be completely cosmetic rules compliant.


**TESTING:-**

	Run the test cases as follows:-
	cd ~/vision_opencv_ws_crystal/src/vision_opencv/
	colcon test


