# opencv rev. 1978
INSTALL_DIR = opencv

all: installed

SVN_DIR = build/opencv-svn
SVN_URL = https://opencvlibrary.svn.sourceforge.net/svnroot/opencvlibrary/tags/latest_tested_snapshot/opencv
#SVN_PATCH = cmake_2.4.6.patch
#SVN_PATCH = calibration.patch
SVN_PATCH = osx.patch
include $(shell rospack find mk)/svn_checkout.mk



CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=RELEASE \
             -D CMAKE_INSTALL_PREFIX=`rospack find opencv_latest`/$(INSTALL_DIR) \
             -D BUILD_PYTHON_SUPPORT=OFF \
						 -D WITH_FFMPEG=OFF \
						 -D WITH_XINE=OFF

installed: $(SVN_DIR) patched
	mkdir -p $(SVN_DIR)/build
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	@echo "patch opencv.pc to pass -Wl,-rpath,-L{libdir}"
	-mv `rospack find opencv_latest`/$(INSTALL_DIR)/lib/pkgconfig/opencv.pc $(CURDIR)/$(INSTALL_DIR)/lib/pkgconfig/opencv.bak
	sed 's%Libs: -L$${libdir} -lcxcore%Libs: -Wl,-rpath,$${libdir} -L$${libdir} -lcxcore%g' `rospack find opencv_latest`/$(INSTALL_DIR)/lib/pkgconfig/opencv.bak > `rospack find opencv_latest`/$(INSTALL_DIR)/lib/pkgconfig/opencv.pc
	touch installed

clean:
	-cd $(SVN_DIR)/build && make clean
	rm -rf $(INSTALL_DIR) installed

wipe: clean
	rm -rf build patched

.PHONY : clean download
