INSTALL_DIR = opencv

all: installed

SVN_DIR = build/opencv-svn
SVN_URL = https://code.ros.org/svn/opencv/trunk/opencv
SVN_REVISION = -r3842
SVN_PATCH = pythontest.patch
include $(shell rospack find mk)/svn_checkout.mk



CMAKE = cmake 
CMAKE_ARGS = -D CMAKE_BUILD_TYPE=RELEASE \
             -D CMAKE_INSTALL_PREFIX=`rospack find opencv2`/$(INSTALL_DIR) \
             -D BUILD_EXAMPLES=ON \
             -D BUILD_NEW_PYTHON_SUPPORT=ON \
             -D BUILD_SWIG_PYTHON_SUPPORT=OFF \
             -D WITH_FFMPEG=OFF \
             -D WITH_XINE=OFF \
             -D WITH_1394=OFF \
             -D BUILD_DOXYGEN_DOCS=OFF

installed: wiped $(SVN_DIR) patched
	mkdir -p $(SVN_DIR)/build
	cd $(SVN_DIR)/build && $(CMAKE) $(CMAKE_ARGS) ..
	cd $(SVN_DIR)/build && make $(ROS_PARALLEL_JOBS) && make install
	@echo "patch opencv.pc to pass -Wl,-rpath,-L{libdir}"
	-mv `rospack find opencv2`/$(INSTALL_DIR)/lib/pkgconfig/opencv.pc $(CURDIR)/$(INSTALL_DIR)/lib/pkgconfig/opencv.bak
	sed 's%Libs: -L$${libdir} -lcxcore%Libs: -Wl,-rpath,$${libdir} -L$${libdir} -lcxcore%g' `rospack find opencv2`/$(INSTALL_DIR)/lib/pkgconfig/opencv.bak > `rospack find opencv2`/$(INSTALL_DIR)/lib/pkgconfig/opencv.pc
	@echo "copy Python module to ./lib"
	mkdir -p $(CURDIR)/lib
	mv $(CURDIR)/$(INSTALL_DIR)/lib/python*/site-packages/* $(CURDIR)/lib/
	# chrpath -r `chrpath $(CURDIR)/lib/cv.so | sed 's!build/opencv-svn/build/lib!opencv/lib!;s/^.*=//'` $(CURDIR)/lib/cv.so
	touch opencv/include/opencv/cxtypes.h
	touch installed

test: installed
	python build/opencv-svn/tests/python/test.py

clean:
	-cd $(SVN_DIR)/build && make clean
	rm -rf $(INSTALL_DIR) installed

wiped: Makefile $(SVN_PATCH)
	make wipe
	touch wiped

wipe: clean
	rm -rf build patched lib

.PHONY : clean download
