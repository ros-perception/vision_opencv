INSTALL_DIR = opencv

all: installed

SVN_DIR = build/opencvpython-svn
SVN_URL = https://opencvpython.svn.sourceforge.net/svnroot/opencvpython
include $(shell rospack find mk)/svn_checkout.mk

installed: $(SVN_DIR)
	-rm -rf src
	mkdir -p src
	cd $(SVN_DIR) && \
			   OPENCV_ROOT=$(PWD)/../opencv_latest/opencv python setup.py build_ext --rpath $(PWD)/../opencv_latest/opencv/lib && \
			   OPENCV_ROOT=$(PWD)/../opencv_latest/opencv python setup.py install --prefix ../../src 
	mv src/lib/python2.?/site-packages/* src/ 
	rm -rf src/bin src/lib
	touch installed
clean:
	rm -rf $(SVN_DIR)

wipe: clean
	rm -rf build patched

.PHONY : clean download

