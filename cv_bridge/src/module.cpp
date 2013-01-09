#include <boost/python.hpp>
#include <cv_bridge/cv_bridge.h>

BOOST_PYTHON_MODULE(cv_bridge_boost)
{
  // Wrap the function to get encodings as OpenCV types
  boost::python::def("getCvType", cv_bridge::getCvType);
}
