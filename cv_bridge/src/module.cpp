#include <iostream>
#include <boost/python.hpp>
#include <cv_bridge/cv_bridge.h>
#include <Python.h>

#include <opencv/cv.h>

PyObject *mod_opencv;

using namespace cv;
namespace bp = boost::python;

// These are sucky, sketchy versions of the real things in OpenCV Python,
// inferior in every way.

#define PYTHON_USE_NUMPY 0 // switch off for now...

struct cvmat_t {
  PyObject_HEAD
  CvMat *a;
  PyObject *data;
  size_t offset;
};

static int failmsg(const char *fmt, ...)
{
  char str[1000];

  va_list ap;
  va_start(ap, fmt);
  vsnprintf(str, sizeof(str), fmt, ap);
  va_end(ap);

  PyErr_SetString(PyExc_TypeError, str);
  return 0;
}

static int is_cvmat(PyObject *o)
{
  return 1;
}

static int convert_to_CvMat(PyObject *o, CvMat **dst, const char *name)
{
  cvmat_t *m = (cvmat_t*)o;
  void *buffer;
  Py_ssize_t buffer_len;

  if (!is_cvmat(o)) {
#if !PYTHON_USE_NUMPY
    return failmsg("Argument '%s' must be CvMat", name);
#else
    PyObject *asmat = fromarray(o, 0);
    if (asmat == NULL)
      return failmsg("Argument '%s' must be CvMat", name);
    // now have the array obect as a cvmat, can use regular conversion
    return convert_to_CvMat(asmat, dst, name);
#endif
  } else {
    m->a->refcount = NULL;
    if (m->data && PyString_Check(m->data)) {
      char *ptr = PyString_AsString(m->data) + m->offset;
      cvSetData(m->a, ptr, m->a->step);
      *dst = m->a;
      return 1;
    } else if (m->data && PyObject_AsWriteBuffer(m->data, &buffer, &buffer_len) == 0) {
      cvSetData(m->a, (void*)((char*)buffer + m->offset), m->a->step);
      *dst = m->a;
      return 1;
    } else {
      return failmsg("CvMat argument '%s' has no data", name);
    }
  }
}

static PyObject *FROM_CvMat(CvMat *m)
{
  PyObject *creatematheader  = PyObject_GetAttrString(mod_opencv, "CreateMatHeader");
  PyObject *setdata  = PyObject_GetAttrString(mod_opencv, "SetData");
  PyObject *args;

  args = Py_BuildValue("iii", m->rows, m->cols, CV_MAT_TYPE(m->type));
  PyObject *pym = PyObject_CallObject(creatematheader, args);
  Py_DECREF(args);

  args = Py_BuildValue("Os#i", pym, m->data.ptr, m->rows * m->step, m->step);
  Py_DECREF(PyObject_CallObject(setdata, args));
  Py_DECREF(args);

  Py_DECREF(creatematheader);
  Py_DECREF(setdata);

  return pym;
}

bp::object
cvtColorWrap(bp::object obj_in, const std::string & encoding_in, const std::string & encoding_out) {
  // Convert the Python input to an image
  CvMat *cv_mat_in;
  convert_to_CvMat(obj_in.ptr(), &cv_mat_in, "image");

  cv::Mat mat_in(cv_mat_in);

  // Call cv_bridge for color conversion
  cv_bridge::CvImagePtr cv_image(new cv_bridge::CvImage(std_msgs::Header(), encoding_in, mat_in));

  cv_bridge::CvImagePtr res = cv_bridge::cvtColor(cv_image, encoding_out);

  CvMat m(res->image);

  return bp::object(boost::python::handle<>(FROM_CvMat(&m)));
}

BOOST_PYTHON_MODULE(cv_bridge_boost)
{
  mod_opencv = PyImport_ImportModule("cv");

  // Wrap the function to get encodings as OpenCV types
  boost::python::def("getCvType", cv_bridge::getCvType);
  boost::python::def("cvtColor", cvtColorWrap);
}
