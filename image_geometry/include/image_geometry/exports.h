#ifndef IMAGE_GEOMETRY_EXPORTS_H
#define IMAGE_GEOMETRY_EXPORTS_H

#include <ros/macros.h>

// Import/export for windows dll's and visibility for gcc shared libraries.

#ifdef ROS_BUILD_SHARED_LIBS // ros is being built around shared libraries
  #ifdef image_geometry_EXPORTS // we are building a shared lib/dll
    #define IMAGE_GEOMETRY_DECL ROS_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define IMAGE_GEOMETRY_DECL ROS_HELPER_IMPORT
  #endif
#else // ros is being built around static libraries
  #define IMAGE_GEOMETRY_DECL
#endif

#endif // IMAGE_GEOMETRY_EXPORTS_H
