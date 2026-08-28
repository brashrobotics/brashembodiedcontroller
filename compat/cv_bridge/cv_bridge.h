#ifndef BRASH_CV_BRIDGE_COMPAT_H_
#define BRASH_CV_BRIDGE_COMPAT_H_

// ROS 2 Jazzy renamed the public C++ header from cv_bridge.h to
// cv_bridge.hpp. Keep the vendor B2 platform source unchanged while exposing
// the legacy include name it expects.
#include <cv_bridge/cv_bridge.hpp>

#endif  // BRASH_CV_BRIDGE_COMPAT_H_
