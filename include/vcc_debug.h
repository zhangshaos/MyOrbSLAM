#ifndef __VCC_DEBUG_H__

#include <easylogging++.h>

#include <boost/exception/all.hpp>
#include <boost/format.hpp>
#include <boost/stacktrace.hpp>

// ¸¨ÖúÒì³£º¯Êý
using traced =
    boost::error_info<struct tag_stacktrace, boost::stacktrace::stacktrace>;
template <typename E>
void throw_with_trace(const E& e) {
  throw boost::enable_error_info(e) << traced(boost::stacktrace::stacktrace());
}

#endif  // !__VCC_DEBUG_H__