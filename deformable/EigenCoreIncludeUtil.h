#ifndef EIGEN_CORE_INCLUDE_UTIL_H
#define EIGEN_CORE_INCLUDE_UTIL_H


#ifdef WIN32
    #if (defined _XOPEN_SOURCE) && (defined TVMET_H) && (defined TVMET_HAVE_IEEE_MATH)
        #pragma push_macro("_XOPEN_SOURCE")
        #undef _XOPEN_SOURCE
        #include <Eigen/Core>
        #pragma pop_macro("_XOPEN_SOURCE")
    #else
        #include <Eigen/Core>
    #endif
#else
#include <Eigen/Core>
#endif // #ifdef WIN32


#endif