//
// MATLAB Compiler: 6.5 (R2017b)
// Date: Mon Aug 19 10:23:40 2019
// Arguments:
// "-B""macro_default""-W""cpplib:getCameraExtrinsics_64""-T""link:lib""getCamer
// aExtrinsics"
//

#ifndef __getCameraExtrinsics_64_h
#define __getCameraExtrinsics_64_h 1

#if defined(__cplusplus) && !defined(mclmcrrt_h) && defined(__linux__)
#  pragma implementation "mclmcrrt.h"
#endif
#include "mclmcrrt.h"
#include "mclcppclass.h"
#ifdef __cplusplus
extern "C" {
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_getCameraExtrinsics_64_C_API 
#define LIB_getCameraExtrinsics_64_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_getCameraExtrinsics_64_C_API 
bool MW_CALL_CONV getCameraExtrinsics_64InitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_getCameraExtrinsics_64_C_API 
bool MW_CALL_CONV getCameraExtrinsics_64Initialize(void);

extern LIB_getCameraExtrinsics_64_C_API 
void MW_CALL_CONV getCameraExtrinsics_64Terminate(void);

extern LIB_getCameraExtrinsics_64_C_API 
void MW_CALL_CONV getCameraExtrinsics_64PrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_getCameraExtrinsics_64_C_API 
bool MW_CALL_CONV mlxGetCameraExtrinsics(int nlhs, mxArray *plhs[], int nrhs, mxArray 
                                         *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_getCameraExtrinsics_64
#define PUBLIC_getCameraExtrinsics_64_CPP_API __declspec(dllexport)
#else
#define PUBLIC_getCameraExtrinsics_64_CPP_API __declspec(dllimport)
#endif

#define LIB_getCameraExtrinsics_64_CPP_API PUBLIC_getCameraExtrinsics_64_CPP_API

#else

#if !defined(LIB_getCameraExtrinsics_64_CPP_API)
#if defined(LIB_getCameraExtrinsics_64_C_API)
#define LIB_getCameraExtrinsics_64_CPP_API LIB_getCameraExtrinsics_64_C_API
#else
#define LIB_getCameraExtrinsics_64_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_getCameraExtrinsics_64_CPP_API void MW_CALL_CONV getCameraExtrinsics(int nargout, mwArray& calHcam, const mwArray& image_path, const mwArray& intrinsicMatrix, const mwArray& Distortion);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
