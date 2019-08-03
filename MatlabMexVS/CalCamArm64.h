//
// MATLAB Compiler: 6.5 (R2017b)
// Date: Fri Aug  2 17:45:25 2019
// Arguments:
// "-B""macro_default""-W""cpplib:CalCamArm64""-T""link:lib""CalCamArm"
//

#ifndef __CalCamArm64_h
#define __CalCamArm64_h 1

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
#ifndef LIB_CalCamArm64_C_API 
#define LIB_CalCamArm64_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_CalCamArm64_C_API 
bool MW_CALL_CONV CalCamArm64InitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_CalCamArm64_C_API 
bool MW_CALL_CONV CalCamArm64Initialize(void);

extern LIB_CalCamArm64_C_API 
void MW_CALL_CONV CalCamArm64Terminate(void);

extern LIB_CalCamArm64_C_API 
void MW_CALL_CONV CalCamArm64PrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_CalCamArm64_C_API 
bool MW_CALL_CONV mlxCalCamArm(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_CalCamArm64
#define PUBLIC_CalCamArm64_CPP_API __declspec(dllexport)
#else
#define PUBLIC_CalCamArm64_CPP_API __declspec(dllimport)
#endif

#define LIB_CalCamArm64_CPP_API PUBLIC_CalCamArm64_CPP_API

#else

#if !defined(LIB_CalCamArm64_CPP_API)
#if defined(LIB_CalCamArm64_C_API)
#define LIB_CalCamArm64_CPP_API LIB_CalCamArm64_C_API
#else
#define LIB_CalCamArm64_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_CalCamArm64_CPP_API void MW_CALL_CONV CalCamArm(int nargout, mwArray& TBase, mwArray& TEnd, mwArray& cameraParams, mwArray& TBaseStd, mwArray& TEndStd, mwArray& pixelErr, const mwArray& imageFolder, const mwArray& armMat_path, const mwArray& squareSize, const mwArray& baseEst);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
