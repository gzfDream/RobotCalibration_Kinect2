//
// MATLAB Compiler: 6.5 (R2017b)
// Date: Sun Aug 18 23:35:56 2019
// Arguments:
// "-B""macro_default""-W""cpplib:cameraCalibrator_64""-T""link:lib""cameraCalib
// rator"
//

#ifndef __cameraCalibrator_64_h
#define __cameraCalibrator_64_h 1

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
#ifndef LIB_cameraCalibrator_64_C_API 
#define LIB_cameraCalibrator_64_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_cameraCalibrator_64_C_API 
bool MW_CALL_CONV cameraCalibrator_64InitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_cameraCalibrator_64_C_API 
bool MW_CALL_CONV cameraCalibrator_64Initialize(void);

extern LIB_cameraCalibrator_64_C_API 
void MW_CALL_CONV cameraCalibrator_64Terminate(void);

extern LIB_cameraCalibrator_64_C_API 
void MW_CALL_CONV cameraCalibrator_64PrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_cameraCalibrator_64_C_API 
bool MW_CALL_CONV mlxCameraCalibrator(int nlhs, mxArray *plhs[], int nrhs, mxArray 
                                      *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_cameraCalibrator_64
#define PUBLIC_cameraCalibrator_64_CPP_API __declspec(dllexport)
#else
#define PUBLIC_cameraCalibrator_64_CPP_API __declspec(dllimport)
#endif

#define LIB_cameraCalibrator_64_CPP_API PUBLIC_cameraCalibrator_64_CPP_API

#else

#if !defined(LIB_cameraCalibrator_64_CPP_API)
#if defined(LIB_cameraCalibrator_64_C_API)
#define LIB_cameraCalibrator_64_CPP_API LIB_cameraCalibrator_64_C_API
#else
#define LIB_cameraCalibrator_64_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_cameraCalibrator_64_CPP_API void MW_CALL_CONV cameraCalibrator(int nargout, mwArray& cameraParams, mwArray& IntrinsicMatrix, mwArray& Distortion, const mwArray& imageFolder, const mwArray& squareSize);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
