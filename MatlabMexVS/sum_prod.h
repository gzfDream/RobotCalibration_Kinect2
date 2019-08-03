//
// MATLAB Compiler: 6.5 (R2017b)
// Date: Fri Aug  2 16:30:47 2019
// Arguments: "-B""macro_default""-W""cpplib:sum_prod""-T""link:lib""sum_prod"
//

#ifndef __sum_prod_h
#define __sum_prod_h 1

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
#ifndef LIB_sum_prod_C_API 
#define LIB_sum_prod_C_API /* No special import/export declaration */
#endif

/* GENERAL LIBRARY FUNCTIONS -- START */

extern LIB_sum_prod_C_API 
bool MW_CALL_CONV sum_prodInitializeWithHandlers(
       mclOutputHandlerFcn error_handler, 
       mclOutputHandlerFcn print_handler);

extern LIB_sum_prod_C_API 
bool MW_CALL_CONV sum_prodInitialize(void);

extern LIB_sum_prod_C_API 
void MW_CALL_CONV sum_prodTerminate(void);

extern LIB_sum_prod_C_API 
void MW_CALL_CONV sum_prodPrintStackTrace(void);

/* GENERAL LIBRARY FUNCTIONS -- END */

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

extern LIB_sum_prod_C_API 
bool MW_CALL_CONV mlxSum_prod(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[]);

/* C INTERFACE -- MLX WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */

#ifdef __cplusplus
}
#endif


/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- START */

#ifdef __cplusplus

/* On Windows, use __declspec to control the exported API */
#if defined(_MSC_VER) || defined(__MINGW64__)

#ifdef EXPORTING_sum_prod
#define PUBLIC_sum_prod_CPP_API __declspec(dllexport)
#else
#define PUBLIC_sum_prod_CPP_API __declspec(dllimport)
#endif

#define LIB_sum_prod_CPP_API PUBLIC_sum_prod_CPP_API

#else

#if !defined(LIB_sum_prod_CPP_API)
#if defined(LIB_sum_prod_C_API)
#define LIB_sum_prod_CPP_API LIB_sum_prod_C_API
#else
#define LIB_sum_prod_CPP_API /* empty! */ 
#endif
#endif

#endif

extern LIB_sum_prod_CPP_API void MW_CALL_CONV sum_prod(int nargout, mwArray& res, const mwArray& p1, const mwArray& p2, const mwArray& str);

/* C++ INTERFACE -- WRAPPERS FOR USER-DEFINED MATLAB FUNCTIONS -- END */
#endif

#endif
