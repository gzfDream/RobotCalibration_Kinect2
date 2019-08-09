//
// MATLAB Compiler: 6.5 (R2017b)
// Date: Thu Aug  8 18:05:52 2019
// Arguments:
// "-B""macro_default""-W""cpplib:CalCamArm_64""-T""link:lib""CalCamArm_"
//

#include <stdio.h>
#define EXPORTING_CalCamArm_64 1
#include "CalCamArm_64.h"

static HMCRINSTANCE _mcr_inst = NULL;

#if defined( _MSC_VER) || defined(__LCC__) || defined(__MINGW64__)
#ifdef __LCC__
#undef EXTERN_C
#endif
#include <windows.h>

static char path_to_dll[_MAX_PATH];

BOOL WINAPI DllMain(HINSTANCE hInstance, DWORD dwReason, void *pv)
{
    if (dwReason == DLL_PROCESS_ATTACH)
    {
        if (GetModuleFileName(hInstance, path_to_dll, _MAX_PATH) == 0)
            return FALSE;
    }
    else if (dwReason == DLL_PROCESS_DETACH)
    {
    }
    return TRUE;
}
#endif
#ifdef __cplusplus
extern "C" {
#endif

static int mclDefaultPrintHandler(const char *s)
{
    return mclWrite(1 /* stdout */, s, sizeof(char)*strlen(s));
}

#ifdef __cplusplus
} /* End extern "C" block */
#endif

#ifdef __cplusplus
extern "C" {
#endif

static int mclDefaultErrorHandler(const char *s)
{
    int written = 0;
    size_t len = 0;
    len = strlen(s);
    written = mclWrite(2 /* stderr */, s, sizeof(char)*len);
    if (len > 0 && s[ len-1 ] != '\n')
        written += mclWrite(2 /* stderr */, "\n", sizeof(char));
    return written;
}

#ifdef __cplusplus
} /* End extern "C" block */
#endif

/* This symbol is defined in shared libraries. Define it here
 * (to nothing) in case this isn't a shared library. 
 */
#ifndef LIB_CalCamArm_64_C_API
#define LIB_CalCamArm_64_C_API /* No special import/export declaration */
#endif

LIB_CalCamArm_64_C_API 
bool MW_CALL_CONV CalCamArm_64InitializeWithHandlers(
    mclOutputHandlerFcn error_handler,
    mclOutputHandlerFcn print_handler)
{
    int bResult = 0;
    if (_mcr_inst != NULL)
        return true;
    if (!mclmcrInitialize())
        return false;
    if (!GetModuleFileName(GetModuleHandle("CalCamArm_64"), path_to_dll, _MAX_PATH))
        return false;
    {
        mclCtfStream ctfStream = 
            mclGetEmbeddedCtfStream(path_to_dll);
        if (ctfStream) {
            bResult = mclInitializeComponentInstanceEmbedded(&_mcr_inst,
                                                             error_handler, 
                                                             print_handler,
                                                             ctfStream);
            mclDestroyStream(ctfStream);
        } else {
            bResult = 0;
        }
    }  
    if (!bResult)
    return false;
    return true;
}

LIB_CalCamArm_64_C_API 
bool MW_CALL_CONV CalCamArm_64Initialize(void)
{
    return CalCamArm_64InitializeWithHandlers(mclDefaultErrorHandler, 
                                            mclDefaultPrintHandler);
}

LIB_CalCamArm_64_C_API 
void MW_CALL_CONV CalCamArm_64Terminate(void)
{
    if (_mcr_inst != NULL)
        mclTerminateInstance(&_mcr_inst);
}

LIB_CalCamArm_64_C_API 
void MW_CALL_CONV CalCamArm_64PrintStackTrace(void) 
{
    char** stackTrace;
    int stackDepth = mclGetStackTrace(&stackTrace);
    int i;
    for(i=0; i<stackDepth; i++)
    {
        mclWrite(2 /* stderr */, stackTrace[i], sizeof(char)*strlen(stackTrace[i]));
        mclWrite(2 /* stderr */, "\n", sizeof(char)*strlen("\n"));
    }
    mclFreeStackTrace(&stackTrace, stackDepth);
}


LIB_CalCamArm_64_C_API 
bool MW_CALL_CONV mlxCalCamArm_(int nlhs, mxArray *plhs[], int nrhs, mxArray *prhs[])
{
    return mclFeval(_mcr_inst, "CalCamArm_", nlhs, plhs, nrhs, prhs);
}

LIB_CalCamArm_64_CPP_API 
void MW_CALL_CONV CalCamArm_(int nargout, mwArray& TBase, mwArray& TEnd, mwArray& 
                             cameraParams, mwArray& pixelErr, const mwArray& imageFolder, 
                             const mwArray& armMat_path, const mwArray& squareSize, const 
                             mwArray& baseEst)
{
    mclcppMlfFeval(_mcr_inst, "CalCamArm_", nargout, 4, 4, &TBase, &TEnd, &cameraParams, &pixelErr, &imageFolder, &armMat_path, &squareSize, &baseEst);
}

