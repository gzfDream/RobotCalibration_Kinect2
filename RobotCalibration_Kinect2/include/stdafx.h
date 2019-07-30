#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <tchar.h>

#include <windows.h>
#include <kinect.h>
#include <iostream>
#include <vector>
#include <fstream>  
#include <string>

// TODO: reference additional headers your program requires here
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL) {
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

