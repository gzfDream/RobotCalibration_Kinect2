#pragma once

#include <stdlib.h>
#include <stdio.h>
#include <tchar.h>

#include <windows.h>
#include <iostream>
#include <vector>
#include <fstream>  
#include <string>

#include <Eigen\Dense>
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>

typedef struct _Camera_Intrinsics
{
	double FLX;
	double FLY;
	double PPX;
	double PPY;
}Camera_Intrinsics;

