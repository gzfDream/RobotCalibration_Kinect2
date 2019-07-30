#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
typedef struct _Camera_Intrinsics
{
	double FLX;
	double FLY;
	double PPX;
	double PPY;
}Camera_Intrinsics;



class ImgProcess
{
public:
	ImgProcess();
	~ImgProcess();

public:
	HRESULT init_kinect();
	Camera_Intrinsics getCameIns();
	bool getHDImage(string str);
	cv::Mat calibration(Camera_Intrinsics camera_ins);
	vector<cv::Point> colorSub(int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV);
	void HSVcheck(string str);
	bool colorSubRT(int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV, vector<cv::Point>& m_center);
	void FindContours(string str);

private:
	Camera_Intrinsics getCameIns(ICoordinateMapper* pCoordinateMapper);
	

private:
	// Create Sensor Instance
	IKinectSensor * pSensor;
	// Retrieved Coordinate Mapper 
	ICoordinateMapper* pCoordinateMapper;
	// Retrieved Color Frame Source
	IColorFrameSource* pColorFrameSource;
	// Retrieved Depth Frame Source
	IDepthFrameSource* pDepthFrameSource;
	// Retrieved Infrared Frame Source
	IInfraredFrameSource* pInfraredFrameSource;
	// Open Color Frame Reader
	IColorFrameReader* pColorReader;
	// Open Depth Frame Reader
	IDepthFrameReader* pDepthReader;
	// Open Infrared Frame Reader
	IInfraredFrameReader* pInfraredReader;
	// Retrieved Color Frame Size
	IFrameDescription* pColorDescription;
	// Retrieved Depth Frame Size
	IFrameDescription* pDepthDescription;
	// Retrieved Infrared Frame Size
	IFrameDescription* pInfraredDescription;
	//Camera Intrinsics
	Camera_Intrinsics camera_ins;

	//颜色帧和深度帧
	const int ColorWidth = 1920;
	const int ColorHeight = 1080;
	const int DepthWidth = 512;
	const int DepthHeight = 424;
};

