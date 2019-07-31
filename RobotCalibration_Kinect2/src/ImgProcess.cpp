#include "stdafx.h"

#include "ImgProcess.h"
#include "CameraCalibration.h"
ImgProcess::ImgProcess()
{
}


ImgProcess::~ImgProcess()
{
	SafeRelease(pColorFrameSource);
	SafeRelease(pDepthFrameSource);
	SafeRelease(pColorReader);
	SafeRelease(pDepthReader);
	SafeRelease(pColorDescription);
	SafeRelease(pDepthDescription);
	SafeRelease(pCoordinateMapper);
	if (pSensor) {
		pSensor->Close();
	}
	SafeRelease(pSensor);
}

HRESULT ImgProcess::init_kinect() {
	// 查找当前默认Kinect
	HRESULT hr = ::GetDefaultKinectSensor(&pSensor);
	// 绅士地打开Kinect
	if (SUCCEEDED(hr)) {
		hr = pSensor->Open();
	}
	// Retrieved Coordinate Mapper
	if (SUCCEEDED(hr)) {
		hr = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	}
	// 获取深度帧源(DepthFrameSource)
	if (SUCCEEDED(hr)) {
		hr = pSensor->get_DepthFrameSource(&pDepthFrameSource);
	}
	// 获取颜色帧源(ColorFrameSource)
	if (SUCCEEDED(hr)) {
		hr = pSensor->get_ColorFrameSource(&pColorFrameSource);
	}
	// 获取红外帧源(ColorFrameSource)
	if (SUCCEEDED(hr)) {
		hr = pSensor->get_InfraredFrameSource(&pInfraredFrameSource);
	}
	// 再获取深度帧读取器
	if (SUCCEEDED(hr)) {
		hr = pDepthFrameSource->OpenReader(&pDepthReader);
	}
	// 再获取颜色帧读取器
	if (SUCCEEDED(hr)) {
		hr = pColorFrameSource->OpenReader(&pColorReader);
	}
	// 再获取红外帧读取器
	if (SUCCEEDED(hr)) {
		hr = pInfraredFrameSource->OpenReader(&pInfraredReader);
	}
	// Retrieved Color Frame Size
	if (SUCCEEDED(hr)) {
		hr = pColorFrameSource->get_FrameDescription(&pColorDescription);
	}
	//pColorDescription->get_Width(&ColorWidth); // 1920
	//pColorDescription->get_Height(&ColorHeight); // 1080
	// Retrieved Depth Frame Size
	if (SUCCEEDED(hr)) {
		hr = pDepthFrameSource->get_FrameDescription(&pDepthDescription);
	}
	//pDepthDescription->get_Width(&DepthWidth); //512
	//pDepthDescription->get_Height(&DepthHeight); //424
	// Retrieved Infrared Frame Size
	if (SUCCEEDED(hr)) {
		hr = pInfraredFrameSource->get_FrameDescription(&pInfraredDescription);
	}
	//pInfraredDescription->get_Width(&InfraredhWidth); //512
	//pInfraredDescription->get_Height(&InfraredHeight); //424

	camera_ins = getCameIns(pCoordinateMapper);
	return hr;
}

Camera_Intrinsics ImgProcess::getCameIns(ICoordinateMapper* pCoordinateMapper) {
	CameraIntrinsics *cameraIntrinsics = new CameraIntrinsics;
	pCoordinateMapper->GetDepthCameraIntrinsics(cameraIntrinsics);
	Camera_Intrinsics camera_ins;

	camera_ins.FLX = cameraIntrinsics->FocalLengthX;
	camera_ins.FLY = cameraIntrinsics->FocalLengthY;
	camera_ins.PPX = cameraIntrinsics->PrincipalPointX;
	camera_ins.PPY = cameraIntrinsics->PrincipalPointY;
	return camera_ins;
}


bool ImgProcess::getHDImage(string str) {
	int t1 = 0;
	char key = 0;
	bool loop = true;

	int nWidth, nHeight;
	ColorImageFormat imageFormat = ColorImageFormat_None;
	uchar *pBuffer = NULL;
	UINT nBufferSize = 0;

	HRESULT hResult = S_OK;

	while (loop) {

		if (!pColorReader)
		{
			break;
		}

		//获取最近的彩色帧
		IColorFrame* pColorFrame = NULL;
		while ((hResult < 0) || (NULL == pColorFrame))//循环直到获取到最近的一帧
		{
			hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		}
		//获取彩色图片信息包括宽，高，格式
		assert(hResult >= 0);
		pColorFrame->get_FrameDescription(&pColorDescription);//获取图片描述信息
		pColorDescription->get_Width(&nWidth);
		pColorDescription->get_Height(&nHeight);
		pColorFrame->get_RawColorImageFormat(&imageFormat);//输出结果为 ColorImageFormat_Yuy2    = 5，为Yuy2格式
														   /*YUY2格式，以4:2:2方式打包 YUV 4:2:2
														   每个色差信道的抽样率是亮度信道的一半，所以水平方向的色度抽样率只是4:4 : 4的一半。对非压缩的8比特量化的图像来说，
														   每个由两个水平方向相邻的像素组成的宏像素需要占用4字节内存。
														   下面的四个像素为：[Y0 U0 V0][Y1 U1 V1][Y2 U2 V2][Y3 U3 V3]
														   存放的码流为：Y0 U0 Y1 V1 Y2 U2 Y3 V3
														   映射出像素点为：[Y0 U0 V1][Y1 U0 V1][Y2 U2 V3][Y3 U2 V3]*/
														   //cout << "imageformat is " << imageFormat << endl;

		cv::Mat colorImg(nHeight, nWidth, CV_8UC4);//新建一个mat对象，用于保存读入的图像,注意参数的高在前，宽在后
		pBuffer = colorImg.data;
		nBufferSize = colorImg.rows*colorImg.step;

		/*调用CopyConvertedFrameDataToArray，此函数的作用是从pColorFrame对象中拷贝nBufferSize个字节到pBuffer所指的Mat矩阵中，按
		ColorImageFormat_Bgra格式保存*/
		hResult = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
		pColorFrame->Release();

		cv::namedWindow("display");
		cv::putText(colorImg, "press button '1' to get img ", cv::Point(50, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(23, 23, 200), 2, 8);//在图片上写文字
		cv::putText(colorImg, "press button 'Esc' to get out", cv::Point(50, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(23, 23, 200), 2, 8);//在图片上写文字
		cv::moveWindow("display", 10, 10);
		imshow("display", colorImg);
		
		key = cv::waitKey(1);
		if (key == '1')
		{
			cv::flip(colorImg, colorImg, 1);
			cv::imwrite(str + std::to_string(t1) + ".png", colorImg);
			t1++;
		}
		else if (key == 27) {
			loop = false;
			cv::destroyAllWindows();
			return true;
		}
	}
	return false;
}



