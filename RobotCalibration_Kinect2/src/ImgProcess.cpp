#include "stdafx.h"

#include "ImgProcess.h"
#include "CalChessboard.h"
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

//计算图心
int aoiGravityCenter(IplImage *src, cv::Point &center)
{
	//if(!src)
	// return GRAVITYCENTER__SRC_IS_NULL;
	double m00, m10, m01;
	CvMoments moment;
	cvMoments(src, &moment, 1);
	m00 = cvGetSpatialMoment(&moment, 0, 0);
	if (m00 == 0)
		return 1;
	m10 = cvGetSpatialMoment(&moment, 1, 0);
	m01 = cvGetSpatialMoment(&moment, 0, 1);
	center.x = (int)(m10 / m00);
	center.y = (int)(m01 / m00);
	return 0;
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

bool ImgProcess::colorSubRT(int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV, vector<cv::Point>& m_center){
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

		//////////////////////////////////////////////////////////////////////////
		//颜色跟踪
		//水平翻转
		cv::flip(colorImg, colorImg, 1);

		cv::Mat img_HSV;
		cv::cvtColor(colorImg, img_HSV, cv::COLOR_BGR2HSV);

		cv::Mat imgThresholded;

		cv::inRange(img_HSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image  

		//开操作 (去除一些噪点)  如果二值化后图片干扰部分依然很多，增大下面的size
		cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
		morphologyEx(imgThresholded, imgThresholded, cv::MORPH_OPEN, element);

		//闭操作 (连接一些连通域)  
		morphologyEx(imgThresholded, imgThresholded, cv::MORPH_CLOSE, element);

		vector<cv::Point> vec_center;
		cv::Point2f center;

		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(imgThresholded, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
		for (int i = 0; i < contours.size(); i++){
			center = cv::minAreaRect(contours[i]).center;
			//cv::minAreaRect(contours[i]).center;
			drawContours(colorImg, contours, i, cv::Scalar(0, 0, 255), 2, 8, hierarchy, 0, cv::Point(0, 0));
			circle(colorImg, center, 5, cv::Scalar(0, 0, 255), 5, 8, 0);//绘制目标位置*/
			vec_center.push_back(center);
		}

		//这里是自定义的求取形心函数，当然用连通域计算更好
		//cv::Point center;
		//IplImage *src;
		//src = &IplImage(imgThresholded);
		//aoiGravityCenter(src, center);
		//circle(colorImg, center, 5, cv::Scalar(0, 0, 255), 5, 8, 0);//绘制目标位置
		//////////////////////////////////////////////////////////////////////////

		cv::namedWindow("display");
		cv::putText(colorImg, "press button '1' to get position ", cv::Point(50, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(23, 23, 200), 2, 8);//在图片上写文字
		cv::putText(colorImg, "press button 'Esc' to get out", cv::Point(50, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(23, 23, 200), 2, 8);//在图片上写文字
		imshow("display", colorImg);

		key = cv::waitKey(1);
		if (key == '1')
		{
			for (int i = 0; i < vec_center.size(); i++)
				cout << vec_center[i] << endl;
			m_center = vec_center;
		}
		else if (key == 27) {
			loop = false;
			cv::destroyAllWindows();
			return true;
		}
	}
	return false;
}

cv::Mat ImgProcess::calibration(Camera_Intrinsics camera_ins) {

	double camD1[9] = { camera_ins.FLX, 0, camera_ins.PPX,
		0, camera_ins.FLY, camera_ins.PPY,
		0, 0, 1 };
	double distCoeffD1[5] = { 0, 0, 0, 0, 0 };
	double markerRealSize = 3;

	CalChessboard cal = CalChessboard(markerRealSize);

	std::string imgpath, calibFile;

	imgpath = "..\\data\\img";
	calibFile = "..\\data\\cal.txt";

	cv::Mat mat = cal.external_reference_calibration(camD1, distCoeffD1, imgpath, calibFile);
	cv::destroyAllWindows();

	return mat;
}

vector<cv::Point> ImgProcess::colorSub(int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV) {
	//蓝色笔筒颜色的HSV范围
	/*int LowH = iLowH / 2;
	int HighH = iHighH / 2;

	int LowS = iLowS * 255 / 100;
	int HighS = iHighS * 255 / 100;

	int LowV = iLowV * 255 / 100;
	int HighV = iHighV * 255 / 100;*/

	string imgpath = "../data/sort/0.png";
	cv::Mat img_color = cv::imread(imgpath, cv::IMREAD_COLOR);
	cv::Mat img_HSV;
	cv::cvtColor(img_color, img_HSV, cv::COLOR_BGR2HSV);

	cv::imwrite("../data/sort/hsv.jpg", img_HSV);

	cv::Mat imgThresholded;

	cv::inRange(img_HSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image  
	cv::imwrite("../data/sort/imgThresholded.jpg", imgThresholded);

	//开操作 (去除一些噪点)  如果二值化后图片干扰部分依然很多，增大下面的size
	cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
	morphologyEx(imgThresholded, imgThresholded, cv::MORPH_OPEN, element);
	cv::imwrite("../data/sort/imgThresholded1.jpg", imgThresholded);

	//闭操作 (连接一些连通域)  
	morphologyEx(imgThresholded, imgThresholded, cv::MORPH_CLOSE, element);
	cv::imwrite("../data/sort/imgThresholded2.jpg", imgThresholded);

	vector<cv::Point> vec_center;
	cv::Point2f center;

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(imgThresholded, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
	for (int i = 0; i < contours.size(); i++){
		center = cv::minAreaRect(contours[i]).center;
		//cv::minAreaRect(contours[i]).center;
		drawContours(img_color, contours, i, cv::Scalar(0, 0, 255), 2, 8, hierarchy, 0, cv::Point(0, 0));
		circle(img_color, center, 5, cv::Scalar(0, 0, 255), 5, 8, 0);//绘制目标位置*/
		vec_center.push_back(center);
	}

	//这里是自定义的求取形心函数，当然用连通域计算更好
	/*vector<cv::Point> vec_center;
	cv::Point center;
	IplImage *src;
	src = &IplImage(imgThresholded);
	aoiGravityCenter(src, center);
	vec_center.push_back(center);
	circle(img_color, center, 5, cv::Scalar(0, 0, 255), 5, 8, 0);//绘制目标位置*/

	cv::namedWindow("color Image", CV_WINDOW_NORMAL);
	cv::moveWindow("color Image", 10, 10);
	cv::imshow("color Image", img_color);

	cv::namedWindow("HSV Image", CV_WINDOW_NORMAL);
	cv::imshow("HSV Image", img_HSV);

	cv::namedWindow("Thresholded Image", CV_WINDOW_NORMAL);
	cv::imshow("Thresholded Image", imgThresholded);

	/*cv::namedWindow("color", CV_WINDOW_NORMAL);
	cv::imshow("color", img_color);*/
	cv::waitKey(0);
	cv::destroyAllWindows();

	return vec_center;
}

void on_mouse(int EVENT, int x, int y, int flags, void* userdata)
{
	cv::Mat rgb, hsv;
	rgb = *(cv::Mat*)userdata;
	cv::Mat temp;
	cv::cvtColor(*(cv::Mat*)userdata, hsv, CV_RGB2HSV);
	cv::Point p(x, y);
	switch (EVENT)
	{
	case cv::EVENT_LBUTTONDOWN:
	{

		printf("b=%d\t", rgb.at<cv::Vec3b>(p)[0]);
		printf("g=%d\t", rgb.at<cv::Vec3b>(p)[1]);
		printf("r=%d\n", rgb.at<cv::Vec3b>(p)[2]);

		printf("H=%d\t", hsv.at<cv::Vec3b>(p)[0]);
		printf("S=%d\t", hsv.at<cv::Vec3b>(p)[1]);
		printf("V=%d\n", hsv.at<cv::Vec3b>(p)[2]);
		printf("-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*\n");
		circle(rgb, p, 2, cv::Scalar(255), 3);

	}
	break;

	}
}

void ImgProcess::HSVcheck(string str) {
	cv::Mat img_color = cv::imread(str, cv::IMREAD_COLOR);
	cv::Mat img_HSV;
	cv::cvtColor(img_color, img_HSV, cv::COLOR_BGR2HSV);

	cv::namedWindow("【display】");
	cv::setMouseCallback("【display】", on_mouse, &img_color);
	while (1)
	{
		cv::imshow("【display】", img_color);
		//cv::waitKey(20);
		int key = cv::waitKey(1);
		if (key == 27) {
			cv::destroyAllWindows();
			break;
		}
	}
}

void ImgProcess::FindContours(string str){

}