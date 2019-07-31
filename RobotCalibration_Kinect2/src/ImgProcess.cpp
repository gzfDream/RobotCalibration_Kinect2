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
	// ���ҵ�ǰĬ��Kinect
	HRESULT hr = ::GetDefaultKinectSensor(&pSensor);
	// ��ʿ�ش�Kinect
	if (SUCCEEDED(hr)) {
		hr = pSensor->Open();
	}
	// Retrieved Coordinate Mapper
	if (SUCCEEDED(hr)) {
		hr = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	}
	// ��ȡ���֡Դ(DepthFrameSource)
	if (SUCCEEDED(hr)) {
		hr = pSensor->get_DepthFrameSource(&pDepthFrameSource);
	}
	// ��ȡ��ɫ֡Դ(ColorFrameSource)
	if (SUCCEEDED(hr)) {
		hr = pSensor->get_ColorFrameSource(&pColorFrameSource);
	}
	// ��ȡ����֡Դ(ColorFrameSource)
	if (SUCCEEDED(hr)) {
		hr = pSensor->get_InfraredFrameSource(&pInfraredFrameSource);
	}
	// �ٻ�ȡ���֡��ȡ��
	if (SUCCEEDED(hr)) {
		hr = pDepthFrameSource->OpenReader(&pDepthReader);
	}
	// �ٻ�ȡ��ɫ֡��ȡ��
	if (SUCCEEDED(hr)) {
		hr = pColorFrameSource->OpenReader(&pColorReader);
	}
	// �ٻ�ȡ����֡��ȡ��
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

		//��ȡ����Ĳ�ɫ֡
		IColorFrame* pColorFrame = NULL;
		while ((hResult < 0) || (NULL == pColorFrame))//ѭ��ֱ����ȡ�������һ֡
		{
			hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
		}
		//��ȡ��ɫͼƬ��Ϣ�������ߣ���ʽ
		assert(hResult >= 0);
		pColorFrame->get_FrameDescription(&pColorDescription);//��ȡͼƬ������Ϣ
		pColorDescription->get_Width(&nWidth);
		pColorDescription->get_Height(&nHeight);
		pColorFrame->get_RawColorImageFormat(&imageFormat);//������Ϊ ColorImageFormat_Yuy2    = 5��ΪYuy2��ʽ
														   /*YUY2��ʽ����4:2:2��ʽ��� YUV 4:2:2
														   ÿ��ɫ���ŵ��ĳ������������ŵ���һ�룬����ˮƽ�����ɫ�ȳ�����ֻ��4:4 : 4��һ�롣�Է�ѹ����8����������ͼ����˵��
														   ÿ��������ˮƽ�������ڵ�������ɵĺ�������Ҫռ��4�ֽ��ڴ档
														   ������ĸ�����Ϊ��[Y0 U0 V0][Y1 U1 V1][Y2 U2 V2][Y3 U3 V3]
														   ��ŵ�����Ϊ��Y0 U0 Y1 V1 Y2 U2 Y3 V3
														   ӳ������ص�Ϊ��[Y0 U0 V1][Y1 U0 V1][Y2 U2 V3][Y3 U2 V3]*/
														   //cout << "imageformat is " << imageFormat << endl;

		cv::Mat colorImg(nHeight, nWidth, CV_8UC4);//�½�һ��mat�������ڱ�������ͼ��,ע������ĸ���ǰ�����ں�
		pBuffer = colorImg.data;
		nBufferSize = colorImg.rows*colorImg.step;

		/*����CopyConvertedFrameDataToArray���˺����������Ǵ�pColorFrame�����п���nBufferSize���ֽڵ�pBuffer��ָ��Mat�����У���
		ColorImageFormat_Bgra��ʽ����*/
		hResult = pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
		pColorFrame->Release();

		cv::namedWindow("display");
		cv::putText(colorImg, "press button '1' to get img ", cv::Point(50, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(23, 23, 200), 2, 8);//��ͼƬ��д����
		cv::putText(colorImg, "press button 'Esc' to get out", cv::Point(50, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(23, 23, 200), 2, 8);//��ͼƬ��д����
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



