#pragma once
#include "common_TY/common.hpp"
#include "predict_post.h"

// ����ڲ�
typedef struct Camera_Intrinsics
{
	double FLX;
	double FLY;
	double PPX;
	double PPY;
	std::vector<double> distCoeffD;
	
	Camera_Intrinsics(){}
	
	Camera_Intrinsics(Camera_Intrinsics& cam) {
		*this = cam;
	}

	Camera_Intrinsics(double fx, double fy, double px, double py, std::vector<double> dist) :
		FLX(fx), FLY(fy), PPX(px), PPY(py), distCoeffD(dist) {}

	Camera_Intrinsics& operator=(const Camera_Intrinsics& cam) {
		if (this == &cam)
			return *this;

		this->FLX = cam.FLX;
		this->FLY = cam.FLY;
		this->PPX = cam.PPX;
		this->PPY = cam.PPY;
		this->distCoeffD = cam.distCoeffD;

		return *this;
	}
}Camera_Intrinsics;

class ImgProcess_TY {
public:

	ImgProcess_TY(const Camera_Intrinsics& cam_in_);
	~ImgProcess_TY();

public:
	/*
	* @brief	������أ��ɼ�ͼƬ����Ԥ��ץȡ����
	* @param	url			��������ַ
	* @param	img_path	�洢·��
	*/
	void getImage(std::string url, std::string img_path);

	/*
	* @brief	�õ����ͼָ�����ص�����ֵ
	* @param	pixel	���ص�
	* @param	depth_path	���ͼ·��
	* @param	depth	�õ������ֵ
	* @return	���ɹ�����򷵻� true
	*/
	bool getPixelDepth(cv::Point pixel, std::string depth_path, double &depth);

	/*
	* @brief	���ص�ת���������ϵ�µ�xyֵ
	* @param	position	���ص�
	* @param	�������ϵ�µ�xyֵ
	*/
	void ImgPixel2CameraPosition(const cv::Point& pixels, cv::Point2d& camera_xy);

private:
	// ����ڲκͻ���
	Camera_Intrinsics cam_in;

};