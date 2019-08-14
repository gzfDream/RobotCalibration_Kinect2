#pragma once
#include "common_TY/common.hpp"
#include "predict_post.h"

// ����ڲ�
typedef struct _Camera_Intrinsics
{
	double FLX;
	double FLY;
	double PPX;
	double PPY;
}Camera_Intrinsics;

class ImgProcess_TY {
public:

	ImgProcess_TY();
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
	static bool getPixelDepth(cv::Point pixel, std::string depth_path, double &depth);

	/*
	* @brief	���ص�ת���������ϵ�µ�xyֵ
	* @param	position	���ص�
	* @param	cam_in		�ڲ�
	* @param	distCoeffD	����
	* @param	�������ϵ�µ�xyֵ
	*/
	static void ImgPixel2CameraPosition(const std::vector<cv::Point>& pixels, const Camera_Intrinsics& cam_in,
		const double distCoeffD[5], std::vector<cv::Point2d>& camera_xy);

private:
	
};