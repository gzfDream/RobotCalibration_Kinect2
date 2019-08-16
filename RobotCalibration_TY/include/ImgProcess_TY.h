#pragma once
#include "./common_TY/common.hpp"
#include "Eigen/Dense"


class ImgProcess_TY {
public:

	ImgProcess_TY();
	~ImgProcess_TY();

public:

	/*
	* @brief	ʹ��ͼ������õ�rgbͼƬ 
	* @param	str			�洢·��
	* @param	depth_or	�Ƿ�洢���ͼ��true��洢��
	*/
	static void getImage(std::string str, bool depth_or);
	
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