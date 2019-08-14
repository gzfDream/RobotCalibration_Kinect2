#pragma once
#include "common_TY/common.hpp"
#include "predict_post.h"

// 相机内参
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
	* @brief	按键监控，采集图片，并预测抓取区域
	* @param	url			服务器地址
	* @param	img_path	存储路径
	*/
	void getImage(std::string url, std::string img_path);

	/*
	* @brief	得到深度图指定像素点的深度值
	* @param	pixel	像素点
	* @param	depth_path	深度图路径
	* @param	depth	得到的深度值
	* @return	若成功获得则返回 true
	*/
	static bool getPixelDepth(cv::Point pixel, std::string depth_path, double &depth);

	/*
	* @brief	像素点转到相机坐标系下的xy值
	* @param	position	像素点
	* @param	cam_in		内参
	* @param	distCoeffD	畸变
	* @param	相机坐标系下的xy值
	*/
	static void ImgPixel2CameraPosition(const std::vector<cv::Point>& pixels, const Camera_Intrinsics& cam_in,
		const double distCoeffD[5], std::vector<cv::Point2d>& camera_xy);

private:
	
};