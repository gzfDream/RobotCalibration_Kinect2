#pragma once
#include "./common_TY/common.hpp"
#include "Eigen/Dense"


class ImgProcess_TY {
public:

	ImgProcess_TY();
	~ImgProcess_TY();

public:

	/*
	* @brief	使用图漾相机得到rgb图片 
	* @param	str			存储路径
	* @param	depth_or	是否存储深度图（true则存储）
	*/
	static void getImage(std::string str, bool depth_or);
	
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