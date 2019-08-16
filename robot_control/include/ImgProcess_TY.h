#pragma once
#include "common_TY/common.hpp"
#include "predict_post.h"

// 相机内参
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
	bool getPixelDepth(cv::Point pixel, std::string depth_path, double &depth);

	/*
	* @brief	像素点转到相机坐标系下的xy值
	* @param	position	像素点
	* @param	相机坐标系下的xy值
	*/
	void ImgPixel2CameraPosition(const cv::Point& pixels, cv::Point2d& camera_xy);

private:
	// 相机内参和畸变
	Camera_Intrinsics cam_in;

};