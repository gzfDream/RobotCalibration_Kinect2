#pragma once

/*
* brief: 标定相机内参、外参
* author： gzf
* date： 2019/8
*/

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
class CameraCalibration
{
public:
	
	/*
	*	@brief:	构造函数
	*	@param:	markerRealSize		棋盘格真实尺寸（厘米）
	*/
	CameraCalibration(double &markerRealSize);
	~CameraCalibration();


	/*
	*	@brief:	计算相机内参
	*	@param  img_files	图片路径
	*   @param  internal_file	保存路径
	*   @reference link:https://my.oschina.net/abcijkxyz/blog/787659
	*/
	void internal_reference_calibration(std::string img_files, std::string internal_file, Camera_Intrinsics& cam_in, double distCoeffD[5]);


	/*
	*	@brief:	计算相机外参(路径下所有图片对应的外参)
	*   @param: camera_ins_H	相机内参
	*	@param：	distCoeffD	畸变参数
	*	@param:	imgpath		图片路径
	*	@param:	calibFile	存储计算结果路径
	*   @param: vec_res		外参
	*/
	bool external_reference_calibration(Camera_Intrinsics camera_ins_H, double distCoeffD[5], std::string imgpath, std::string calibFile, std::vector<cv::Mat>& vec_res);


	/*
	*	@brief:	计算相机外参(单张图片)
	*   @param: camera_ins_H	相机内参
	*	@param：	distCoeffD	畸变参数
	*	@param:	imgpath		图片路径
	*	@param:	external_mat	外参		
	*/
	bool external_reference_calibration_singleImage(Camera_Intrinsics camera_ins_H, double distCoeffD[5], std::string imgpath, cv::Mat& external_mat);


private:

	/*
	*	@brief:	检测角点
	*	@param:	imgpath		图片路径
	*/
	bool corner_detection(std::string &imgpath);


	/*
	*	@brief:	计算相机外参
	*	@param:	camera_matrix	内参
	*   @param:	distortion_coefficients	相机畸变参数
	*   @param:	rvec	旋转向量
	*   @param: tvec	平移向量
	*/
	bool calibration(const cv::Matx33f& camera_matrix, const cv::Matx<float, 5, 1>& distortion_coefficients, cv::Mat &rvec, cv::Mat &tvec);


	/*
	*	@brief: 将旋转向量和平移向量变为4x4齐次变换矩阵
	*/
	cv::Mat process_transMatrix(const cv::Mat& rvec, const cv::Mat& tvec);


private:

	std::vector<cv::Point3f> m_markerCorners3d;
	std::vector<cv::Point2f> m_markerCorners2d;

	cv::Mat image_color;
	
};

