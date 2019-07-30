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
class CalChessboard
{
public:
	
	/*
	*	@brief:	构造函数
	*	@param:	imgpath		图片路径
	*	@param:	calibFile	存储计算结果路径
	*/
	CalChessboard(double &markerRealSize);
	~CalChessboard();


	/*
	*	@brief:	计算相机内参
	*	@param  图片路径vector
	*   @param  保存路径
	*   @reference link:https://my.oschina.net/abcijkxyz/blog/787659
	*/
	void internal_reference_calibration(std::vector<std::string> img_files, std::string internal_file);


	/*
	*	@brief:	计算相机外参
	*	@param:	imgpath		图片路径
	*	@param:	calibFile	存储计算结果路径
	*/
	cv::Mat external_reference_calibration(double camD[9], double distCoeffD[5], std::string &imgpath, std::string &calibFile);

private:

	/*
	*	@brief:	检测角点
	*	@param:	imgpath		图片路径
	*/
	void corner_detection(std::string &imgpath);


	/*
	*	@brief:	计算相机外参
	*	@param:	camera_matrix	内参
	*   @param:	distortion_coefficients	相机畸变参数
	*   @param:	rvec	旋转向量
	*   @param: tvec	平移向量
	*/
	void calibration(cv::Matx33f camera_matrix, cv::Matx<float, 5, 1> distortion_coefficients, cv::Mat &rvec, cv::Mat &tvec);


	/*
	*	@brief: 将旋转向量和平移向量变为4x4齐次变换矩阵
	*/
	cv::Mat process_transMatrix(cv::Mat rvec, cv::Mat tvec);


private:

	std::vector<cv::Point3f> m_markerCorners3d;
	std::vector<cv::Point2f> m_markerCorners2d;

	cv::Mat image_color;
	
};

