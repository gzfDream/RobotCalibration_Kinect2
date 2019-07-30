#pragma once
/*
* brief: �궨����ڲΡ����
* author�� gzf
* date�� 2019/8
*/

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
class CalChessboard
{
public:
	
	/*
	*	@brief:	���캯��
	*	@param:	imgpath		ͼƬ·��
	*	@param:	calibFile	�洢������·��
	*/
	CalChessboard(double &markerRealSize);
	~CalChessboard();


	/*
	*	@brief:	��������ڲ�
	*	@param  ͼƬ·��vector
	*   @param  ����·��
	*   @reference link:https://my.oschina.net/abcijkxyz/blog/787659
	*/
	void internal_reference_calibration(std::vector<std::string> img_files, std::string internal_file);


	/*
	*	@brief:	����������
	*	@param:	imgpath		ͼƬ·��
	*	@param:	calibFile	�洢������·��
	*/
	cv::Mat external_reference_calibration(double camD[9], double distCoeffD[5], std::string &imgpath, std::string &calibFile);

private:

	/*
	*	@brief:	���ǵ�
	*	@param:	imgpath		ͼƬ·��
	*/
	void corner_detection(std::string &imgpath);


	/*
	*	@brief:	����������
	*	@param:	camera_matrix	�ڲ�
	*   @param:	distortion_coefficients	����������
	*   @param:	rvec	��ת����
	*   @param: tvec	ƽ������
	*/
	void calibration(cv::Matx33f camera_matrix, cv::Matx<float, 5, 1> distortion_coefficients, cv::Mat &rvec, cv::Mat &tvec);


	/*
	*	@brief: ����ת������ƽ��������Ϊ4x4��α任����
	*/
	cv::Mat process_transMatrix(cv::Mat rvec, cv::Mat tvec);


private:

	std::vector<cv::Point3f> m_markerCorners3d;
	std::vector<cv::Point2f> m_markerCorners2d;

	cv::Mat image_color;
	
};

