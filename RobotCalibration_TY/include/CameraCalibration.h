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
class CameraCalibration
{
public:
	
	/*
	*	@brief:	���캯��
	*	@param:	markerRealSize		���̸���ʵ�ߴ磨���ף�
	*/
	CameraCalibration(double &markerRealSize);
	~CameraCalibration();


	/*
	*	@brief:	��������ڲ�
	*	@param  img_files	ͼƬ·��
	*   @param  internal_file	����·��
	*   @reference link:https://my.oschina.net/abcijkxyz/blog/787659
	*/
	void internal_reference_calibration(std::string img_files, std::string internal_file, Camera_Intrinsics& cam_in, double distCoeffD[5]);


	/*
	*	@brief:	����������(·��������ͼƬ��Ӧ�����)
	*   @param: camera_ins_H	����ڲ�
	*	@param��	distCoeffD	�������
	*	@param:	imgpath		ͼƬ·��
	*	@param:	calibFile	�洢������·��
	*   @param: vec_res		���
	*/
	bool external_reference_calibration(Camera_Intrinsics camera_ins_H, double distCoeffD[5], std::string imgpath, std::string calibFile, std::vector<cv::Mat>& vec_res);


	/*
	*	@brief:	����������(����ͼƬ)
	*   @param: camera_ins_H	����ڲ�
	*	@param��	distCoeffD	�������
	*	@param:	imgpath		ͼƬ·��
	*	@param:	external_mat	���		
	*/
	bool external_reference_calibration_singleImage(Camera_Intrinsics camera_ins_H, double distCoeffD[5], std::string imgpath, cv::Mat& external_mat);


private:

	/*
	*	@brief:	���ǵ�
	*	@param:	imgpath		ͼƬ·��
	*/
	bool corner_detection(std::string &imgpath);


	/*
	*	@brief:	����������
	*	@param:	camera_matrix	�ڲ�
	*   @param:	distortion_coefficients	����������
	*   @param:	rvec	��ת����
	*   @param: tvec	ƽ������
	*/
	bool calibration(const cv::Matx33f& camera_matrix, const cv::Matx<float, 5, 1>& distortion_coefficients, cv::Mat &rvec, cv::Mat &tvec);


	/*
	*	@brief: ����ת������ƽ��������Ϊ4x4��α任����
	*/
	cv::Mat process_transMatrix(const cv::Mat& rvec, const cv::Mat& tvec);


private:

	std::vector<cv::Point3f> m_markerCorners3d;
	std::vector<cv::Point2f> m_markerCorners2d;

	cv::Mat image_color;
	
};

