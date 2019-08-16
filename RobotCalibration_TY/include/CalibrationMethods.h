#pragma once
#include <opencv2/core/eigen.hpp>
#include "CalCamArm_64.h"

#pragma comment(lib,"CalCamArm_64.lib")

/*
*	@brief	���۱궨�ķ������������㷨�ͱ궨�巨���궨��̶��ڻ�е��ĩ�ˣ�
*/

using namespace cv;
using namespace std;

class CalibrationMethods
{
public:
	CalibrationMethods();
	~CalibrationMethods();

public:

	/*
	*	@brief		�����������е�۵ı任����(���̸�̶������)
	*	@param		robot		��е��ĩ�˲�ͬλ�ö�Ӧ�任���� baseHend
	*   @param		cam_cal		��������̸�任���� calHcam
	*   @param		result_file	��������ļ�
	*   @return		�����������е������ϵ�任���� robHcam
	*/
	void Method_BoardOnRobot(std::string robot, std::string cam_cal, std::string result_file, cv::Mat& m_result);

	
	/*
	*	@brief	���㷨����λ�˹���
	*
	*/


	/*
	*	@brief		���㷨�궨
	*	@param		pointO		�궨������ϵԭ��
	*	@param		pointX		�궨������ϵX����һ��
	*	@param		pointXOY	�궨������ϵXOYƽ����һ��
	*   @param		camHcal		�궨�嵽���̸�任����
	*   @param		camera_ins_H ����ڲ�
	*	@return					�����������е������ϵ�任���� baseHcam
	*/
	void Method_ThreePointsCalibration(cv::Point3d pointO, cv::Point3d pointX, cv::Point3d pointXOY, cv::Mat camHcal, cv::Mat& baseHcam);


	/*
	*	@brief		�궨����Ż�
	*	@param		img_path		ͼƬ·��
	*	@param		armMat_path		�첲ĩ�˱任����·��
	*	@param		baseHcam		���۱任�����ʼ
	*	@param		baseHcam_op		�Ż����
	*/
	void Calibration_Optimization(string img_path, string armMat_path, const cv::Mat& baseHcam, cv::Mat& baseHcam_op, cv::Mat& endHcal_op);


	/*
	*	@brief	�궨���Ȳ���
	*
	*/
	void Calibration_PrecisionTest(const cv::Mat& baseHcam_op, const cv::Mat& endHcal_op);


private:

	/*
	*	@brief	��ⷽ���� Ax=xB
	*	@param	Hgij	������궨��ı任���� calHcam
	*   @param	Hcij	��е��ĩ�˵���е�ۻ����ı任���� baseHend
	*   @return ���ػ�е�۵�����ı任���� camHbase
	*/
	cv::Mat HandEyeMethod(const vector<Mat>Hgij, const vector<Mat>Hcij);


	/*
	*	@brief		���㷨�궨
	*   @brief		step1		�������㣬�õ��궨�嵽��е�۵ı任����  baseHcal
	*	@param		pointO		�궨������ϵԭ��
	*	@param		pointX		�궨������ϵX����һ��
	*	@param		pointXOY	�궨������ϵXOYƽ����һ��
	*	@return					���ر궨�嵽��е�۵ı任����  baseHcal
	*/
	cv::Mat ThreePointsCalibration(cv::Point3d pointO, cv::Point3d pointX, cv::Point3d pointXOY);


	//cv::Mat ת mwArray
	void eigenMat2mwArray(const Eigen::Matrix4d& mat, mwArray& arr);

	//cv::Mat ת mwArray
	void mwArray2eigenMat(const mwArray& arr, Eigen::Matrix4d& mat);
};

