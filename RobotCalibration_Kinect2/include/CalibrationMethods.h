#pragma once

/*
*	@brief	手眼标定的方法，包括六点法和标定板法（标定板固定在机械臂末端）
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
	*	@brief		计算相机到机械臂的变换矩阵(棋盘格固定在相机)
	*	@param		robot		机械臂末端不同位置对应变换矩阵 baseHend
	*   @param		cam_cal		相机到棋盘格变换矩阵 calHcam
	*   @param		result_file	结果保存文件
	*   @return		返回相机到机械臂坐标系变换矩阵 robHcam
	*/
	void Method_BoardOnRobot(std::string robot, std::string cam_cal, std::string result_file, cv::Mat& m_result);

	
	/*
	*	@brief	六点法工具位姿估计
	*
	*/


	/*
	*	@brief		三点法标定
	*	@param		pointO		标定板坐标系原点
	*	@param		pointX		标定板坐标系X轴上一点
	*	@param		pointXOY	标定板坐标系XOY平面上一点
	*   @param		camHcal		标定板到棋盘格变换矩阵
	*   @param		camera_ins_H 相机内参
	*	@return					返回相机到机械臂坐标系变换矩阵 baseHcam
	*/
	void Method_ThreePointsCalibration(cv::Point3d pointO, cv::Point3d pointX, cv::Point3d pointXOY, cv::Mat camHcal, cv::Mat& baseHcam);


private:

	/*
	*	@brief	求解方程组 Ax=xB
	*	@param	Hgij	相机到标定板的变换矩阵 calHcam
	*   @param	Hcij	机械臂末端到机械臂基座的变换矩阵 baseHend
	*   @return 返回机械臂到相机的变换矩阵 camHbase
	*/
	cv::Mat HandEyeMethod(const vector<Mat>Hgij, const vector<Mat>Hcij);


	/*
	*	@brief		三点法标定
	*   @brief		step1		输入三点，得到标定板到机械臂的变换矩阵  baseHcal
	*	@param		pointO		标定板坐标系原点
	*	@param		pointX		标定板坐标系X轴上一点
	*	@param		pointXOY	标定板坐标系XOY平面上一点
	*	@return					返回标定板到机械臂的变换矩阵  baseHcal
	*/
	cv::Mat ThreePointsCalibration(cv::Point3d pointO, cv::Point3d pointX, cv::Point3d pointXOY);
};

