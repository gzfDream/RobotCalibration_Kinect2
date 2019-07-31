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
	*	@brief	计算相机到机械臂的变换矩阵(棋盘格固定在相机)
	*	@param	robot	机械臂末端不同位置对应变换矩阵
	*   @param	cam_cal 相机棋盘格变换矩阵
	*   @param	result_file	结果保存文件
	*/
	cv::Mat Method_BoardOnRobot(std::string robot, std::string cam_cal, std::string result_file);


private:
	cv::Mat HandEyeMethod(const vector<Mat>Hgij, const vector<Mat>Hcij);
};

