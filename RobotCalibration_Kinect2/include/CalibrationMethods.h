#pragma once

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
	*	@brief	�����������е�۵ı任����(���̸�̶������)
	*	@param	robot	��е��ĩ�˲�ͬλ�ö�Ӧ�任����
	*   @param	cam_cal ������̸�任����
	*   @param	result_file	��������ļ�
	*/
	cv::Mat Method_BoardOnRobot(std::string robot, std::string cam_cal, std::string result_file);


private:
	cv::Mat HandEyeMethod(const vector<Mat>Hgij, const vector<Mat>Hcij);
};

