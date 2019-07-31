#include "stdafx.h"
#include "CalibrationMethods.h"


CalibrationMethods::CalibrationMethods()
{
}


CalibrationMethods::~CalibrationMethods()
{
}

int getlineNum(string str) {
	ifstream  file(str);
	int lineNum = 0;
	string temp;
	if (!file.fail())
		while (getline(file, temp))
			lineNum++;
	file.close();
	return lineNum;
}

std::vector<Mat> readf_vec(string str) {
	std::vector<Mat> vec_mat;
	Mat mat(4, 4, CV_64FC1);
	float x;
	int lineNum = getlineNum(str);
	ifstream  file(str);
	for (int i = 0; i < lineNum; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			file >> x;
			mat.at<double>((i % 4), j) = x;
		}

		if (i % 4 == 3) {
			vec_mat.push_back(mat);
			mat.release();
			mat = Mat::ones(4, 4, CV_64FC1);
		}

	}
	file.close();
	//cout << mat << endl;
	return vec_mat;
}

cv::Mat zhiji_Multiply(Mat&A, Mat&B)
{
	//矩阵直积
	int height = A.rows*B.rows;
	int width = A.cols*B.cols;
	Mat dst(height, width, CV_64FC1);
	for (int i = 0; i < A.rows; i++)
		for (int j = 0; j < A.cols; j++)
			for (int k = 0; k < B.rows; k++)
				for (int m = 0; m < B.cols; m++)
					dst.at<double>(i*B.rows + k, j*B.cols + m)
					= A.at<double>(i, j)*B.at<double>(k, m);
	return dst;
}

cv::Mat vector_To_Mat(Mat&A)
{
	Mat dst(3, 3, CV_64FC1);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			dst.at<double>(i, j) = A.at<double>(i * 3 + j, 0);
	return dst;
}

//优化的手眼标定
Mat CalibrationMethods::HandEyeMethod(const vector<Mat>Hgij, const vector<Mat>Hcij)
{
	CV_Assert(Hgij.size() == Hcij.size());
	int nRobotState = Hgij.size();

	Mat I = Mat::eye(3, 3, CV_64FC1);
	Mat leftMat, rigthMat;
	for (int i = 0; i < nRobotState; i++)
	{
		Mat Rgij(3, 3, CV_64FC1);
		Mat Tgij(3, 1, CV_64FC1);
		Mat Rcij(3, 3, CV_64FC1);
		Mat Tcij(3, 1, CV_64FC1);
		Hgij[i](Rect(0, 0, 3, 3)).copyTo(Rgij);
		Hgij[i](Rect(3, 0, 1, 3)).copyTo(Tgij);
		Hcij[i](Rect(0, 0, 3, 3)).copyTo(Rcij);
		Hcij[i](Rect(3, 0, 1, 3)).copyTo(Tcij);
		Mat RcijT = Rcij.t();
		Mat tempMatLU = zhiji_Multiply(Rgij, I) - zhiji_Multiply(I, RcijT);
		Mat TbT = Tcij.t();
		Mat tempMatLD = zhiji_Multiply(I, TbT);
		Mat tempMatRD = I - Rgij;
		Mat tempLeftMat(12, 12, CV_64FC1, Scalar(0));
		tempMatLU.copyTo(tempLeftMat(Rect(0, 0, 9, 9)));
		tempMatLD.copyTo(tempLeftMat(Rect(0, 9, 9, 3)));
		tempMatRD.copyTo(tempLeftMat(Rect(9, 9, 3, 3)));
		leftMat.push_back(tempLeftMat);
		Mat tempRightMat(12, 1, CV_64FC1, Scalar(0));
		Tgij.copyTo(tempRightMat(Rect(0, 9, 1, 3)));
		rigthMat.push_back(tempRightMat);
	}
	Mat mat;
	hconcat(leftMat, rigthMat, mat);
	int rows = nRobotState * 12;
	Mat lamda(12, 1, CV_64FC1);         //奇异值
	Mat U(rows, 12, CV_64FC1);          //左矩阵  (不是方阵！！！)
	Mat VT(12, 12, CV_64FC1);         //右矩阵的转置
	SVD::compute(mat, lamda, U, VT);
	//cout << "左矩阵：\n" << U << endl << endl;       //Ut*U=I
	//cout << "特征值：\n" << lamda << endl;           //Vt*V=I
	//cout << "右矩阵：\n" << VT << endl << endl;
	Mat V = VT.t();
	Mat mm;
	V(Rect(12, 12, 1, 1)).copyTo(mm);
	double mmmm = mm.at<double>(0, 0);
	Mat vector_x(9, 1, CV_64FC1);
	V(Rect(12, 0, 1, 9)).copyTo(vector_x);
	Mat Rx = vector_To_Mat(vector_x) / mmmm * (-1);
	Mat Tx(1, 3, CV_64FC1);
	V(Rect(12, 9, 1, 3)).copyTo(Tx);
	Tx /= (-mmmm);

	Mat dst;
	hconcat(Rx, Tx, dst);
	//cout << Temp << endl;
	double homogeneous[4] = { 0, 0, 0, 1 };
	Mat h(1, 4, CV_64FC1, homogeneous);
	dst.push_back(h);
	//vconcat(dst, h, dst);
	return dst;
}



cv::Mat CalibrationMethods::Method_BoardOnRobot(std::string robot, std::string cam_cal, std::string result_file) {

	cv::Mat Hcg(4, 4, CV_64FC1);
	vector<cv::Mat> Hgij;
	vector<cv::Mat> Hcij;
	std::vector<cv::Mat> basHtool;
	std::vector<cv::Mat> calHcam;
	cv::Mat matA(4, 4, CV_64FC1), matB(4, 4, CV_64FC1);

	// 读取机械臂末端位姿
	basHtool = readf_vec(robot);
	// 读取相机和标定板的变换矩阵
	calHcam = readf_vec(cam_cal);

	if (basHtool.size() == calHcam.size())
		for (int i = 0; i < basHtool.size() - 1; i++)
		{
			//calHcam*camHbase*baseHtool
			matB = basHtool[i + 1] * basHtool[i].inv();
			matA = calHcam[i + 1].inv() * calHcam[i];

			Hgij.push_back(matA);
			Hcij.push_back(matB);

			cout << "matA" << matA << endl;
			cout << "matB" << matB << endl;

			matA.release();
			matB.release();
		}
	/*
	*手眼标定
	*/
	Hcg = HandEyeMethod(Hgij, Hcij);

	cout << "Hcg: \n" << Hcg << endl;

	cv::Mat m_result = Hcg.inv();
	cout << "result: \n" << m_result << endl;
	std::ofstream ofsCalib(result_file);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++)
		{
			ofsCalib << m_result.at<double>(i, j) << " ";
		}
		ofsCalib << std::endl;
	}

	ofsCalib.close();
	return m_result;
}
