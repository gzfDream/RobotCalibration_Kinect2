#pragma once

#include "quaternion.h"

using namespace cv;
using namespace std;

class HandEyeMethods
{
public:
	HandEyeMethods();
	~HandEyeMethods();

public:
	/**
	* Hand/Eye calibration using Tsai' method.
	* Read the paper "A New Technique for Fully Autonomous and Efficient 3D Robotics
	* Hand-Eye Calibration, Tsai, 1989" for further details.
	*
	* Solving AX=XB
	* @Returns  void
	* @param Hcg [out] 4x4 matrix:The transformation from the camera to the marker or robot arm.
	* @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
	* @param Hcij [in] 4x4xN matrix:The transformation between the cameras.
	*/
	void Tsai_HandEye(Mat Hcg, vector<Mat> Hgij, vector<Mat> Hcij);

	/**
	* Hand/Eye calibration using Park' method(NAVY).
	* Read the paper "Robot Sensor Calibration: Solving AX = XB on the Euclidean Group, 1994" for further details.
	*
	* @Returns  void
	* @param Hcg [out] 4x4 matrix:The transformation from the camera to the marker or robot arm.
	* @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
	* @param Hcij [in] 4x4xN matrix:The transformation between the cameras.
	*/
	void Navy_HandEye(Mat Hcg, vector<Mat> Hgij, vector<Mat> Hcij);

	/**
	* Hand/Eye calibration using Daniilidis' method.
	* Read the paper "Hand-Eye Calibration Using Dual Quaternions, Konstantinos Daniilidis, 1999" for further details.
	*
	* @Returns  void
	* @param Hcg [out] 4x4 matrix:The transformation from the camera to the marker or robot arm.
	* @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
	* @param Hcij [in] 4x4xN matrix:The transformation between the cameras.
	*/
	void DualQ_HandEye(Mat Hcg, vector<Mat> Hgij, vector<Mat> Hcij);

	/**
	* Hand/Eye calibration using Andreff' method.
	* Read the paper "Robot Hand-Eye Calibration Using Structure-from-Motion, Andreff N, Horaud R, 2001" for further details.
	*
	* @Returns  void
	* @param Hcg [out] 4x4 matrix:The transformation from the camera to the marker or robot arm.
	* @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
	* @param Hcij [in] 4x4xN matrix:The transformation between the cameras.
	*/
	void Kron_HandEye(Mat Hcg, vector<Mat> Hgij, vector<Mat> Hcij);

	/**
	* Hand/Eye calibration using Horaud' method(INRIA).
	* Read the paper "Hand-Eye Calibration, Radu Horaud and Fadi Dornaika, 1995" for further details.
	*
	* @Returns  void
	* @param Hcg [out] 4x4 matrix:The transformation from the camera to the marker or robot arm.
	* @param Hgij [in] 4x4xN matrix:The transformation between the the markers or robot arms.
	* @param Hcij [in] 4x4xN matrix:The transformation between the cameras.
	*/
	void Inria_HandEye(Mat Hcg, vector<Mat> Hgij, vector<Mat> Hcij);

	//优化的手眼标定1
	Mat developMethod1(const vector<Mat>Hgij, const vector<Mat>Hcij);

	//优化的手眼标定2
	Mat developMethod2(const vector<Mat>Hgij, const vector<Mat>Hcij);

	//优化的手眼标定3
	Mat developMethod3(const vector<Mat>Hgij, const vector<Mat>Hcij);


private:
	void matrixMulti(double a[][4], double b[][4], double c[][4]);

	Mat R_to_k(Mat &R);

	Mat zhiji_Multiply(Mat&A, Mat&B);

	Mat vector_To_Mat(Mat&A);

	//判断是否为单位正交矩阵
	void judgement(const Mat &A);

	void errorCal(const vector<Mat>&Hgij, const vector<Mat>&Hcij, const Mat& handEye);

	void errorWithHcg(const Mat HcgCalculate, const Mat Hcg);


	Mat skew(Mat A);

	Mat readfcv(string str);

	int getlineNum(string str);

	std::vector<Mat> readf_vec(string str);

	/**
	* Creates a dual quaternion from a rotation matrix and a translation vector.
	*
	* @Returns  void
	* @param q [out] q
	* @param qprime [out] q'
	* @param R [in] Rotation
	* @param t [in] Translation
	*/
	void getDualQ(Mat q, Mat qprime, Mat R, Mat t);


	/**
	* Compute the Kronecker tensor product of matrix A and B.
	*
	* @Returns  cv::Mat (MP)x(NQ) matrix
	* @param A [in] MxN matrix
	* @param B [in] PxQ matrix
	*/
	Mat kron(Mat A, Mat B);


	/**
	* Signum function.
	* For each element of X, SIGN(X) returns 1 if the element is greater than zero,
	* return 0 if it equals zero and -1 if it is less than zero.
	*
	* @Returns  double
	* @param a [in]
	*/
	double sign(double a);

	
};




