#include "stdafx.h"
#include "CalibrationMethods.h"
#include "ImgProcess_TY.h"
#include "CameraCalibration.h"

CalibrationMethods::CalibrationMethods()
{
}


CalibrationMethods::~CalibrationMethods()
{
}

static int getlineNum(string str) {
	ifstream  file(str);
	int lineNum = 0;
	string temp;
	if (!file.fail())
		while (getline(file, temp))
			lineNum++;
	file.close();
	return lineNum;
}

static  std::vector<Mat> readf_vec(string str) {
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

static  cv::Mat zhiji_Multiply(Mat&A, Mat&B)
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

static  cv::Mat vector_To_Mat(Mat&A)
{
	Mat dst(3, 3, CV_64FC1);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			dst.at<double>(i, j) = A.at<double>(i * 3 + j, 0);
	return dst;
}

// 将cv::Point3d转成eigen::vector3d
static Eigen::Vector3d cvPoint3d_eigenVector3d(cv::Point3d p) {
	Eigen::Vector3d res(p.x, p.y, p.z);

	return res;
}

 
//优化的手眼标定Ax=xB
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


// 三点法
cv::Mat CalibrationMethods::ThreePointsCalibration(cv::Point3d pointO, cv::Point3d pointX, cv::Point3d pointXOY)
{
	Eigen::Vector3d O = cvPoint3d_eigenVector3d(pointO);
	Eigen::Vector3d X = cvPoint3d_eigenVector3d(pointX);
	Eigen::Vector3d Y = cvPoint3d_eigenVector3d(pointXOY);

	Eigen::Vector3d Xn = X - O;
	Eigen::Vector3d Yn = Y - O;
	Eigen::Vector3d Zn = Xn.cross(Yn);

	Xn.normalize();
	Zn.normalize();

	double d = sqrt(pow(Zn(1), 2) + pow(Zn(2), 2));

	Eigen::Matrix4d Rx(4, 4), Ry(4, 4), Rz(4, 4);

	if (d == 0)
	{
		Rx << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
	}
	else {
		Rx << 1, 0, 0, 0,
			0, Zn(2) / d, -Zn(1) / d, 0,
			0, Zn(1) / d, Zn(2) / d, 0,
			0, 0, 0, 1;
	}
	Ry << d, 0, -Zn(0), 0,
		0, 1, 0, 0,
		Zn(0), 0, d, 0,
		0, 0, 0, 1;
	Eigen::Vector4d X4;
	X4 << Xn(0), Xn(1), Xn(2), 0;

	Eigen::Vector4d Xt = Ry * Rx * X4;
	Xt.normalize();

	Rz << Xt(0), Xt(1), 0, 0,
		-Xt(1), Xt(0), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	Eigen::Matrix4d R;

	Rz << Xt(0), -Xt(1), 0, 0,
		Xt(1), Xt(0), 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;
	if (d == 0)
	{
		Rx << 1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
	}
	else
	{
		Rx << 1, 0, 0, 0,
			0, Zn(2) / d, Zn(1) / d, 0,
			0, -Zn(1) / d, Zn(2) / d, 0,
			0, 0, 0, 1;
	}
	Ry << d, 0, Zn(0), 0,
		0, 1, 0, 0,
		-Zn(0), 0, d, 0,
		0, 0, 0, 1;

	R = Rx * Ry*Rz;

	R(0, 3) = O(0);
	R(1, 3) = O(1);
	R(2, 3) = O(2);

	cout << R << endl;

	// 输出
	/*std::ofstream ofsCalib("../data/robot2cal.txt");
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++)
		{
			ofsCalib << R(i, j) << " ";
		}
		ofsCalib << std::endl;
	}
	
	ofsCalib.close();
	*/

	cv::Mat baseHcal;
	cv::eigen2cv(R, baseHcal);
	
	return baseHcal;
}


// 求解endHcal
// baseHend1*endHcal*calHcam1 = baseHend2*endHcal*calHcam2
// inv(baseHend2)*baseHend1*calHcam = calHcam*calHcam2*inv(calHcam1)

void CalibrationMethods::Method_get_endHcal(std::string robot, std::string cam_cal, cv::Mat& endHcal) {
	cv::Mat Hcg(4, 4, CV_64FC1);
	vector<cv::Mat> Hgij;
	vector<cv::Mat> Hcij;
	std::vector<cv::Mat> baseHend;
	std::vector<cv::Mat> calHcam;
	cv::Mat matA(4, 4, CV_64FC1), matB(4, 4, CV_64FC1);

	// 读取机械臂末端位姿
	baseHend = readf_vec(robot);
	// 读取相机和标定板的变换矩阵
	calHcam = readf_vec(cam_cal);

	if (baseHend.size() == calHcam.size())
		for (int i = 0; i < baseHend.size() - 1; i++)
		{
			matA = baseHend[i + 1].inv() * baseHend[i];
			matB = calHcam[i + 1] * calHcam[i].inv();

			Hgij.push_back(matA);
			Hcij.push_back(matB);

			//cout << "matA" << matA << endl;
			//cout << "matB" << matB << endl;

			matA.release();
			matB.release();
		}

	Hcg = HandEyeMethod(Hgij, Hcij);

	endHcal = Hcg;
	cout << "endHcal: \n" << endl << endHcal << endl;
	
	return;
}

/*
* [calHcam1] * camHrobot*[robotHend1] = [calHcam2] * camHrobot*[robotHend2]
* inv([calHcam2]) * [calHcam1] * camHrobot = camHrobot * [robotHend2] * inv([robotHend1])
* matA : inv([calHcam2]) * [calHcam1]
* matB : [robotHend2] * inv([robotHend1])
* X : camHrobot
*/
void CalibrationMethods::Method_BoardOnRobot(std::string robot, std::string cam_cal, std::string result_file, cv::Mat& m_result) {

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

			//cout << "matA" << matA << endl;
			//cout << "matB" << matB << endl;

			matA.release();
			matB.release();
		}
	/*
	*手眼标定
	*/
	Hcg = HandEyeMethod(Hgij, Hcij);

	// cout << "Hcg: \n" << Hcg << endl;

	// camHbase -> baseHcam
	m_result = Hcg.inv();
	cout << "baseHcam: \n" << m_result << endl;
	std::ofstream ofsCalib(result_file);
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++)
		{
			ofsCalib << m_result.at<double>(i, j) << " ";
		}
		ofsCalib << std::endl;
	}

	ofsCalib.close();
	return;
}


void CalibrationMethods::Method_ThreePointsCalibration(cv::Point3d pointO, cv::Point3d pointX, cv::Point3d pointXOY, cv::Mat calHcam, cv::Mat& baseHcam)
{
	cv::Mat baseHcal = ThreePointsCalibration(pointO, pointX, pointXOY);

	// cv::Mat baseHcam;
	baseHcam = baseHcal * calHcam;
	return;
}


void CalibrationMethods::eigenMat2mwArray(const Eigen::Matrix4d& m1, mwArray& arr) {
	Eigen::Matrix4d m = m1.transpose();

	double tmp[] = {
		m(0,0), m(0,1), m(0,2), m(0,3),
		m(1,0), m(1,1), m(1,2), m(1,3),
		m(2,0), m(2,1), m(2,2), m(2,3),
		m(3,0), m(3,1), m(3,2), m(3,3)
	};

	arr.SetData(tmp, 16);

}

void CalibrationMethods::mwArray2eigenMat(const mwArray& arr, Eigen::Matrix4d& mat) {
	mat << arr.Get(1, 1), arr.Get(1, 5), arr.Get(1, 9), arr.Get(1, 13),
		arr.Get(1, 2), arr.Get(1, 6), arr.Get(1, 10), arr.Get(1, 14),
		arr.Get(1, 3), arr.Get(1, 7), arr.Get(1, 11), arr.Get(1, 15),
		arr.Get(1, 4), arr.Get(1, 8), arr.Get(1, 12), arr.Get(1, 16);
}

void CalibrationMethods::Calibration_Optimization(string img_path, string armMat_path, const cv::Mat& baseHcam, cv::Mat& baseHcam_op, cv::Mat& endHcal_op) {
	if (!mclInitializeApplication(nullptr, 0)) {
		std::cerr << "Could not initialize the application properly" << std::endl;
		return;
	}

	if (!CalCamArm_64Initialize()) {
		std::cerr << "Could not initialize the library properly" << std::endl;
		return;
	}
	else {
		try {
			// 返回值
			mwArray TBase;
			mwArray TEnd;
			mwArray cameraParams;
			mwArray pixelErr;

			// 输入值
			// char img_path[] = "D:/OneDrive/code/RE3D_grasp_demo/Re3DX_Grasp_demo/EyeToHand/data/Images/";
			mwArray imageFolder(img_path.c_str());
			// char armMat_path[] = "D:/OneDrive/code/RE3D_grasp_demo/Re3DX_Grasp_demo/EyeToHand/data/robot.txt";
			mwArray armMat(armMat_path.c_str());

			int realSize[] = { 20 };
			mwArray squareSize(1, 1, mxINT32_CLASS, mxREAL);
			squareSize.SetData(realSize, 1);

			mwArray baseEst(4, 4, mxDOUBLE_CLASS, mxREAL);
			Eigen::Matrix4d base_mat;
			cv::cv2eigen(baseHcam, base_mat);
			// cout << "base_mat" << base_mat << endl;
			Eigen::Matrix4d base_mat_inv = base_mat.inverse();
			// std::cout << "base_mat_inv: " << base_mat_inv << endl;
			eigenMat2mwArray(base_mat_inv, baseEst);

			std::cout << "imageFolder: " << imageFolder << endl;
			std::cout << "armMat: " << armMat << endl;
			std::cout << "squareSize: " << squareSize << endl;
			std::cout << "baseEst: " << baseEst << endl;

			CalCamArm_(4, TBase, TEnd, cameraParams, pixelErr, imageFolder, armMat, squareSize, baseEst);
			std::cout << "TBase: " << TBase << endl;

			Eigen::Matrix4d camHbase, endHcal;
			mwArray2eigenMat(TBase, camHbase);
			mwArray2eigenMat(TEnd, endHcal);

			Eigen::Matrix4d baseHcam = camHbase.inverse();

			cv::eigen2cv(baseHcam, baseHcam_op);
			cv::eigen2cv(endHcal, endHcal_op);
		}
		catch (const mwException& e) {
			std::cerr << e.what() << std::endl;
			return;
		}
		catch (...) {
			std::cerr << "Unexpected error thrown" << std::endl;
			return;
		}

		CalCamArm_64Terminate();
	}

	mclTerminateApplication();
}

void CalibrationMethods::Calibration_PrecisionTest(const cv::Mat& baseHcam_op, const cv::Mat& endHcal_op, const Camera_Intrinsics& camera_ins_H, double distCoeffD[5])
{
	std::string str = ".\\data\\ImagesForTest\\00.png";
	// ImgProcess_TY::getImage(str, false);
	
	double markerRealSize = 2.;//cm
	CameraCalibration cam = CameraCalibration(markerRealSize, cv::Size(6,9));
	
	// 外参矩阵
	cv::Mat calHcam;
	cam.external_reference_calibration_singleImage(camera_ins_H, distCoeffD, str, calHcam);
	// cam.external_reference_calibration_singleImage_MATLAB(camera_ins_H, distCoeffD, str, calHcam);

	std::cout << "baseHcam_op: " << std::endl << baseHcam_op << std::endl;
	std::cout << "endHcal_op: " << std::endl << endHcal_op << std::endl;
	std::cout << "calHcam: " << std::endl << calHcam << std::endl;

	cv::Mat baseHend = baseHcam_op* calHcam.inv() * endHcal_op.inv();
	std::cout << baseHend << std::endl;

}