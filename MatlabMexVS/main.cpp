#include <iostream>
#include "CalCamArm64.h"
#include "sum_prod.h"

#include "mclmcr.h"
#include "matrix.h"
#include "mclcppclass.h"
#include <Eigen/Dense>

#pragma comment(lib,"CalCamArm64.lib")
#pragma comment(lib,"sum_prod.lib")
using namespace std;


//
//int main() {
//	cout << "test" << endl;
//	mclmcrInitialize();
//	if(!mclInitializeApplication(NULL, 0))
//		return -1;
//	
//	if (!CalCamArm64Initialize()) {
//		
//		system("pause");
//		exit(EXIT_FAILURE);
//	}
//
//	// 返回值
//	mwArray TBase; 
//	mwArray TEnd;
//	mwArray cameraParams;
//	mwArray TBaseStd;
//	mwArray TEndStd;
//	mwArray pixelErr;
//
//	// 输入值
//	char img_path[] =  "D:/OneDrive/code/RE3D_grasp_demo/Re3DX_Grasp_demo/EyeToHand/data/Images/";
//	mwArray imageFolder(img_path);
//	char armMat_path[] = "D:/OneDrive/code/RE3D_grasp_demo/Re3DX_Grasp_demo/EyeToHand/data/robot.txt";
//	mwArray armMat(armMat_path);
//
//	int realSize[] = { 20 };
//	mwArray squareSize(1, 1, mxINT32_CLASS, mxREAL);
//	squareSize.SetData(realSize, 1);
//
//	mwArray baseEst(4, 4, mxDOUBLE_CLASS, mxREAL);
//	double base[] = {
//	0.9878,    0.0117, - 0.1553,         0,
//	0.1557, - 0.0971,    0.9830,         0,
//   - 0.0036, - 0.9952, - 0.0977,         0,
//   - 0.6330,    0.2241,    0.6082 ,   1.0000 };
//	baseEst.SetData(base, 16);
//	mwArray varargin;
//
//	cout << imageFolder << endl;
//	cout << armMat << endl;
//	cout << squareSize << endl;
//	cout << baseEst << endl;
//	CalCamArm(1, TBase, TEnd, cameraParams, TBaseStd, TEndStd, pixelErr, imageFolder, armMat, squareSize, baseEst, varargin);
//
//	CalCamArm64Terminate();
//	mclTerminateApplication();
//
//	cout << TBase << endl;
//
//	system("pause");
//	return 0;
//}


void eigenMat2mwArray(const Eigen::Matrix4d& mat, mwArray& arr) {
	Eigen::Matrix4d m = mat.transpose();

	double tmp[] = {
		m(0,0), m(0,1), m(0,2), m(0,3),
		m(1,0), m(1,1), m(1,2), m(1,3), 
		m(2,0), m(2,1), m(2,2), m(2,3), 
		m(3,0), m(3,1), m(3,2), m(3,3)
	};

	arr.SetData(tmp, 16);
}


void mwArray2eigenMat(const mwArray& arr, Eigen::Matrix4d& mat) {
	mat << arr.Get(1, 1), arr.Get(1, 5), arr.Get(1, 9), arr.Get(1, 13),
		arr.Get(1, 2), arr.Get(1, 6), arr.Get(1, 10), arr.Get(1, 14),
		arr.Get(1, 3), arr.Get(1, 7), arr.Get(1, 11), arr.Get(1, 15),
		arr.Get(1, 4), arr.Get(1, 8), arr.Get(1, 12), arr.Get(1, 16);
}

int run_main(int argc, const char **argv) {
	if (!CalCamArm64Initialize() || !sum_prodInitialize()) {

		std::cerr << "Could not initialize the library properly" << std::endl;
		return -1;

	}
	else {
		try {
			// 返回值
			mwArray TBase;
			mwArray TEnd;
			mwArray cameraParams;
			mwArray TBaseStd;
			mwArray TEndStd;
			mwArray pixelErr;

			// 输入值
			char img_path[] =  "D:/OneDrive/code/RE3D_grasp_demo/Re3DX_Grasp_demo/EyeToHand/data/Images/";
			mwArray imageFolder(img_path);
			char armMat_path[] = "D:/OneDrive/code/RE3D_grasp_demo/Re3DX_Grasp_demo/EyeToHand/data/robot.txt";
			mwArray armMat(armMat_path);

			int realSize[] = { 20 };
			mwArray squareSize(1, 1, mxINT32_CLASS, mxREAL);
			squareSize.SetData(realSize, 1);

			mwArray baseEst(4, 4, mxDOUBLE_CLASS, mxREAL);
			Eigen::Matrix4d base_mat;
			base_mat << 0.9878,     0.011699, - 0.15533,      0.71708,
				0.15573, - 0.097098,      0.98301, - 0.47752,
				- 0.0035819, - 0.9952, - 0.097734,      0.28021,
				0 ,           0,            0,            1;
			
			cout << base_mat << endl;
			Eigen::Matrix4d base_mat_inv = base_mat.inverse();
			cout << base_mat_inv << endl;
			eigenMat2mwArray(base_mat_inv, baseEst);
			/*double base[] = {
			0.9878,    0.0117, - 0.1553,         0,
			0.1557, - 0.0971,    0.9830,         0,
		   - 0.0036, - 0.9952, - 0.0977,         0,
		   - 0.6330,    0.2241,    0.6082 ,   1.0000 };
			baseEst.SetData(base, 16);*/

			cout << imageFolder << endl;
			cout << armMat << endl;
			cout << squareSize << endl;
			cout << baseEst << endl;
			CalCamArm(6, TBase, TEnd, cameraParams, TBaseStd, TEndStd, pixelErr, imageFolder, armMat, squareSize, baseEst);
			cout << TBase << endl;

			Eigen::Matrix4d result;
			mwArray2eigenMat(TBase, result);
			cout << result << endl;
			Eigen::Matrix4d result_inv = result.inverse();
			cout << result_inv << endl;
			system("pause");
		}
		catch (const mwException& e) {

			std::cerr << e.what() << std::endl;
			return -2;

		}
		catch (...) {

			std::cerr << "Unexpected error thrown" << std::endl;
			return -3;

		}

		// Call the application and library termination routine
		CalCamArm64Terminate();
	}

	// mclTerminateApplication shuts down the MATLAB Runtime.
	// You cannot restart it by calling mclInitializeApplication.
	// Call mclTerminateApplication once and only once in your application.

	mclTerminateApplication();

	return 0;
}

// The main routine. On the Mac, the main thread runs the system code, and
// user code must be processed by a secondary thread. On other platforms, 
// the main thread runs both the system code and the user code.

int main(int argc, const char **argv) {

	// Call application and library initialization. Perform this 
	// initialization before calling any API functions or
	// Compiler SDK-generated libraries.

	if (!mclInitializeApplication(nullptr, 0)) {

		std::cerr << "Could not initialize the application properly" << std::endl;
		return -1;

	}

	return mclRunMain(static_cast<mclMainFcnType>(run_main), argc, argv);

}


//int main()
//{
//	mclmcrInitialize();
//	if(!mclInitializeApplication(NULL, 0))
//		return -1;
//	
//	if (!sum_prodInitialize())  //必须写
//	{
//		system("pause");
//		exit(EXIT_FAILURE);
//	}
//	double p[2][2] = { 1, 2, 3, 4 };
//	mwArray X(2, 2, mxDOUBLE_CLASS);
//	mwArray Y(2, 2, mxDOUBLE_CLASS);
//	mwArray SUM(2, 2, mxDOUBLE_CLASS);
//	mwArray PROD(2, 2, mxDOUBLE_CLASS);
//	for (int i = 0; i < 2; i++)
//	{
//		for (int j = 0; j < 2; j++)
//		{
//			X(i + 1, j + 1) = p[i][j];
//			Y(i + 1, j + 1) = p[i][j];
//		}
//	}
//	sum_prod(2, SUM, PROD, X, Y);
//
//	double  sum[2][2], prod[2][2];
//	for (int i = 0; i < 2; i++)
//	{
//		for (int j = 0; j < 2; j++)
//		{
//			sum[i][j] = SUM(i + 1, j + 1);
//			prod[i][j] = PROD(i + 1, j + 1);
//		}
//	}
//	for (int i = 0; i < 2; i++)
//	{
//		for (int j = 0; j < 2; j++)
//		{
//			cout << sum[i][j];
//		}
//		cout << endl;
//	}
//	cout << endl;
//	for (int i = 0; i < 2; i++)
//	{
//		for (int j = 0; j < 2; j++)
//		{
//			cout << prod[i][j];
//		}
//		cout << endl;
//	}
//	cout << endl;
//	system("pause");
//	return 0;
//}