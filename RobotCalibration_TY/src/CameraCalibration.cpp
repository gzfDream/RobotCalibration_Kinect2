#include "stdafx.h"

#include "CameraCalibration.h"
#include <stdio.h>  
#include <io.h>  
#include <Windows.h> 



// 得到路径下的所有文件名，存到vector中
static void getAllFiles(std::string path, std::vector<std::string>& files)
{
	intptr_t   hFile = 0;

	struct _finddata_t fileinfo;
	std::string p;
	std::cout << p.assign(path).append("\\*").c_str() << std::endl;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					//files.push_back(p.assign(path).append("\\").append(fileinfo.name) );
					getAllFiles(p.assign(path).append("\\").append(fileinfo.name), files);
				}
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}


CameraCalibration::CameraCalibration(const double &markerRealSize_, const cv::Size& chessboard_size_)
{
	/*m_markerCorners3d.push_back(cv::Point3f(0., 0., 0.));
	m_markerCorners3d.push_back(cv::Point3f(4. * markerRealSize, 0., 0.));
	m_markerCorners3d.push_back(cv::Point3f(8. * markerRealSize, 0., 0.));
	m_markerCorners3d.push_back(cv::Point3f(8. * markerRealSize, 5. * markerRealSize, 0.));
	m_markerCorners3d.push_back(cv::Point3f(4. * markerRealSize, 5. * markerRealSize, 0.));
	m_markerCorners3d.push_back(cv::Point3f(0., 5. * markerRealSize, 0.));
	m_markerCorners3d.push_back(cv::Point3f(2. * markerRealSize, 1. * markerRealSize, 0.));
	m_markerCorners3d.push_back(cv::Point3f(6. * markerRealSize, 1. * markerRealSize, 0.));
	m_markerCorners3d.push_back(cv::Point3f(6. * markerRealSize, 4. * markerRealSize, 0.));
	m_markerCorners3d.push_back(cv::Point3f(2. * markerRealSize, 4. * markerRealSize, 0.));*/

	chessboard_size = chessboard_size_;
	markerRealSize = markerRealSize_;

	for (int i = 0; i < chessboard_size.height; i++)
	{
		for (int j = 0; j < chessboard_size.width; j++)
		{
			cv::Point3f realPoint;
			/* 假设标定板放在世界坐标系中z=0的平面上 */
			realPoint.x = j * markerRealSize;
			realPoint.y = i * markerRealSize;
			realPoint.z = 0;
			m_markerCorners3d.push_back(realPoint);
		}
	}
}


CameraCalibration::~CameraCalibration()
{
}


bool CameraCalibration::corner_detection(std::string &imgpath) {
	cv::Mat img_color = cv::imread(imgpath, cv::IMREAD_COLOR);

	//翻转
	// flip(img_color, image_color, 1);
	image_color = img_color;

	cv::Mat image_gray;
	cv::cvtColor(image_color, image_gray, cv::COLOR_BGR2GRAY);

	std::vector<cv::Point2f> corners, corner_t;

	bool ret = cv::findChessboardCorners(image_gray, chessboard_size,
		//cv::Size(6, 9),
		corners//,
		//cv::CALIB_CB_ADAPTIVE_THRESH |
		//CV_CALIB_CB_FAST_CHECK |
		//cv::CALIB_CB_NORMALIZE_IMAGE
	);

	if (ret) {
		//指定亚像素计算迭代标注  
		cv::TermCriteria criteria = cv::TermCriteria(
			cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
			40,
			0.1);

		//亚像素检测  
		cv::cornerSubPix(image_gray, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);

		/*m_markerCorners2d.push_back(corners[0]);
		m_markerCorners2d.push_back(corners[24]);
		m_markerCorners2d.push_back(corners[48]);
		m_markerCorners2d.push_back(corners[53]);
		m_markerCorners2d.push_back(corners[29]);
		m_markerCorners2d.push_back(corners[5]);
		m_markerCorners2d.push_back(corners[13]);
		m_markerCorners2d.push_back(corners[37]);
		m_markerCorners2d.push_back(corners[40]);
		m_markerCorners2d.push_back(corners[16]);*/

		/*m_markerCorners2d.push_back(corners[5]);
		m_markerCorners2d.push_back(corners[29]);
		m_markerCorners2d.push_back(corners[53]);
		m_markerCorners2d.push_back(corners[48]);
		m_markerCorners2d.push_back(corners[24]);
		m_markerCorners2d.push_back(corners[0]);
		m_markerCorners2d.push_back(corners[16]);
		m_markerCorners2d.push_back(corners[40]);
		m_markerCorners2d.push_back(corners[37]);
		m_markerCorners2d.push_back(corners[13]);*/

		m_markerCorners2d = corners;

		return true;
	}
	else
		return false;
}


// 计算一张图片对应的外参（private）
bool CameraCalibration::calibration(const cv::Matx33f& camera_matrix, const cv::Matx<float, 5, 1>& distortion_coefficients, cv::Mat &rvec, cv::Mat &tvec) {

	for (int i = 0; i < m_markerCorners2d.size(); i++)
	{
		cv::circle(image_color, m_markerCorners2d[i], 1, cv::Scalar(255, 5 * i, 255), 2);
		cv::putText(image_color, std::to_string(i), m_markerCorners2d[i], 1, 1, cv::Scalar(255, 0, 0));
	}
	cv::circle(image_color, m_markerCorners2d[0], 3, cv::Scalar(0, 0, 255), 4);
	cv::arrowedLine(image_color, m_markerCorners2d[0], m_markerCorners2d[chessboard_size.width-1], cv::Scalar(0, 0, 255), 2);
	cv::arrowedLine(image_color, m_markerCorners2d[0], m_markerCorners2d[chessboard_size.width*(chessboard_size.height-1)], cv::Scalar(0, 255, 0), 2);

	cv::Mat objPM;
	cv::Mat(m_markerCorners3d).convertTo(objPM, CV_32F);

	//cv::solvePnPRansac(objPM, cv::Mat(m_markerCorners2d), camera_matrix, distortion_coefficients, rvec, tvec);
	bool ret = cv::solvePnP(objPM, cv::Mat(m_markerCorners2d), camera_matrix, distortion_coefficients, rvec, tvec);

	if (ret) {
		cv::Mat objPM1;
		//m_markerCorners3d.push_back(cv::Point3f(22.2, -11.7, 6.4));
		cv::Mat(m_markerCorners3d).convertTo(objPM1, CV_32F);

		std::vector<cv::Point2f> projectedPoints;
		cv::projectPoints(objPM1, rvec, tvec, camera_matrix, distortion_coefficients, projectedPoints);

		for (unsigned int i = 0; i < projectedPoints.size(); ++i)
		{
			circle(image_color, projectedPoints[i], 1.5, cv::Scalar(255, 0, 0), -1, 8);
		}
		cv::imshow("chessboard corners", image_color);
		cv::waitKey(550);
		//cv::imshow("chessboard corners", image_color2);
		//cv::waitKey(0);
		// std::cout << "R:\n" << rvec << std::endl;
		// std::cout << "T:\n" << tvec << std::endl;

		return true;
	}
	else
		return false;
	
}



cv::Mat CameraCalibration::process_transMatrix(const cv::Mat& rvec, const cv::Mat& tvec) {
	cv::Mat trans_mat(4, 4, CV_32F), rotM;

	Rodrigues(rvec, rotM);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			trans_mat.at<float>(i, j) = rotM.at<float>(i, j);
		}
	}
	trans_mat.at<float>(0, 3) = tvec.at<float>(0, 0) / 100;
	trans_mat.at<float>(1, 3) = tvec.at<float>(1, 0) / 100;
	trans_mat.at<float>(2, 3) = tvec.at<float>(2, 0) / 100;

	trans_mat.at<float>(3, 0) = 0;
	trans_mat.at<float>(3, 1) = 0;
	trans_mat.at<float>(3, 2) = 0;
	trans_mat.at<float>(3, 3) = 1;

	// std::cout << "trans_mat:\n" << trans_mat << std::endl;

	//camHcal->calHcam
	trans_mat = trans_mat.inv();

	return trans_mat;
}


//计算外参
bool CameraCalibration::external_reference_calibration(Camera_Intrinsics camera_ins_H, double distCoeffD[5], 
														std::string imgpath, std::string calibFile, std::vector<cv::Mat>& vec_res) {

	// 设置相机内参
	cv::Matx33f camera_matrix;
	cv::Matx<float, 5, 1> distortion_coefficients; 

	double camD[9] = { camera_ins_H.FLX, 0, camera_ins_H.PPX,
							0, camera_ins_H.FLY, camera_ins_H.PPY,
							0, 0, 1 };

	camera_matrix = cv::Mat(3, 3, CV_64FC1, camD);
	distortion_coefficients = cv::Mat(5, 1, CV_64FC1, distCoeffD);

	std::cout << "camera_matrix: " << camera_matrix << std::endl;
	std::cout << "distortion_coefficients: " << distortion_coefficients << std::endl;

	std::vector<std::string> img_files;
	getAllFiles(imgpath, img_files);

	cv::Mat trans_mat(4, 4, CV_32F);

	cv::Mat rvec, tvec;
	std::ofstream ofsCalib(calibFile);

	for (int i = 0; i < img_files.size(); i++)
	{
		if (corner_detection(img_files[i]) && calibration(camera_matrix, distortion_coefficients, rvec, tvec)) {
			trans_mat = process_transMatrix(rvec, tvec);
			vec_res.push_back(trans_mat);

			for (int i = 0; i < 4; i++) {
				for (int j = 0; j < 4; j++)
				{
					ofsCalib << trans_mat.at<float>(i, j) << " ";
				}
				ofsCalib << std::endl;
			}
			m_markerCorners2d.clear();
			image_color.release();
			rvec.release();
			tvec.release();
		}
		else
		{
			std::cout << "error：计算外参失败！" <<std::endl;
			ofsCalib.close();
			cv::destroyAllWindows();
			return false;
		}
	}

	ofsCalib.close();
	cv::destroyAllWindows();
	return true;
}


//计算单张图片外参
bool CameraCalibration::external_reference_calibration_singleImage(Camera_Intrinsics camera_ins_H, double distCoeffD[5], std::string imgpath, cv::Mat& external_mat)
{
	// 设置相机内参
	cv::Matx33f camera_matrix;
	cv::Matx<float, 5, 1> distortion_coefficients;

	double camD[9] = { camera_ins_H.FLX, 0, camera_ins_H.PPX,
							0, camera_ins_H.FLY, camera_ins_H.PPY,
							0, 0, 1 };

	camera_matrix = cv::Mat(3, 3, CV_64FC1, camD);
	distortion_coefficients = cv::Mat(5, 1, CV_64FC1, distCoeffD);

	std::cout << "camera_matrix: " << camera_matrix << std::endl;
	std::cout << "distortion_coefficients: " << distortion_coefficients << std::endl;

	cv::Mat rvec, tvec;

	if (corner_detection(imgpath) && calibration(camera_matrix, distortion_coefficients, rvec, tvec)) {
		external_mat = process_transMatrix(rvec, tvec);

		m_markerCorners2d.clear();
		image_color.release();
		rvec.release();
		tvec.release();
	}
	else
	{
		std::cout << "error：计算外参失败！" << std::endl;
		cv::destroyAllWindows();
		return false;
	}

	cv::destroyAllWindows();
	return true;
}


bool CameraCalibration::external_reference_calibration_singleImage_MATLAB(Camera_Intrinsics camera_ins_H, double distCoeffD[5], std::string img_path, cv::Mat &external_mat)
{
	if (!mclInitializeApplication(nullptr, 0)) {
		std::cerr << "Could not initialize the application properly" << std::endl;
		return false;
	}

	if (!getCameraExtrinsics_64Initialize()) {
		std::cerr << "Could not initialize the library properly" << std::endl;
		return false;
	}
	else {
		try {

			// 输出参数
			mwArray calHcam;
			// 输入参数
			mwArray image_path(img_path.c_str());
			mwArray IntrinsicMatrix(3, 3, mxDOUBLE_CLASS, mxREAL);;
			mwArray Distortion(1, 5, mxDOUBLE_CLASS, mxREAL);

			double tmp[] = {
				camera_ins_H.FLX, 0, 0,
				0, camera_ins_H.FLY, 0,
				camera_ins_H.PPX, camera_ins_H.PPY, 1
			};
			IntrinsicMatrix.SetData(tmp, 9);
			Distortion.SetData(distCoeffD, 5);
			std::cout << IntrinsicMatrix <<std::endl;
			std::cout << Distortion << std::endl;

			// 计算外参
			getCameraExtrinsics(1, calHcam, image_path, IntrinsicMatrix, Distortion);
			std::cout << calHcam << std::endl;

			double mat_[16] = {
				calHcam.Get(1, 1), calHcam.Get(1, 5), calHcam.Get(1, 9), calHcam.Get(1, 13),
				calHcam.Get(1, 2), calHcam.Get(1, 6), calHcam.Get(1, 10), calHcam.Get(1, 14),
				calHcam.Get(1, 3), calHcam.Get(1, 7), calHcam.Get(1, 11), calHcam.Get(1, 15),
				calHcam.Get(1, 4), calHcam.Get(1, 8), calHcam.Get(1, 12), calHcam.Get(1, 16) };

			external_mat = cv::Mat(4, 4, CV_64FC1, mat_);
			std::cout << external_mat << std::endl;
		}
		catch (const mwException& e) {
			std::cerr << e.what() << std::endl;
			return false;
		}
		catch (...) {
			std::cerr << "Unexpected error thrown" << std::endl;
			return false;
		}

		getCameraExtrinsics_64Terminate();
	}

	mclTerminateApplication();

	return true;
}



// 计算相机内参
void CameraCalibration::internal_reference_calibration(std::string img_path, std::string internal_file, Camera_Intrinsics& cam_in, double distCoeffD[5]) {
	//ifstream fin("calibdata.txt"); /* 标定所用图像文件的路径 */

	std::vector<std::string> img_files;

	getAllFiles(img_path, img_files);
	std::ofstream fout(internal_file);  /* 保存标定结果的文件 */
													 //读取每一幅图像，从中提取出角点，然后对角点进行亚像素精确化	
	std::cout << "开始提取角点………………";
	int image_count = 0;  /* 图像数量 */
	cv::Size image_size;  /* 图像的尺寸 */
	cv::Size board_size = chessboard_size; // cv::Size(9, 6);    /* 标定板上每行、列的角点数 */
	std::vector<cv::Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
	std::vector<std::vector<cv::Point2f>> image_points_seq; /* 保存检测到的所有角点 */

	int count = -1;//用于存储角点个数。
	for (int index = 0; index < img_files.size(); index++) {
		image_count++;
		// 用于观察检验输出
		std::cout << "image_count = " << image_count << std::endl;
		/* 输出检验*/
		std::cout << "-->count = " << count;
		cv::Mat imageInput = cv::imread(img_files[index]);
		if (image_count == 1)  //读入第一张图片时获取图像宽高信息
		{
			image_size.width = imageInput.cols;
			image_size.height = imageInput.rows;
			std::cout << "image_size.width = " << image_size.width << std::endl;
			std::cout << "image_size.height = " << image_size.height << std::endl;
		}

		/* 提取角点 */
		if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
		{
			std::cout << "can not find chessboard corners!\n"; //找不到角点
			return;
		}
		else
		{
			cv::Mat view_gray;
			cvtColor(imageInput, view_gray, CV_RGB2GRAY);
			/* 亚像素精确化 */
			find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(5, 5)); //对粗提取的角点进行精确化
																			//cornerSubPix(view_gray,image_points_buf,Size(5,5),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
			image_points_seq.push_back(image_points_buf);  //保存亚像素角点
														   /* 在图像上显示角点位置 */
			drawChessboardCorners(view_gray, board_size, image_points_buf, false); //用于在图片中标记角点
			imshow("Camera Calibration", view_gray);//显示图片
			cv::waitKey(500);//暂停0.5S		
		}
	}

	int total = image_points_seq.size();
	std::cout << "total = " << total << std::endl;
	int CornerNum = board_size.width*board_size.height;  //每张图片上总的角点数
	for (int ii = 0; ii < total; ii++)
	{
		if (0 == ii % CornerNum)// 24 是每幅图片的角点个数。此判断语句是为了输出 图片号，便于控制台观看 
		{
			int i = -1;
			i = ii / CornerNum;
			int j = i + 1;
			std::cout << "--> 第 " << j << "图片的数据 --> : " << std::endl;
		}
		if (0 == ii % 3)	// 此判断语句，格式化输出，便于控制台查看
		{
			std::cout << std::endl;
		}
		else
		{
			std::cout.width(10);
		}
		//输出所有的角点
		std::cout << " -->" << image_points_seq[ii][0].x;
		std::cout << " -->" << image_points_seq[ii][0].y;
	}
	std::cout << "角点提取完成！\n";

	//以下是摄像机标定
	std::cout << "开始标定………………";
	/*棋盘三维信息*/
	cv::Size square_size = cv::Size(markerRealSize, markerRealSize);  /* 实际测量得到的标定板上每个棋盘格的大小 cm*/
	std::vector<std::vector<cv::Point3f>> object_points; /* 保存标定板上角点的三维坐标 */
										   /*内外参数*/
	cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* 摄像机内参数矩阵 */
	std::vector<int> point_counts;  // 每幅图像中角点的数量
	cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
	std::vector<cv::Mat> tvecsMat;  /* 每幅图像的旋转向量 */
	std::vector<cv::Mat> rvecsMat; /* 每幅图像的平移向量 */
						  /* 初始化标定板上角点的三维坐标 */
	int i, j, t;
	for (t = 0; t < image_count; t++)
	{
		std::vector<cv::Point3f> tempPointSet;
		for (i = 0; i < board_size.height; i++)
		{
			for (j = 0; j < board_size.width; j++)
			{
				cv::Point3f realPoint;
				/* 假设标定板放在世界坐标系中z=0的平面上 */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	/* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
	for (i = 0; i < image_count; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}
	/* 开始标定 */
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	std::cout << "标定完成！\n";
	cv::destroyAllWindows();

	//对标定结果进行评价
	std::cout << "开始评价标定结果………………\n";
	double total_err = 0.0; /* 所有图像的平均误差的总和 */
	double err = 0.0; /* 每幅图像的平均误差 */
	std::vector<cv::Point2f> image_points2; /* 保存重新计算得到的投影点 */
	std::cout << "\t每幅图像的标定误差：\n";
	fout << "每幅图像的标定误差：\n";
	for (i = 0; i < image_count; i++)
	{
		std::vector<cv::Point3f> tempPointSet = object_points[i];
		/* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
		cv::projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		/* 计算新的投影点和旧的投影点之间的误差*/
		std::vector<cv::Point2f> tempImagePoint = image_points_seq[i];
		cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
		cv::Mat image_points2Mat = cv::Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = cv::norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);
		total_err += err /= point_counts[i];
		std::cout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << std::endl;
		fout << "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << std::endl;
	}
	std::cout << "总体平均误差：" << total_err / image_count << "像素" << std::endl;
	fout << "总体平均误差：" << total_err / image_count << "像素" << std::endl << std::endl;
	std::cout << "评价完成！" << std::endl;
	//保存定标结果  	
	std::cout << "开始保存定标结果………………" << std::endl;
	cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
	fout << "相机内参数矩阵：" << std::endl;
	fout << cameraMatrix << std::endl << std::endl;
	fout << "畸变系数：\n";
	fout << distCoeffs << std::endl << std::endl << std::endl;
	for (int i = 0; i < image_count; i++)
	{
		fout << "第" << i + 1 << "幅图像的旋转向量：" << std::endl;
		fout << tvecsMat[i] << std::endl;
		/* 将旋转向量转换为相对应的旋转矩阵 */
		cv::Rodrigues(tvecsMat[i], rotation_matrix);
		fout << "第" << i + 1 << "幅图像的旋转矩阵：" << std::endl;
		fout << rotation_matrix << std::endl;
		fout << "第" << i + 1 << "幅图像的平移向量：" << std::endl;
		fout << rvecsMat[i] << std::endl << std::endl;
	}
	std::cout << "完成保存" << std::endl;
	fout << std::endl;
	/************************************************************************
	显示定标结果
	*************************************************************************/
	/*cv::Mat mapx = cv::Mat(image_size, CV_32FC1);
	cv::Mat mapy = cv::Mat(image_size, CV_32FC1);
	cv::Mat R = cv::Mat::eye(3, 3, CV_32F);

	std::cout << "保存矫正图像" << std::endl;
	std::string imageFileName;
	std::stringstream StrStm;
	for (int i = 0; i != image_count; i++)
	{
		std::cout << "Frame #" << i + 1 << "..." << std::endl;
		initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
		StrStm.clear();
		imageFileName.clear();
		std::string filePath = "chess";
		StrStm << i + 1;
		StrStm >> imageFileName;
		filePath += imageFileName;
		filePath += ".bmp";
		cv::Mat imageSource = cv::imread(filePath);
		cv::Mat newimage = imageSource.clone();
		//另一种不需要转换矩阵的方式
		//undistort(imageSource,newimage,cameraMatrix,distCoeffs);
		cv::remap(imageSource, newimage, mapx, mapy, CV_INTER_LINEAR);
		StrStm.clear();
		filePath.clear();
		StrStm << i + 1;
		StrStm >> imageFileName;
		imageFileName += "_d.jpg";
		cv::imwrite(imageFileName, newimage);
	}
	*/
	
	std::cout << "保存结束" << std::endl;

	std::cout << cameraMatrix << std::endl;
	cam_in.FLX = cameraMatrix.at<double>(0, 0);
	cam_in.FLY = cameraMatrix.at<double>(1, 1);
	cam_in.PPX = cameraMatrix.at<double>(0, 2);
	cam_in.PPY = cameraMatrix.at<double>(1, 2);

	for(int i=0; i<distCoeffs.cols; i++)
		distCoeffD[i] = distCoeffs.at<double>(0,i);
}


// 计算相机内参 MATLAB
void CameraCalibration::internal_reference_calibration_MATLAB(std::string img_files, std::string internal_file, Camera_Intrinsics& cam_in, double distCoeffD[5])
{
	if (!mclInitializeApplication(nullptr, 0)) {
		std::cerr << "Could not initialize the application properly" << std::endl;
		return;
	}

	if (!cameraCalibrator_64Initialize()) {
		std::cerr << "Could not initialize the library properly" << std::endl;
		return;
	}
	else {
		try {
			// 返回值
			mwArray cameraParams;
			mwArray IntrinsicMatrix;
			mwArray Distortion;

			// 输入值
			mwArray imageFolder(img_files.c_str());
			int realSize[] = { markerRealSize*10 }; //mm
			mwArray squareSize(1, 1, mxINT32_CLASS, mxREAL);
			squareSize.SetData(realSize, 1);

			// std::cout << "imageFolder: " << imageFolder << std::endl;
			// std::cout << "squareSize: " << squareSize << std::endl;

			// 计算内参
			cameraCalibrator(3, cameraParams, IntrinsicMatrix, Distortion, imageFolder, squareSize);
			std::cout << IntrinsicMatrix << std::endl;
			std::cout << Distortion << std::endl;

			std::ofstream fout(internal_file);  /* 保存标定结果的文件 */
			fout << "相机内参数矩阵：" << std::endl;
			fout << IntrinsicMatrix << std::endl << std::endl;
			fout << "畸变系数：\n";
			fout << Distortion << std::endl << std::endl << std::endl;
			fout.close();

			cam_in.FLX = IntrinsicMatrix.Get(1, 1);
			cam_in.FLY = IntrinsicMatrix.Get(1, 5);
			cam_in.PPX = IntrinsicMatrix.Get(1, 7);
			cam_in.PPY = IntrinsicMatrix.Get(1, 8);

			distCoeffD[0] = Distortion.Get(1, 1);
			distCoeffD[1] = Distortion.Get(1, 2);
			distCoeffD[2] = Distortion.Get(1, 3);
			distCoeffD[3] = Distortion.Get(1, 4);
			distCoeffD[4] = Distortion.Get(1, 5);
		}
		catch (const mwException& e) {
			std::cerr << e.what() << std::endl;
			return;
		}
		catch (...) {
			std::cerr << "Unexpected error thrown" << std::endl;
			return;
		}

		cameraCalibrator_64Terminate();
	}

	mclTerminateApplication();

}

