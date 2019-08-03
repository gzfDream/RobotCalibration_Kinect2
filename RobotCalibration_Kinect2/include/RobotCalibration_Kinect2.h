#pragma once

#include "stdafx.h"

#include "CalibrationMethods.h"
#include "ImgProcess.h"
#include "CameraCalibration.h"


void printMenu() {
	printf("Commands:\n");
	printf("  g  采集高清图片\n");
	printf("  i  计算内参\n");
	printf("  e  计算外参\n");
	printf("  p  标定（棋盘格在机械臂上）\n");
	printf("  o  标定结果优化（棋盘格在机械臂上）\n");
	printf("  s  三点法标定 \n");
	printf("  q  退出\n");
}



void CalibrationFunc() {

	//文件存储位置（调试用）
	// 存储相机内参标定图片
	std::string img_internal_file = ".\\data\\Images_internal";
	// 存储相机内参标定结果
	std::string cam_internal_file = ".\\data\\internal_reference.txt";

	// 存储标定时图片
	std::string cam_cal_file = ".\\data\\Images\\";
	// 标定时机械臂末端的位姿
	std::string robot_pose_file = ".\\data\\robot.txt";
	// 相机外参
	std::string cam_external_file = ".\\data\\cam_cal_robot.txt";
	// 存储标定结果
	std::string result_file = ".\\data\\result.txt";

	double markerRealSize = 2;//cm
	CameraCalibration cal = CameraCalibration(markerRealSize);

	//相机内参
	Camera_Intrinsics camera_ins_H;
	camera_ins_H.FLX = 1933.037839984974;
	camera_ins_H.FLY = 1949.554793781403;
	camera_ins_H.PPX = 983.8643660960072;
	camera_ins_H.PPY = 495.4767992148006;

	// 相机畸变
	double distCoeffD[5] = { 0.04012892375375712, -0.007314429288468308, -0.01186231085084112, -0.003724066345932373, 0.3225867897472355 };

	// 标定结果
	cv::Mat res_mat;

	//打印菜单
	printMenu();
	string line;
	bool running = true;
	while (running)
	{
		printf("please input the command >>> \n");
		std::getline(std::cin, line);
		switch (line[0]) {
		case 'g': //获得图片
		{
			//初始化相机Kinect
			ImgProcess imgP;
			HRESULT hr = imgP.init_kinect();

			imgP.getHDImage(cam_cal_file);

			break;
		}

		case 'i': //棋盘格相机标定
			std::cout << "开始获得相机内参" << endl;
			cal.internal_reference_calibration(img_internal_file, cam_internal_file, camera_ins_H, distCoeffD);
			break;

		case 'e': 
		{
			std::cout << "开始获得相机外参" << endl;
			cout << "-*-*-*-*-是否重新计算相机内参（输入1/0）-*-*-*-*-" << endl;
			int ok = 0;
			cin >> ok;
			if (ok) {
				std::cout << "开始获得相机内参" << endl;
				cal.internal_reference_calibration(img_internal_file, cam_internal_file, camera_ins_H, distCoeffD);
			}

			vector<cv::Mat> res;
			cal.external_reference_calibration(camera_ins_H, distCoeffD, cam_cal_file, cam_external_file, res);
			break;
		}
			
		case 'p':
		{
			std::cout << "获得标定结果" << endl;
			CalibrationMethods c;
			c.Method_BoardOnRobot(robot_pose_file, cam_external_file, result_file, res_mat);
			break;
		}

		case 'o':
		{
			cv::Mat result;
			CalibrationMethods c;
			c.Calibration_Optimization(cam_cal_file, robot_pose_file, res_mat, result);
			std::cout << result << endl;
			
			break;
		}

		case 's':
		{
			std::cout << "开始三点法标定" << endl;
			std::cout << "请示教机械臂到标定板坐标系原点，x轴一点，xoy平面内一点（不一定在y轴）" << endl;
			cv::Point3d pos_O, pos_X, pos_Y;

			std::cout << "please input the three points(X,Y,Z):" << endl;
			std::cout << "point O:" << endl;
			cin >> pos_O.x >> pos_O.y >> pos_O.z;

			std::cout << "point X:" << endl;
			cin >> pos_X.x >> pos_X.y >> pos_X.z;

			std::cout << "point Y:" << endl;
			cin >> pos_Y.x >> pos_Y.y >> pos_Y.z;

			std::cout << "用相机拍摄标定板, 可拍摄多张" << endl;
			//初始化相机Kinect
			ImgProcess imgP;
			HRESULT hr = imgP.init_kinect();
			imgP.getHDImage(cam_cal_file);

			vector<cv::Mat> res;
			cal.external_reference_calibration(camera_ins_H, distCoeffD, cam_cal_file, cam_external_file, res);

			cv::Mat ex_mat;
			for (auto mat_ : res)
				ex_mat += mat_;

			ex_mat = ex_mat / res.size();

			CalibrationMethods c;
			c.Method_ThreePointsCalibration(pos_O, pos_X, pos_Y, ex_mat, res_mat);
			break;
		}

		case 'q':
			running = false;
			break;

		default:
			printMenu();
			break;
		}
	}

	return;
}