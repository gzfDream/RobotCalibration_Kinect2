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
	printf("  s  六点法标定");
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

	double markerRealSize = 3;
	CameraCalibration cal = CameraCalibration(markerRealSize);

	//相机内参
	Camera_Intrinsics camera_ins_H;
	camera_ins_H.FLX = 1053.918;
	camera_ins_H.FLY = 1053.918;
	camera_ins_H.PPX = 942.797;
	camera_ins_H.PPY = 552.087;

	// 相机畸变
	double distCoeffD[5] = { -0.3766702758870145, 0.0336750411497953, 0.005424035460253849, 0.003744876142253783, -0.5389160914426057 };

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

			imgP.getHDImage("../data/img/");
			break;
		}
		case 'i': //棋盘格相机标定
			cout << "开始获得相机内参" << endl;
			cal.internal_reference_calibration(img_internal_file, cam_internal_file);
			break;
		
		case 'e':
			cout << "开始获得相机外参" << endl;
			cal.external_reference_calibration(camera_ins_H, distCoeffD, cam_cal_file, cam_external_file);
			break;
		case 'p':
		{
			cout << "获得标定结果" << endl;
			CalibrationMethods c;
			c.Method_BoardOnRobot(robot_pose_file, cam_external_file, result_file);
			break;
		}
		
		case 's':
			break;

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