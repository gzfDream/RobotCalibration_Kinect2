#pragma once

#include "stdafx.h"

#include "CalibrationMethods.h"
#include "ImgProcess.h"
#include "CalChessboard.h"
#include <opencv2/core/eigen.hpp>

void printMenu() {
	printf("Commands:\n");
	printf("  g  采集高清图片\n");
	printf("  i  计算内参\n");
	printf("  e  计算外参\n");
	printf("  p  \n");
	printf("  r  \n");
	printf("  d  \n");
	printf("  h  \n");
	printf("  o  \n");
	printf("  s  \n");
	printf("  q  \n");
}


void CalibrationFunc() {

	//初始化相机Kinect
	ImgProcess imgP;
	HRESULT hr = imgP.init_kinect();

	//相机内参
	Camera_Intrinsics camera_ins_H;
	camera_ins_H.FLX = 1053.918;
	camera_ins_H.FLY = 1053.918;
	camera_ins_H.PPX = 942.797;
	camera_ins_H.PPY = 552.087;

	// 相机到棋盘格变换矩阵(cv)
	cv::Mat cam2cal;

	// 相机到棋盘格变换矩阵(eigen)
	Eigen::Matrix4d m_cam2cal;
	Eigen::Matrix4d m_robot2cal;
	vector<cv::Point> vec_center;
	vector<Eigen::Vector3d> vec_world;
	cv::Point center;
	Eigen::Vector2d m_center;

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
			imgP.getHDImage("../data/img/");
			break;

		case 'c': //棋盘格相机标定（相机到棋盘格）
			cam2cal = imgP.calibration(camera_ins_H);
			cv::cv2eigen(cam2cal, m_cam2cal);
			// cout << "cam2cal: " << cam2cal << endl;
			cout << "m_cam2cal: " << m_cam2cal << endl;
			break;

		case 'p': //三点法标定
		{
			
			Eigen::Vector3d pos_O, pos_X, pos_Y;

			cout << "please input the three points(X,Y,Z):" << endl;
			cout << "point O:" << endl;
			cin >> pos_O.x() >> pos_O.y() >> pos_O.z();

			cout << "point X:" << endl;
			cin >> pos_X.x() >> pos_X.y() >> pos_X.z();

			cout << "point Y:" << endl;
			cin >> pos_Y.x() >> pos_Y.y() >> pos_Y.z();

			m_robot2cal = CalibrationMethods::conversionOFcoordinates(pos_O, pos_X, pos_Y);

			break;
		}

		case 'r': //
			imgP.getHDImage("../data/sort/");
			break;

		case 'd': //
			//vec_center = imgP.colorSub(55, 68, 110, 180, 100, 140);
			imgP.colorSubRT(55, 68, 110, 180, 100, 140, vec_center);
			for (int i = 0; i < vec_center.size(); i++)
				cout << vec_center[i] << endl;

			break;

		case 'h': //
			imgP.HSVcheck("../data/sort/0.png");
			break;

		case 'o':
		{
			Eigen::Vector4d camera_ins;
			camera_ins[0] = camera_ins_H.FLX;
			camera_ins[1] = camera_ins_H.FLY;
			camera_ins[2] = camera_ins_H.PPX;
			camera_ins[3] = camera_ins_H.PPY;
			vec_world.clear();
			for (int i = 0; i < vec_center.size(); i++) {
				m_center.x() = vec_center[i].x;
				m_center.y() = vec_center[i].y;
				//cout << CalibrationMethods::cam_cal_robot(m_cam2cal, m_robot2cal, m_center, camera_ins) << endl;
				Eigen::Vector3d pp = CalibrationMethods::cam_cal_robot(m_cam2cal, m_robot2cal, m_center, camera_ins);
				vec_world.push_back(pp);
			}

			break;
		}
		case 's':
		{
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