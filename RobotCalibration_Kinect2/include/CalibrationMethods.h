#pragma once

#include <iostream>
#include <Eigen\Dense>

using namespace std;
using namespace Eigen;
class CalibrationMethods
{
public:
	CalibrationMethods();
	~CalibrationMethods();

public:
	static Matrix4d conversionOFcoordinates(Eigen::Vector3d O, Eigen::Vector3d X, Eigen::Vector3d Y);
	static Eigen::Vector3d cam_cal_robot(const Eigen::Matrix4d& cam2cal, const Eigen::Matrix4d& cal2robot, const Eigen::Vector2d& position, const Eigen::Vector4d& camera_ins_H);
};

