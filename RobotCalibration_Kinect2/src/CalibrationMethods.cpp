#include "stdafx.h"
#include "CalibrationMethods.h"


CalibrationMethods::CalibrationMethods()
{
}


CalibrationMethods::~CalibrationMethods()
{
}

Matrix4d CalibrationMethods::conversionOFcoordinates(Eigen::Vector3d O, Eigen::Vector3d X, Eigen::Vector3d Y) {
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

	Eigen::Vector4d Xt = Ry * Rx*X4;
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

	std::ofstream ofsCalib("../data/robot2cal.txt");
	for (int i = 0; i < 4; i++){
		for (int j = 0; j < 4; j++)
		{
			ofsCalib << R(i, j) << " ";
		}
		ofsCalib << std::endl;
	}

	ofsCalib.close();

	return R;
}

//单目
//相机外参 cam2cal
//像素点 position
//相机内参 camera_ins_H
Eigen::Vector3d CalibrationMethods::cam_cal_robot(const Eigen::Matrix4d& cam2cal, const Eigen::Matrix4d& robot2cal, const Eigen::Vector2d& position, const Eigen::Vector4d& camera_ins_H) {
	Eigen::Vector3d result;

	cout << "cam2cal:" << cam2cal << endl;
	
	//相机内参
	//Eigen::Matrix3Xd  camera_ins;
	Eigen::MatrixXd camera_ins = Eigen::MatrixXd::Zero(3, 4);
	camera_ins << camera_ins_H[0], 0, camera_ins_H[2], 0,
		0, camera_ins_H[1], camera_ins_H[3], 0,
		0, 0, 1, 0;
	cout << "camera_ins:" << camera_ins << endl;

	//像素坐标系转到世界坐标系(忽略Z轴，因为z轴坐标相同，)
	Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(3, 4);
	mat = camera_ins * cam2cal;
	cout <<"mat:" << mat << endl;

	//求解线性方程组
	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2,2);
	A << position.x() *mat(2, 0) - mat(0, 0), position.x()*mat(2, 1) - mat(0, 1),
		position.y()*mat(2, 0) - mat(1, 0), position.y()*mat(2, 1) - mat(1, 1);
	cout << "A:" << A << endl;

	Eigen::Vector2d B;
	B << mat(0, 3) - position.x()*mat(2, 3), mat(1, 3) - position.y()*mat(2, 3);
	cout << "B:" << B << endl;

	Eigen::Vector2d X = A.colPivHouseholderQr().solve(B);
	cout << "X:" << X << endl;
	//X = A.llt().solve(B);
	//X = A.ldlt().solve(B);

	Eigen::Vector4d coord(X.x() * 1000, X.y() * 1000, 0, 1);
	Eigen::Vector4d coord_robot;
	
	coord_robot = robot2cal * coord;
	result.x() = coord_robot[0];
	result.y() = coord_robot[1];
	result.z() = coord_robot[2];
	cout << "result：" << result << endl;
	return result;
}

