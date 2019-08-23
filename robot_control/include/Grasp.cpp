#include "Grasp.h"
#include "predict_post.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
Grasp::Grasp() {

}

Grasp::Grasp(std::string url_, const Camera_Intrinsics& cam_in_) {
	setUrl(url_);
	setCamera_Intrinsics(cam_in_);
}

Grasp::~Grasp() {

}

static void drawbox(const cv::Mat& rgb_m, std::vector<cv::Point> ps) {
	cv::line(rgb_m, ps[0], ps[1], cv::Scalar(255, 0, 0), 4);
	cv::line(rgb_m, ps[1], ps[2], cv::Scalar(255, 0, 0), 4);
	cv::line(rgb_m, ps[2], ps[3], cv::Scalar(255, 0, 0), 4);
	cv::line(rgb_m, ps[3], ps[0], cv::Scalar(255, 0, 0), 4);
	cv::imshow("", rgb_m);
}


bool Grasp::getPixelDepth(const std::vector<cv::Point>& pixels, std::string depth_path, const cv::Point& startPoint, std::vector<double> &depth) {
	cv::Mat depth_image = cv::imread(depth_path, CV_16U);

	if (depth_image.type() != CV_16U || depth_image.total() == 0) {
		std::cout << "error: 图片未找到！" << std::endl;
		return false;
	}

	//cv::Mat out = depth_image.clone();
	//// cv::GaussianBlur(depth_image, out, cv::Size(5,5), 0);
	//cv::medianBlur(depth_image, out, 5);
	//cv::imshow("", out);
	//cv::waitKey(0);

	double depth_scale_unit = 0.001;
	double d = 0;
	int sum = 0;
	for (int i = 0; i < pixels.size(); ++i) {
		int x = pixels[i].x + startPoint.x;
		int y = pixels[i].y + startPoint.y;
		double d = depth_image.at<cv::uint16_t>(x, y);
		/*if (d <= 0.0001) {
			for (int m = -6; m <= 6; m++) {
				for (int n = -6; n <= 6; n++) {
					double tmp = depth_image.at<cv::uint8_t>(x + m, y + n);
					std::cout << tmp << " ";
					if (tmp >= 0.0001) {
						d += tmp;
						sum++;
					}
				}
				std::cout << std::endl;
			}

			if (sum != 0)
				d /= sum;
			sum = 0;
			if (d <= 0.0001) {
				std::cout << "error深度为0！" << std::endl;
				return false;
			}
		}*/
		depth.push_back(d * depth_scale_unit);
	}

	return true;
}


bool Grasp::ImgPixel2CameraPosition(const std::vector<cv::Point>& pixels, const cv::Point& startPoint, std::vector<cv::Point2d>& camera_xy) {
	// 参考链接：https://blog.csdn.net/xholes/article/details/80599802
	//相机内参
	/*
	Eigen::MatrixXd camera_ins = Eigen::MatrixXd::Zero(3, 4);
	camera_ins << cam_in.FLX, 0, cam_in.PPX, 0,
		0, cam_in.FLY, cam_in.PPY, 0,
		0, 0, 1, 0;
	std::cout << "camera_ins:" << camera_ins << std::endl;
	*/

	if (pixels.empty()) {
		std::cout << "error: 输入为空！" << std::endl;
		return false;
	}

	for (int i = 0; i < pixels.size(); ++i) {
		cv::Point2d p;
		p.x = (pixels[i].x + startPoint.x - cam_in.PPX) / cam_in.FLX;
		p.y = (pixels[i].y + startPoint.y - cam_in.PPY) / cam_in.FLY;

		//std::cout << "p.x: " << p.x << std::endl;
		//std::cout << "p.y: " << p.y << std::endl;

		/*double r_2 = p.x*p.x + p.y*p.y;
		cv::Point2d p_ = (1 + cam_in.distCoeffD[0] * r_2 + cam_in.distCoeffD[1] * r_2*r_2 + cam_in.distCoeffD[2] * pow(r_2, 3)) * p
			+ cv::Point2d(2 * cam_in.distCoeffD[3] * p.x*p.y + cam_in.distCoeffD[4] * (r_2 + 2 * p.x*p.x),
				2 * cam_in.distCoeffD[3] * (r_2 + 2 * p.y*p.y) + 2 * cam_in.distCoeffD[4] * p.x *p.y);*/
		camera_xy.push_back(p);
	}

	return true;
}


// 步骤1：获得相机坐标系下bounding box的坐标值
int Grasp::getBoundingBox(std::string rgb_img, std::string depth_img, const cv::Point& startPoint, std::vector<cv::Point3d>& box_points)
{
	// <suck_box , ori_box>
	std::pair<std::vector<cv::Point>, std::vector<cv::Point>> ps;
	int back_code = Predict_Post::start_predict((char*)url.c_str(), (char*)rgb_img.c_str(), (char*)depth_img.c_str(), ps);

	cv::Mat rgb_m = cv::imread(rgb_img);
	drawbox(rgb_m, ps.first);
	drawbox(rgb_m, ps.second);
	// cv::Mat depth_m = cv::imread(depth_img);
	// drawbox(depth_m, ps.first);
	// drawbox(depth_m, ps.second);
	cv::waitKey(0);

	if (ps.first.size() != ps.second.size() || ps.second.size() != 4) {
		std::cout << "error：返回数据错误！" << std::endl;
		return -1;
	}
	else {
		box_points.clear();
		std::vector<double> depths;
		std::vector<cv::Point2d> cameras_suck;
		std::vector<cv::Point2d> cameras_box;
		cv::Point3d point;
		std::string depth_img_ori = depth_img.substr(0, depth_img.size() - 9) + ".png";
		if (getPixelDepth(ps.second, depth_img_ori, startPoint, depths) && ImgPixel2CameraPosition(ps.first, startPoint, cameras_suck) && ImgPixel2CameraPosition(ps.second, startPoint, cameras_box)) {
			// 吸盘位置
			for (int i=0; i<ps.first.size(); ++i){
				point.x = cameras_suck[i].x * 1;
				point.y = cameras_suck[i].y * 1;
				point.z = 1;

				box_points.push_back(point);
			}
			
			// bounding box
			for (int i=0; i<ps.second.size(); ++i) {
				point.x = cameras_box[i].x * depths[i];
				point.y = cameras_box[i].y * depths[i];
				point.z = depths[i];

				box_points.push_back(point);
			}
		}
		else{
			return -1;
		}
	}

	return box_points.size();
}


// 步骤2：计算抓取位置，方向，旋转
void Grasp::computeGraspPosition(const std::vector<cv::Point3d>& box_points, cv::Point3d& position, cv::Vec3d& gnorm, cv::Mat& rotate_mat)
{
	// 求抓取方向
	Eigen::Vector3d v1(box_points[4].x - box_points[6].x, box_points[4].y - box_points[6].y, box_points[4].z - box_points[6].z);
	Eigen::Vector3d v2(box_points[5].x - box_points[7].x, box_points[5].y - box_points[7].y, box_points[5].z - box_points[7].z);
	v1.normalize();
	v2.normalize();
	Eigen::Vector3d norm = v1.cross(v2);
	cv::eigen2cv(norm, gnorm);

	// 抓取位置
	position.z = (box_points[4].z + box_points[5].z + box_points[6].z + box_points[7].z) / 4;
	position.x = (box_points[0].x + box_points[2].x + box_points[1].x + box_points[3].x) / 4 * position.z;
	position.y = (box_points[0].y + box_points[2].y + box_points[1].y + box_points[3].y) / 4 * position.z;
	

	Eigen::Vector3d v3(box_points[0].x - box_points[1].x, box_points[0].y - box_points[1].y, 0);
	Eigen::Vector3d v4(box_points[1].x - box_points[2].x, box_points[1].y - box_points[2].y, 0);
	double v3_norm = v3.norm();
	double v4_norm = v4.norm();	

	v3.normalize();
	v4.normalize();

	double angle = 0.;
	Eigen::Matrix3d rot;
	Eigen::Matrix4d trans;
	if (v3_norm > v4_norm) {
		 angle = std::acos(std::abs(v3.dot(Eigen::Vector3d(1., 0, 0))));

		 Eigen::AngleAxisd r_z(angle, Eigen::Vector3d(0., 0., 1.));
		 if (std::abs(std::abs((r_z*v4).dot(Eigen::Vector3d(0., 1., 0.))) - 1) < 0.001)
			 rot = r_z.matrix().inverse();
		 else
			 rot = r_z.matrix();
	}
	else {
		angle = std::acos(std::abs(v4.dot(Eigen::Vector3d(1., 0, 0))));

		if (angle > PI / 2)
			angle = PI - angle;

		Eigen::AngleAxisd r_z(angle, Eigen::Vector3d(0., 0., 1.));
		if (std::abs(std::abs((r_z*v3).dot(Eigen::Vector3d(0., 1., 0.))) - 1) < 0.001)
			rot = r_z.matrix().inverse();
		else
			rot = r_z.matrix();
			
	}

	// 对齐Z轴
	Eigen::Vector3d Zn = norm;
	double d = sqrt(pow(Zn(1), 2) + pow(Zn(2), 2));

	Eigen::Matrix3d Rx(3, 3), Ry(3, 3), Rz(3, 3);

	if (d == 0)
	{
		Rx << 1, 0, 0, 
			0, 1, 0,
			0, 0, 1;
	}
	else {
		Rx << 1, 0, 0,
			0, Zn(2) / d, -Zn(1) / d,
			0, Zn(1) / d, Zn(2) / d;
	}
	Ry << d, 0, -Zn(0),
		0, 1, 0,
		Zn(0), 0, d;

	Rz = Ry * Rx;

	std::cout << Rz << std::endl;
	
	std::cout << rot << std::endl;

	Eigen::Matrix3d rotate_z = Rz.transpose() * rot;
	trans << rotate_z(0, 0), rotate_z(0, 1), rotate_z(0, 2), position.x,
		rotate_z(1, 0), rotate_z(1, 1), rotate_z(1, 2), position.y,
		rotate_z(2, 0), rotate_z(2, 1), rotate_z(2, 2), position.z,
			 0, 0, 0, 1;
	cv::eigen2cv(trans, rotate_mat);

					// !!!注意！！！//
	//trans*camHbase*baseHend才是最终的抓取的变换矩阵//
	
}

