#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#define PI 3.1415926
bool align_rectangle(std::vector<cv::Point2d> src, std::vector<cv::Point2d> dst, cv::Mat& rotate_) {
	cv::Mat vec_src(src[1] - src[0]);
	cv::Mat vec_dst(dst[1] - dst[0]);

	double theta = 0.;

	if (cv::norm(vec_src) != 0 && cv::norm(vec_dst) != 0) {
		theta = acosf(vec_src.dot(vec_dst) / (cv::norm(vec_src)*cv::norm(vec_dst)));
		if (theta > (PI - theta))
			theta = PI - theta;
	}
	else {
		std::cout << "ERROR: 输入抓取矩形不正确！" << std::endl;
		return false;
	}

	std::cout << "theta: " << theta << std::endl;

	double mat_[4] = { cos(theta), sin(theta), -sin(theta), cos(theta) };
	cv::Mat rotate_mat = cv::Mat(2, 2, CV_64FC1, mat_);

	double tmp = vec_dst.dot(rotate_mat*vec_src);
	if (tmp - 1 < 0.0000001 && tmp - 1 > -0.0000001) {

	}
	else {
		double mat_[4] = { cos(PI / 2 - theta), sin(PI / 2 - theta), -sin(PI / 2 - theta), cos(PI / 2 - theta) };
		rotate_mat = cv::Mat(2, 2, CV_64FC1, mat_);
	}

	rotate_ = rotate_mat.clone();
	return true;
}