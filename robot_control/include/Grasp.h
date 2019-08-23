#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#define PI 3.1415926

// 相机内参
typedef struct Camera_Intrinsics
{
	double FLX;
	double FLY;
	double PPX;
	double PPY;
	std::vector<double> distCoeffD;

	Camera_Intrinsics() {}

	Camera_Intrinsics(Camera_Intrinsics& cam) {
		*this = cam;
	}

	Camera_Intrinsics(double fx, double fy, double px, double py, std::vector<double> dist) :
		FLX(fx), FLY(fy), PPX(px), PPY(py), distCoeffD(dist) {}

	Camera_Intrinsics& operator=(const Camera_Intrinsics& cam) {
		if (this == &cam)
			return *this;

		this->FLX = cam.FLX;
		this->FLY = cam.FLY;
		this->PPX = cam.PPX;
		this->PPY = cam.PPY;
		this->distCoeffD = cam.distCoeffD;

		return *this;
	}
}Camera_Intrinsics;


class Grasp {
public:
	Grasp();
	Grasp(std::string url_, const Camera_Intrinsics& cam_in_);

	~Grasp();


public:

	/*
	*	@Method	getBoundingBox
	* 	@brief  	获得三维相机空间的bounding box位置
	* 	@param		std::string rgb_img		rgb图路径
	* 	@param		std::string depth_img	depth图路径
	* 	@param		const cv::Point& startPoint	向服务器传输的裁剪图片的裁剪起点
	* 	@param		std::vector<cv::Point3d> & box_points	三维相机空间的bounding box位置
	* 	@return	int	错误返回-1；正确返回bounding box点数量
	*/
	int getBoundingBox(std::string rgb_img, std::string depth_img, const cv::Point& startPoint, std::vector<cv::Point3d>& box_points);



	/*
	*	@Method	computeGraspPosition
	* 	@brief  	计算抓取的位置和旋转
	* 	@param		const std::vector<cv::Point3d> & box_points	三维相机空间的bounding box位置
	* 	@param		cv::Point3d& position	抓取位置
	* 	@param		cv::Vec3d& gnorm		抓取向量（抓手Z轴朝向）
	* 	@param		cv::Mat& rotate_mat		机械臂抓手旋转矩阵（相对于机械臂基座标）	
	* 	@return	void
	*/
	void computeGraspPosition(const std::vector<cv::Point3d>& box_points, cv::Point3d& position, cv::Vec3d& gnorm, cv::Mat& rotate_mat);
private:

	/*
	*	@Method	setUrl
	* 	@brief  	设置URL
	* 	@param		std::string url_
	* 	@return	void
	*/
	void setUrl(std::string url_) {
		url = url_;
	}

	/*
	*	@Method	setCamera_Intrinsics
	* 	@brief  	设置相机内参
	* 	@param		const Camera_Intrinsics& cam_in_
	* 	@return	void
	*/
	void setCamera_Intrinsics(const Camera_Intrinsics& cam_in_) {
		cam_in = cam_in_;
	}

	/*
	* @brief	得到深度图指定像素点的深度值
	* @param	pixels	像素点
	* @param	depth_path	深度图路径
	* @param	startPoint		向服务器传输的裁剪图片的裁剪起点
	* @param	depth	得到的深度值
	* @return	若成功获得则返回 true
	*/
	bool getPixelDepth(const std::vector<cv::Point>& pixels, std::string depth_path, const cv::Point& startPoint, std::vector<double> &depth);

	/*
	* @brief	像素点转到相机坐标系下的xy值
	* @param	pixels	像素点
	* @param	startPoint		向服务器传输的裁剪图片的裁剪起点
	* @param	camera_xy	相机坐标系下的xy值
	* @return		
	*/
	bool ImgPixel2CameraPosition(const std::vector<cv::Point>& pixels, const cv::Point& startPoint, std::vector<cv::Point2d>& camera_xy);

private:
	// url 地址
	std::string url = "http://192.168.3.39:5000/predict";

	// 相机内参和畸变
	Camera_Intrinsics cam_in;

};

//bool align_rectangle(std::vector<cv::Point2d> src, std::vector<cv::Point2d> dst, cv::Mat& rotate_) {
//	cv::Mat vec_src(src[1] - src[0]);
//	cv::Mat vec_dst(dst[1] - dst[0]);
//
//	double theta = 0.;
//
//	if (cv::norm(vec_src) != 0 && cv::norm(vec_dst) != 0) {
//		theta = acosf(vec_src.dot(vec_dst) / (cv::norm(vec_src)*cv::norm(vec_dst)));
//		if (theta > (PI - theta))
//			theta = PI - theta;
//	}
//	else {
//		std::cout << "ERROR: 输入抓取矩形不正确！" << std::endl;
//		return false;
//	}
//
//	std::cout << "theta: " << theta << std::endl;
//
//	double mat_[4] = { cos(theta), sin(theta), -sin(theta), cos(theta) };
//	cv::Mat rotate_mat = cv::Mat(2, 2, CV_64FC1, mat_);
//
//	double tmp = vec_dst.dot(rotate_mat*vec_src);
//	if (tmp - 1 < 0.0000001 && tmp - 1 > -0.0000001) {
//
//	}
//	else {
//		double mat_[4] = { cos(PI / 2 - theta), sin(PI / 2 - theta), -sin(PI / 2 - theta), cos(PI / 2 - theta) };
//		rotate_mat = cv::Mat(2, 2, CV_64FC1, mat_);
//	}
//
//	rotate_ = rotate_mat.clone();
//	return true;
//}