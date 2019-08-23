#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#define PI 3.1415926

// ����ڲ�
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
	* 	@brief  	�����ά����ռ��bounding boxλ��
	* 	@param		std::string rgb_img		rgbͼ·��
	* 	@param		std::string depth_img	depthͼ·��
	* 	@param		const cv::Point& startPoint	�����������Ĳü�ͼƬ�Ĳü����
	* 	@param		std::vector<cv::Point3d> & box_points	��ά����ռ��bounding boxλ��
	* 	@return	int	���󷵻�-1����ȷ����bounding box������
	*/
	int getBoundingBox(std::string rgb_img, std::string depth_img, const cv::Point& startPoint, std::vector<cv::Point3d>& box_points);



	/*
	*	@Method	computeGraspPosition
	* 	@brief  	����ץȡ��λ�ú���ת
	* 	@param		const std::vector<cv::Point3d> & box_points	��ά����ռ��bounding boxλ��
	* 	@param		cv::Point3d& position	ץȡλ��
	* 	@param		cv::Vec3d& gnorm		ץȡ������ץ��Z�ᳯ��
	* 	@param		cv::Mat& rotate_mat		��е��ץ����ת��������ڻ�е�ۻ����꣩	
	* 	@return	void
	*/
	void computeGraspPosition(const std::vector<cv::Point3d>& box_points, cv::Point3d& position, cv::Vec3d& gnorm, cv::Mat& rotate_mat);
private:

	/*
	*	@Method	setUrl
	* 	@brief  	����URL
	* 	@param		std::string url_
	* 	@return	void
	*/
	void setUrl(std::string url_) {
		url = url_;
	}

	/*
	*	@Method	setCamera_Intrinsics
	* 	@brief  	��������ڲ�
	* 	@param		const Camera_Intrinsics& cam_in_
	* 	@return	void
	*/
	void setCamera_Intrinsics(const Camera_Intrinsics& cam_in_) {
		cam_in = cam_in_;
	}

	/*
	* @brief	�õ����ͼָ�����ص�����ֵ
	* @param	pixels	���ص�
	* @param	depth_path	���ͼ·��
	* @param	startPoint		�����������Ĳü�ͼƬ�Ĳü����
	* @param	depth	�õ������ֵ
	* @return	���ɹ�����򷵻� true
	*/
	bool getPixelDepth(const std::vector<cv::Point>& pixels, std::string depth_path, const cv::Point& startPoint, std::vector<double> &depth);

	/*
	* @brief	���ص�ת���������ϵ�µ�xyֵ
	* @param	pixels	���ص�
	* @param	startPoint		�����������Ĳü�ͼƬ�Ĳü����
	* @param	camera_xy	�������ϵ�µ�xyֵ
	* @return		
	*/
	bool ImgPixel2CameraPosition(const std::vector<cv::Point>& pixels, const cv::Point& startPoint, std::vector<cv::Point2d>& camera_xy);

private:
	// url ��ַ
	std::string url = "http://192.168.3.39:5000/predict";

	// ����ڲκͻ���
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
//		std::cout << "ERROR: ����ץȡ���β���ȷ��" << std::endl;
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