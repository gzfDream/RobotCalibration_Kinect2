#include <iostream>
#include "ImgProcess_TY.h"
#include "predict_post.h"
#include "Grasp.h"
#include <Eigen/Dense>


//int main() {
//
//	std::vector<cv::Point2d> src = { cv::Point2d(0.,0.), cv::Point2d(0.,200.), cv::Point2d(100.,200.), cv::Point2d(100.,0.) };
//	/*std::vector<cv::Point2d> dst = { cv::Point2d(499.,261.), cv::Point2d(672.,184.),
//									 cv::Point2d(732.,320.), cv::Point2d(558.,396.) };*/
//	std::vector<cv::Point2d> dst = { cv::Point2d(325,206), cv::Point2d(438,166), cv::Point2d(486,303), cv::Point2d(373,343) };
//	//std::vector<cv::Point2d> dst = { cv::Point2d(357,643), cv::Point2d(378,5601), cv::Point2d(511,595), cv::Point2d(490,677) };
//
//	cv::Mat m;
//	align_rectangle(src, dst, m);
//	std::cout << m << std::endl;
//
//	cv::Mat rgb = cv::imread("rgb.jpg");
//
//	cv::namedWindow("rgb image");
//
//	cv::line(rgb, dst[0], dst[1], cv::Scalar(255, 0, 0), 4);
//	cv::line(rgb, dst[1], dst[2], cv::Scalar(255, 0, 0), 4);
//	cv::line(rgb, dst[2], dst[3], cv::Scalar(255, 0, 0), 4);
//	cv::line(rgb, dst[3], dst[0], cv::Scalar(255, 0, 0), 4);
//	
//	cv::Mat m0 = m * cv::Mat(src[0]) + cv::Mat(dst[0] - src[0]);
//	cv::circle(rgb, cv::Point(m0), 5, cv::Scalar(255, 255, 255));
//	cv::Mat m1 = m * cv::Mat(src[1]) + cv::Mat(dst[0] - src[0]);
//	cv::Mat m2 = m * cv::Mat(src[2]) + cv::Mat(dst[0] - src[0]);
//	cv::Mat m3 = m * cv::Mat(src[3]) + cv::Mat(dst[0] - src[0]);
//
//	cv::line(rgb, cv::Point(m0), cv::Point(m1), cv::Scalar(0, 0, 220), 4);
//	cv::line(rgb, cv::Point(m1), cv::Point(m2), cv::Scalar(0, 0, 220), 4);
//	cv::line(rgb, cv::Point(m2), cv::Point(m3), cv::Scalar(0, 0, 220), 4);
//	cv::line(rgb, cv::Point(m3), cv::Point(m0), cv::Scalar(0, 0, 220), 4);
//
//	cv::imshow("rgb image", rgb);
//	cv::waitKey(1);
//
//	system("pause");
//	return 0;
//}


cv::Point cameraXYZ2pixel(cv::Point3d p, Camera_Intrinsics cam) {
	Eigen::Matrix3d m;
	m << cam.FLX, 0, cam.PPX,
		 0, cam.FLY, cam.PPY,
		 0, 0, 1;

	Eigen::Vector3d v(p.x, p.y, p.z);

	cv::Point pixel;
	Eigen::Vector3d res = m * v;
	pixel.x = res.x() / res.z();
	pixel.y = res.y() / res.z();

	return pixel;
}

void clip_img(std::string path, int x_begin, int y_begin, int width, int height) {

	IplImage* src = cvLoadImage(path.c_str(), 1);
	if (!src) {
		std::cout << " image read error!" << std::endl;
		cvReleaseImage(&src);
		return;
	}
	int srcWidth = src->width;	//获取原图宽、高
	int srcHeight = src->height;

	//控制裁取区域不超过原图
	if (width < 1 || height < 1 || width > srcWidth || height > srcHeight) {
		std::cout << "[Rect error: srcWidth = " << srcWidth << ", srcHeight = " << srcHeight << ", x_begin = "
			<< x_begin << ", y_begin = " << y_begin << ", width = " << width << ", height = " << height << " ]";
		cvReleaseImage(&src);
		return;
	}

	if (x_begin + width > srcWidth)
		width = srcWidth - x_begin;
	if (y_begin + height > srcHeight)
		height = srcHeight - y_begin;

	//区域裁取
	IplImage* image_src = cvCloneImage(src);	//备份原图
	cvSetImageROI(image_src, cvRect(x_begin, y_begin, width, height));	//设置待裁取ROI
	IplImage* dst = cvCreateImage(cvSize(width, height), src->depth, src->nChannels);	//创建裁取区域大小的IplImage*
	cvCopy(image_src, dst);		//将ROI区域拷贝至dst

	uint16_t* pixel = (uint16_t*)(src->imageData + (220+145) * src->widthStep + 263);
	std::cout << "pixel=" << (*pixel) + 0 << std::endl;//+0隐式转换为整型，否则会打印出字符

	pixel = (uint16_t*)(dst->imageData +  220 * dst->widthStep + 263);
	std::cout << "pixel=" << (*pixel) + 0 << std::endl;//+0隐式转换为整型，否则会打印出字符

	cvShowImage("", dst);
	cvWaitKey(0);
	char p[] = "./data/test.png";

	//释放资源
	cvReleaseImage(&src);
	cvResetImageROI(image_src);
	cvReleaseImage(&image_src);
}

int main()
{
	/*int back_code = 0;
	char url_[] = "http://192.168.3.39:5000/predict";
	char rgb[] = "./data/rgb.jpg";
	char depth[] = "./data/depth.png";
	std::vector<cv::Point> ps;
	back_code =Predict_Post::start_predict(url_, rgb, depth, ps);*/

	int n = 11;

	std::string rgb_ = "./data/00" + std::to_string(n) + ".jpg";
	std::string depth_ = "./data/00" + std::to_string(n) + ".png";

	// clip_img(depth_, 145, 0, 900, 960);
	//cv::Mat color_m = cv::imread(rgb_);
	// cv::Mat depth_m = cv::imread(depth_, CV_16U);
	// std::cout << depth_m.at<uint16_t>(609+145, 263) << std::endl;
	
	// cv::Rect io_select = cv::Rect(145, 00, 900, 960);

	// cv::Mat io_rgb = color_m(io_select);
	// cv::Mat io_depth = depth_m(io_select);
	
	// cv::convertScaleAbs(io_depth, io_depth, pow(2,8)/(2,16), 0);
	// std::cout << io_depth.at<uint16_t>(609, 263) << std::endl;

	//std::string rgb_path = "./data/00" + std::to_string(n) + "_clip.jpg";
	//std::string depth_path = "./data/00" + std::to_string(n) + "_clip.png";
	//cv::imwrite(rgb_path, io_rgb);
	//cv::imwrite(depth_path, io_depth);

	std::string rgb = "./data/00" + std::to_string(n) + "_clip.jpg";
	std::string depth = "./data/00" + std::to_string(n) + "_clip.png";
	std::string url = "http://192.168.3.39:5000/predict";

	std::vector<double>dis = { -0.2891808654148305, 0.2203800962619925, -0.0001150963076652628, -0.0004038443881143156, -0.1637493969089135 };
	Camera_Intrinsics cam(1164.8, 1164.38, 614.1, 455.94, dis);

	Grasp graspObject(url, cam);
	std::vector<cv::Point3d> box;
	cv::Point3d position;
	cv::Vec3d gnorm;
	cv::Mat rotate_mat;
	if (graspObject.getBoundingBox(rgb, depth, cv::Point(145, 0), box)>0) {
		graspObject.computeGraspPosition(box, position, gnorm, rotate_mat);

		for (int i = 0; i < box.size(); ++i) {
			std::cout << box[i] << std::endl;
		}
		std::cout << std::endl;
		std::cout << position << std::endl;
		std::cout << gnorm << std::endl;
		std::cout << rotate_mat << std::endl;
	}
	cv::Mat rgb_img = cv::imread(rgb_);


	cv::Point pp = cameraXYZ2pixel(box[4], cam);
	cv::circle(rgb_img, pp, 3, cv::Scalar(0, 255, 255), 2);

	pp = cameraXYZ2pixel(box[5], cam);
	cv::circle(rgb_img, pp, 3, cv::Scalar(0, 255, 255), 2);

	pp = cameraXYZ2pixel(box[6], cam);
	cv::circle(rgb_img, pp, 3, cv::Scalar(0, 255, 255), 2);

	pp = cameraXYZ2pixel(box[7], cam);
	cv::circle(rgb_img, pp, 3, cv::Scalar(0, 255, 255), 2);

	pp = cameraXYZ2pixel(position, cam);
	cv::circle(rgb_img, pp, 3, cv::Scalar(0, 0, 255), 2);

	cv::imshow("", rgb_img);

	cv::waitKey(0);
	system("pause");
	return 0;
}