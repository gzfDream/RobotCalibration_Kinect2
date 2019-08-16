#include <iostream>
#include "ImgProcess_TY.h"
#include "predict_post.h"
#include "Grasp.h"



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




int main()
{
	/*int back_code = 0;
	char url_[] = "http://192.168.3.39:5000/predict";
	char rgb[] = "./data/rgb.jpg";
	char depth[] = "./data/depth.png";
	std::vector<cv::Point> ps;
	back_code =Predict_Post::start_predict(url_, rgb, depth, ps);*/


	std::vector<double>dis = { -0.263800155103853, 0.145978275196663, -0.0808361022675977, -0.000373220970137041,	0.000490335127893494 };
	Camera_Intrinsics cam(1154.6, 1155.5, 613.4968, 449.4059, dis);

	ImgProcess_TY img_TY(cam);
	img_TY.getImage("", "");

	system("pause");
	return 0;
}