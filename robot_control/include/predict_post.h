#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <json/json.h>
#include <curl/curl.h>

/*
* @brief	该结构用于存储服务器返回结果
*/
struct MemoryStruct {
	char *memory;
	size_t size;
};


/*
* @brief	预测抓取位置的类
*/
class Predict_Post
{
public:
	Predict_Post();
	~Predict_Post();

	/*
	* @brief	预测抓取区域
	* @param	url		服务器IP和端口
	* @param	rgb_image_path	颜色图路径
	* @param	depth_image_path	深度图路径
	* @param	box_points		待抓取物体包围盒(4*cv::Point)+吸盘位置包围盒(4*cv::Point)
	* @return	url初始化失败返回 0；连接失败返回 -1； 成功返回数据长度
	*/
	static int start_predict(char *url, char *rgb_image_path, char *depth_image_path, std::vector<cv::Point>& box_points);

private:
	/*
	* @brief	json解析
	* @param	strJsonContent		服务器返回（string）
	* @param	vec_points			待抓取物体包围盒(4*cv::Point)+吸盘位置包围盒(4*cv::Point)
	* @return	解析成功返回true，否则返回false
	*/
	static bool parseStrJson(std::string strJsonContent, std::vector<cv::Point>& vec_points);

	/*
	* @brief	回调函数
	*/
	static size_t post_return(char *ptr, size_t size, size_t nmemb, void *stream);

};
