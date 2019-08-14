#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <json/json.h>
#include <curl/curl.h>

/*
* @brief	�ýṹ���ڴ洢���������ؽ��
*/
struct MemoryStruct {
	char *memory;
	size_t size;
};


/*
* @brief	Ԥ��ץȡλ�õ���
*/
class Predict_Post
{
public:
	Predict_Post();
	~Predict_Post();

	/*
	* @brief	Ԥ��ץȡ����
	* @param	url		������IP�Ͷ˿�
	* @param	rgb_image_path	��ɫͼ·��
	* @param	depth_image_path	���ͼ·��
	* @param	box_points		��ץȡ�����Χ��(4*cv::Point)+����λ�ð�Χ��(4*cv::Point)
	* @return	url��ʼ��ʧ�ܷ��� 0������ʧ�ܷ��� -1�� �ɹ��������ݳ���
	*/
	static int start_predict(char *url, char *rgb_image_path, char *depth_image_path, std::vector<cv::Point>& box_points);

private:
	/*
	* @brief	json����
	* @param	strJsonContent		���������أ�string��
	* @param	vec_points			��ץȡ�����Χ��(4*cv::Point)+����λ�ð�Χ��(4*cv::Point)
	* @return	�����ɹ�����true�����򷵻�false
	*/
	static bool parseStrJson(std::string strJsonContent, std::vector<cv::Point>& vec_points);

	/*
	* @brief	�ص�����
	*/
	static size_t post_return(char *ptr, size_t size, size_t nmemb, void *stream);

};
