#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <json/json.h>
#include <curl/curl.h>

static void parseStrJson(std::string strJsonContent) {
	Json::Reader reader;
	Json::Value root;

	std::vector<cv::Point> vec_points;

	std::string state = "";
	if (reader.parse(strJsonContent, root))
	{
		cv::Point p0;
		p0.x = std::stoi(root["p0_x"].asString());
		p0.y = std::stoi(root["p0_y"].asString());
		std::cout << p0 << std::endl;

		p0.x = std::stoi(root["p1_x"].asString());
		p0.y = std::stoi(root["p1_y"].asString());
		std::cout << p0 << std::endl;

		p0.x = std::stoi(root["p2_x"].asString());
		p0.y = std::stoi(root["p2_y"].asString());
		std::cout << p0 << std::endl;

		p0.x = std::stoi(root["p3_x"].asString());
		p0.y = std::stoi(root["p3_y"].asString());
		std::cout << p0 << std::endl;

		state = root["state"].asString();
	}


	std::cout << "state is: " << state << std::endl;
}

static size_t post_return(char *ptr, size_t size, size_t nmemb, void *stream)
{
	std::string strJsonContent = ptr;
	parseStrJson(strJsonContent);
	return nmemb;
}

static int start_predict(char *url, char *rgb_image_path, char *depth_image_path)
{
	CURL *pCurl = NULL;
	CURLcode res;

	struct curl_slist *headerlist = NULL;
	struct curl_httppost *post = NULL;
	struct curl_httppost *last = NULL;
	curl_formadd(&post, &last, CURLFORM_COPYNAME, "rgb_image",
		CURLFORM_FILE, rgb_image_path,
		CURLFORM_END);

	curl_formadd(&post, &last, CURLFORM_COPYNAME, "depth_image",
		CURLFORM_FILE, depth_image_path,
		CURLFORM_END);

	pCurl = curl_easy_init();

	if (NULL != pCurl)
	{
		curl_easy_setopt(pCurl, CURLOPT_URL, url);
		curl_easy_setopt(pCurl, CURLOPT_HTTPPOST, post);
		curl_easy_setopt(pCurl, CURLOPT_WRITEFUNCTION, post_return);

		res = curl_easy_perform(pCurl);
		if (res != CURLE_OK)
		{
			printf("curl_easy_perform() failed£¬error code is:%s\n", curl_easy_strerror(res));
		}

		curl_easy_cleanup(pCurl);
	}
	return 1;
}
