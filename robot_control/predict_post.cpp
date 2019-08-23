#include "predict_post.h"


Predict_Post::Predict_Post()
{
}

Predict_Post::~Predict_Post()
{
}


bool Predict_Post::parseStrJson(std::string strJsonContent, std::pair<std::vector<cv::Point>, std::vector<cv::Point>>& vec_points) {
	Json::Reader reader;
	Json::Value root;
	bool show_not = true;

	bool state = false;
	if (reader.parse(strJsonContent, root))
	{
		cv::Point p0;

		//////////////////////////////////////////////////////////////////////////
		// 物体表面点
		p0.x = std::stoi(root["ori_p0_x"].asString());
		p0.y = std::stoi(root["ori_p0_y"].asString());
		vec_points.second.push_back(p0);

		p0.x = std::stoi(root["ori_p1_x"].asString());
		p0.y = std::stoi(root["ori_p1_y"].asString());
		vec_points.second.push_back(p0);

		p0.x = std::stoi(root["ori_p2_x"].asString());
		p0.y = std::stoi(root["ori_p2_y"].asString());
		vec_points.second.push_back(p0);

		p0.x = std::stoi(root["ori_p3_x"].asString());
		p0.y = std::stoi(root["ori_p3_y"].asString());
		vec_points.second.push_back(p0);

		//////////////////////////////////////////////////////////////////////////

		p0.x = std::stoi(root["pre_p0_x"].asString());
		p0.y = std::stoi(root["pre_p0_y"].asString());
		vec_points.first.push_back(p0);

		p0.x = std::stoi(root["pre_p1_x"].asString());
		p0.y = std::stoi(root["pre_p1_y"].asString());
		vec_points.first.push_back(p0);

		p0.x = std::stoi(root["pre_p2_x"].asString());
		p0.y = std::stoi(root["pre_p2_y"].asString());
		vec_points.first.push_back(p0);

		p0.x = std::stoi(root["pre_p3_x"].asString());
		p0.y = std::stoi(root["pre_p3_y"].asString());
		vec_points.first.push_back(p0);

		state = root["state"].asBool();
		if (show_not) {
			for (int i = 0; i < vec_points.first.size(); ++i) {
				std::cout << vec_points.first[i] << std::endl;
			}
			for (int i = 0; i < vec_points.second.size(); ++i) {
				std::cout << vec_points.second[i] << std::endl;
			}

			std::cout << "state is: " << state << std::endl;
		}	
	}
	return state;
}


size_t Predict_Post::post_return(char *contents, size_t size, size_t nmemb, void *userp)
{
	/*std::string strJsonContent = contents;
	std::vector<cv::Point> box_points;
	parseStrJson(strJsonContent, box_points);
	return nmemb;*/
	
	size_t realsize = size * nmemb;
	struct MemoryStruct *mem = (struct MemoryStruct *)userp;

	char *ptr = (char*)realloc(mem->memory, mem->size + realsize + 1);
	if (ptr == NULL) {
		/* out of memory! */
		printf("not enough memory (realloc returned NULL)\n");
		return 0;
	}

	mem->memory = ptr;
	memcpy(&(mem->memory[mem->size]), contents, realsize);
	mem->size += realsize;
	mem->memory[mem->size] = 0;

	return realsize;

}


int Predict_Post::start_predict(char *url, char *rgb_image_path, char *depth_image_path, std::pair<std::vector<cv::Point>, std::vector<cv::Point>>& box_points)
{
	MemoryStruct chunk;
	chunk.memory = (char*)malloc(1);  /* will be grown as needed by the realloc above */
	chunk.size = 0;    /* no data at this point */

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
		curl_easy_setopt(pCurl, CURLOPT_WRITEDATA, (void *)&chunk);

		res = curl_easy_perform(pCurl);
		if (res != CURLE_OK)
		{
			printf("curl_easy_perform() failed，error code is:%s\n", curl_easy_strerror(res));
			curl_easy_cleanup(pCurl);
			return -1;
		}
		else {
			// std::cout << chunk.memory << std::endl;
			std::string strJsonContent = chunk.memory;

			if (!parseStrJson(strJsonContent, box_points))
				std::cout << "json parse error!" << std::endl;

			curl_easy_cleanup(pCurl);
			return chunk.size;
		}
	}
	return 0;
}

