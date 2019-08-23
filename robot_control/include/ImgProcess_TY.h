#pragma once
#include "common_TY/common.hpp"
#include "predict_post.h"

class ImgProcess_TY {
public:

	ImgProcess_TY();
	~ImgProcess_TY();

public:
	/*
	* @brief	按键监控，采集图片，并预测抓取区域
	* @param	url			服务器地址
	* @param	img_path	存储路径
	*/
	void getImage(std::string url, std::string img_path);

};