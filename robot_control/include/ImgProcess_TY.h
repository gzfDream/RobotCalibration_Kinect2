#pragma once
#include "common_TY/common.hpp"
#include "predict_post.h"

class ImgProcess_TY {
public:

	ImgProcess_TY();
	~ImgProcess_TY();

public:
	/*
	* @brief	������أ��ɼ�ͼƬ����Ԥ��ץȡ����
	* @param	url			��������ַ
	* @param	img_path	�洢·��
	*/
	void getImage(std::string url, std::string img_path);

};