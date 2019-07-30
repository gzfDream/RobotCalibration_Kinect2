
#include "stdafx.h"

#include "CalChessboard.h"
#include <stdio.h>  
#include <io.h>  
#include <Windows.h> 



// �õ�·���µ������ļ������浽vector��
void getAllFiles(std::string path, std::vector<std::string>& files)
{
	long   hFile = 0;

	struct _finddata_t fileinfo;
	std::string p;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					//files.push_back(p.assign(path).append("\\").append(fileinfo.name) );
					getAllFiles(p.assign(path).append("\\").append(fileinfo.name), files);
				}
			}
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}


CalChessboard::CalChessboard(double &markerRealSize)
{
	m_markerCorners3d.push_back(cv::Point3f(0., 0., 0.));
	m_markerCorners3d.push_back(cv::Point3f(4. * markerRealSize, 0., 0.));
	m_markerCorners3d.push_back(cv::Point3f(8. * markerRealSize, 0., 0.));
	m_markerCorners3d.push_back(cv::Point3f(8. * markerRealSize, 5. * markerRealSize, 0.));
	m_markerCorners3d.push_back(cv::Point3f(4. * markerRealSize, 5. * markerRealSize, 0.));
	m_markerCorners3d.push_back(cv::Point3f(0., 5. * markerRealSize, 0.));
	m_markerCorners3d.push_back(cv::Point3f(2. * markerRealSize, 1. * markerRealSize, 0.));
	m_markerCorners3d.push_back(cv::Point3f(6. * markerRealSize, 1. * markerRealSize, 0.));
	m_markerCorners3d.push_back(cv::Point3f(6. * markerRealSize, 4. * markerRealSize, 0.));
	m_markerCorners3d.push_back(cv::Point3f(2. * markerRealSize, 4. * markerRealSize, 0.));
}


CalChessboard::~CalChessboard()
{
}


void CalChessboard::corner_detection(std::string &imgpath) {
	//cv::Mat img_color = cv::imread(imgpath, cv::IMREAD_COLOR);
	//��ת
	//flip(img_color, image_color, 1);

	image_color = cv::imread(imgpath, cv::IMREAD_COLOR);
	cv::Mat image_gray;
	cv::cvtColor(image_color, image_gray, cv::COLOR_BGR2GRAY);

	std::vector<cv::Point2f> corners, corner_t;

	bool ret = cv::findChessboardCorners(image_gray,
		cv::Size(6, 9),
		corners,
		cv::CALIB_CB_ADAPTIVE_THRESH |
		CV_CALIB_CB_FAST_CHECK |
		cv::CALIB_CB_NORMALIZE_IMAGE);

	//ָ�������ؼ��������ע  
	cv::TermCriteria criteria = cv::TermCriteria(
		cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS,
		40,
		0.1);

	//�����ؼ��  
	cv::cornerSubPix(image_gray, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);

	m_markerCorners2d.push_back(corners[0]);
	m_markerCorners2d.push_back(corners[24]);
	m_markerCorners2d.push_back(corners[48]);
	m_markerCorners2d.push_back(corners[53]);
	m_markerCorners2d.push_back(corners[29]);
	m_markerCorners2d.push_back(corners[5]);
	m_markerCorners2d.push_back(corners[13]);
	m_markerCorners2d.push_back(corners[37]);
	m_markerCorners2d.push_back(corners[40]);
	m_markerCorners2d.push_back(corners[16]);

}


// ����һ��ͼƬ��Ӧ�����
void CalChessboard::calibration(cv::Matx33f camera_matrix, cv::Matx<float, 5, 1> distortion_coefficients, cv::Mat &rvec, cv::Mat &tvec) {

	for (int i = 0; i < m_markerCorners2d.size(); i++)
	{
		cv::circle(image_color, m_markerCorners2d[i], 1, cv::Scalar(255, 40 * i, 255), 2);
	}
	cv::circle(image_color, m_markerCorners2d[0], 1, cv::Scalar(0, 0, 255), 2);

	cv::Mat objPM;
	cv::Mat(m_markerCorners3d).convertTo(objPM, CV_32F);

	//cv::solvePnPRansac(objPM, cv::Mat(m_markerCorners2d), camera_matrix, distortion_coefficients, rvec, tvec);
	cv::solvePnP(objPM, cv::Mat(m_markerCorners2d), camera_matrix, distortion_coefficients, rvec, tvec);

	cv::Mat objPM1;
	//m_markerCorners3d.push_back(cv::Point3f(22.2, -11.7, 6.4));
	cv::Mat(m_markerCorners3d).convertTo(objPM1, CV_32F);

	std::vector<cv::Point2f> projectedPoints;
	cv::projectPoints(objPM1, rvec, tvec, camera_matrix, distortion_coefficients, projectedPoints);

	for (unsigned int i = 0; i < projectedPoints.size(); ++i)
	{
		circle(image_color, projectedPoints[i], 2, cv::Scalar(255, 0, 0), -1, 8);
	}
	cv::imshow("chessboard corners", image_color);
	
	cv::moveWindow("chessboard corners", 0, 0);
	//cv::imshow("chessboard corners", image_color2);
	cv::waitKey(0);
	std::cout << "R:\n" << rvec << std::endl;
	std::cout << "T:\n" << tvec << std::endl;
}


cv::Mat CalChessboard::process_transMatrix(cv::Mat rvec, cv::Mat tvec) {
	cv::Mat trans_mat(4, 4, CV_32F), rotM;

	Rodrigues(rvec, rotM);

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			trans_mat.at<float>(i, j) = rotM.at<double>(i, j);
		}
	}
	trans_mat.at<float>(0, 3) = tvec.at<double>(0, 0) / 100;
	trans_mat.at<float>(1, 3) = tvec.at<double>(1, 0) / 100;
	trans_mat.at<float>(2, 3) = tvec.at<double>(2, 0) / 100;

	trans_mat.at<float>(3, 0) = 0;
	trans_mat.at<float>(3, 1) = 0;
	trans_mat.at<float>(3, 2) = 0;
	trans_mat.at<float>(3, 3) = 1;

	std::cout << "trans_mat:\n" << trans_mat << std::endl;

	//camHcal->calHcam
	//trans_mat = trans_mat.inv();

	return trans_mat;
}


//�������
cv::Mat CalChessboard::external_reference_calibration(double camD[9], double distCoeffD[5], std::string &imgpath, std::string &calibFile) {

	// ��������ڲ�
	cv::Matx33f camera_matrix;
	cv::Matx<float, 5, 1> distortion_coefficients; 
	camera_matrix = cv::Mat(3, 3, CV_64FC1, camD);
	distortion_coefficients = cv::Mat(5, 1, CV_64FC1, distCoeffD);

	std::vector<std::string> img_files;
	getAllFiles(imgpath, img_files);

	cv::Mat trans_mat(4, 4, CV_32F);

	cv::Mat rvec, tvec;
	std::ofstream ofsCalib(calibFile);
	for (int i = 0; i < img_files.size(); i++)
	{
		corner_detection(img_files[i]);
		calibration(camera_matrix, distortion_coefficients, rvec, tvec);
		trans_mat = process_transMatrix(rvec, tvec);
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++)
			{
				ofsCalib << trans_mat.at<float>(i, j) << " ";
			}
			ofsCalib << std::endl;
		}
		m_markerCorners2d.clear();
		image_color.release();
		rvec.release();
		tvec.release();
	}

	ofsCalib.close();
	return trans_mat;
}



// ��������ڲ�
void CalChessboard::internal_reference_calibration(std::vector<std::string> img_files, std::string internal_file) {
	//ifstream fin("calibdata.txt"); /* �궨����ͼ���ļ���·�� */
	std::ofstream fout(internal_file);  /* ����궨������ļ� */
													 //��ȡÿһ��ͼ�񣬴�����ȡ���ǵ㣬Ȼ��Խǵ���������ؾ�ȷ��	
	std::cout << "��ʼ��ȡ�ǵ㡭����������";
	int image_count = 0;  /* ͼ������ */
	cv::Size image_size;  /* ͼ��ĳߴ� */
	cv::Size board_size = cv::Size(9, 6);    /* �궨����ÿ�С��еĽǵ��� */
	std::vector<cv::Point2f> image_points_buf;  /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
	std::vector<std::vector<cv::Point2f>> image_points_seq; /* �����⵽�����нǵ� */

	int count = -1;//���ڴ洢�ǵ������
	for (int index = 0; index < img_files.size(); index++) {
		image_count++;
		// ���ڹ۲�������
		std::cout << "image_count = " << image_count << std::endl;
		/* �������*/
		std::cout << "-->count = " << count;
		cv::Mat imageInput = cv::imread(img_files[index]);
		if (image_count == 1)  //�����һ��ͼƬʱ��ȡͼ������Ϣ
		{
			image_size.width = imageInput.cols;
			image_size.height = imageInput.rows;
			std::cout << "image_size.width = " << image_size.width << std::endl;
			std::cout << "image_size.height = " << image_size.height << std::endl;
		}

		/* ��ȡ�ǵ� */
		if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
		{
			std::cout << "can not find chessboard corners!\n"; //�Ҳ����ǵ�
			exit(1);
		}
		else
		{
			cv::Mat view_gray;
			cvtColor(imageInput, view_gray, CV_RGB2GRAY);
			/* �����ؾ�ȷ�� */
			find4QuadCornerSubpix(view_gray, image_points_buf, cv::Size(5, 5)); //�Դ���ȡ�Ľǵ���о�ȷ��
																			//cornerSubPix(view_gray,image_points_buf,Size(5,5),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
			image_points_seq.push_back(image_points_buf);  //���������ؽǵ�
														   /* ��ͼ������ʾ�ǵ�λ�� */
			drawChessboardCorners(view_gray, board_size, image_points_buf, false); //������ͼƬ�б�ǽǵ�
			imshow("Camera Calibration", view_gray);//��ʾͼƬ
			cv::waitKey(500);//��ͣ0.5S		
		}
	}

	int total = image_points_seq.size();
	std::cout << "total = " << total << std::endl;
	int CornerNum = board_size.width*board_size.height;  //ÿ��ͼƬ���ܵĽǵ���
	for (int ii = 0; ii < total; ii++)
	{
		if (0 == ii % CornerNum)// 24 ��ÿ��ͼƬ�Ľǵ���������ж������Ϊ����� ͼƬ�ţ����ڿ���̨�ۿ� 
		{
			int i = -1;
			i = ii / CornerNum;
			int j = i + 1;
			std::cout << "--> �� " << j << "ͼƬ������ --> : " << std::endl;
		}
		if (0 == ii % 3)	// ���ж���䣬��ʽ����������ڿ���̨�鿴
		{
			std::cout << std::endl;
		}
		else
		{
			std::cout.width(10);
		}
		//������еĽǵ�
		std::cout << " -->" << image_points_seq[ii][0].x;
		std::cout << " -->" << image_points_seq[ii][0].y;
	}
	std::cout << "�ǵ���ȡ��ɣ�\n";

	//������������궨
	std::cout << "��ʼ�궨������������";
	/*������ά��Ϣ*/
	cv::Size square_size = cv::Size(3, 3);  /* ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С */
	std::vector<std::vector<cv::Point3f>> object_points; /* ����궨���Ͻǵ����ά���� */
										   /*�������*/
	cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* ������ڲ������� */
	std::vector<int> point_counts;  // ÿ��ͼ���нǵ������
	cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); /* �������5������ϵ����k1,k2,p1,p2,k3 */
	std::vector<cv::Mat> tvecsMat;  /* ÿ��ͼ�����ת���� */
	std::vector<cv::Mat> rvecsMat; /* ÿ��ͼ���ƽ������ */
						  /* ��ʼ���궨���Ͻǵ����ά���� */
	int i, j, t;
	for (t = 0; t < image_count; t++)
	{
		std::vector<cv::Point3f> tempPointSet;
		for (i = 0; i < board_size.height; i++)
		{
			for (j = 0; j < board_size.width; j++)
			{
				cv::Point3f realPoint;
				/* ����궨�������������ϵ��z=0��ƽ���� */
				realPoint.x = i * square_size.width;
				realPoint.y = j * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}
	/* ��ʼ��ÿ��ͼ���еĽǵ��������ٶ�ÿ��ͼ���ж����Կ��������ı궨�� */
	for (i = 0; i < image_count; i++)
	{
		point_counts.push_back(board_size.width*board_size.height);
	}
	/* ��ʼ�궨 */
	calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	std::cout << "�궨��ɣ�\n";
	cv::destroyAllWindows();

	//�Ա궨�����������
	std::cout << "��ʼ���۱궨���������������\n";
	double total_err = 0.0; /* ����ͼ���ƽ�������ܺ� */
	double err = 0.0; /* ÿ��ͼ���ƽ����� */
	std::vector<cv::Point2f> image_points2; /* �������¼���õ���ͶӰ�� */
	std::cout << "\tÿ��ͼ��ı궨��\n";
	fout << "ÿ��ͼ��ı궨��\n";
	for (i = 0; i < image_count; i++)
	{
		std::vector<cv::Point3f> tempPointSet = object_points[i];
		/* ͨ���õ������������������Կռ����ά���������ͶӰ���㣬�õ��µ�ͶӰ�� */
		cv::projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, image_points2);
		/* �����µ�ͶӰ��;ɵ�ͶӰ��֮������*/
		std::vector<cv::Point2f> tempImagePoint = image_points_seq[i];
		cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
		cv::Mat image_points2Mat = cv::Mat(1, image_points2.size(), CV_32FC2);
		for (int j = 0; j < tempImagePoint.size(); j++)
		{
			image_points2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(image_points2[j].x, image_points2[j].y);
			tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
		}
		err = cv::norm(image_points2Mat, tempImagePointMat, cv::NORM_L2);
		total_err += err /= point_counts[i];
		std::cout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << std::endl;
		fout << "��" << i + 1 << "��ͼ���ƽ����" << err << "����" << std::endl;
	}
	std::cout << "����ƽ����" << total_err / image_count << "����" << std::endl;
	fout << "����ƽ����" << total_err / image_count << "����" << std::endl << std::endl;
	std::cout << "������ɣ�" << std::endl;
	//���涨����  	
	std::cout << "��ʼ���涨����������������" << std::endl;
	cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* ����ÿ��ͼ�����ת���� */
	fout << "����ڲ�������" << std::endl;
	fout << cameraMatrix << std::endl << std::endl;
	fout << "����ϵ����\n";
	fout << distCoeffs << std::endl << std::endl << std::endl;
	for (int i = 0; i < image_count; i++)
	{
		fout << "��" << i + 1 << "��ͼ�����ת������" << std::endl;
		fout << tvecsMat[i] << std::endl;
		/* ����ת����ת��Ϊ���Ӧ����ת���� */
		cv::Rodrigues(tvecsMat[i], rotation_matrix);
		fout << "��" << i + 1 << "��ͼ�����ת����" << std::endl;
		fout << rotation_matrix << std::endl;
		fout << "��" << i + 1 << "��ͼ���ƽ��������" << std::endl;
		fout << rvecsMat[i] << std::endl << std::endl;
	}
	std::cout << "��ɱ���" << std::endl;
	fout << std::endl;
	/************************************************************************
	��ʾ������
	*************************************************************************/
	cv::Mat mapx = cv::Mat(image_size, CV_32FC1);
	cv::Mat mapy = cv::Mat(image_size, CV_32FC1);
	cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
	std::cout << "�������" << std::endl;
}
