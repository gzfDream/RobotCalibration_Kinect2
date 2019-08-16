#include "stdafx.h"
#include "ImgProcess_TY.h"

ImgProcess_TY::ImgProcess_TY() {
	
}
ImgProcess_TY::~ImgProcess_TY() {
	
}

static void eventCallback(TY_EVENT_INFO *event_info, void *userdata)
{
	if (event_info->eventId == TY_EVENT_DEVICE_OFFLINE) {
		LOGD("=== Event Callback: Device Offline!");
		*(bool*)userdata = true;
	}
	else if (event_info->eventId == TY_EVENT_LICENSE_ERROR) {
		LOGD("=== Event Callback: License Error!");
	}
}


void ImgProcess_TY::getImage(std::string str, bool depth_or) {
	
	LOGD("=================================");
	LOGD("===== start capturing image =====");
	LOGD("=================================");

	std::string ID, IP;
	TY_INTERFACE_HANDLE hIface = NULL;
	TY_ISP_HANDLE hColorIspHandle = NULL;
	TY_DEV_HANDLE hDevice = NULL;
	
	LOGD("Init lib");
	ASSERT_OK(TYInitLib());
	TY_VERSION_INFO ver;
	ASSERT_OK(TYLibVersion(&ver));
	// LOGD("     - lib version: %d.%d.%d", ver.major, ver.minor, ver.patch);

	std::vector<TY_DEVICE_BASE_INFO> selected;
	ASSERT_OK(selectDevice(TY_INTERFACE_ALL, ID, IP, 1, selected));
	ASSERT(selected.size() > 0);
	TY_DEVICE_BASE_INFO& selectedDev = selected[0];

	ASSERT_OK(TYOpenInterface(selectedDev.iface.id, &hIface));
	ASSERT_OK(TYOpenDevice(hIface, selectedDev.id, &hDevice));

	int32_t allComps;
	ASSERT_OK(TYGetComponentIDs(hDevice, &allComps));

	///try to enable color camera
	if (allComps & TY_COMPONENT_RGB_CAM) {
		LOGD("Has RGB camera, open RGB cam");
		ASSERT_OK(TYEnableComponents(hDevice, TY_COMPONENT_RGB_CAM));
		//create a isp handle to convert raw image(color bayer format) to rgb image
		ASSERT_OK(TYISPCreate(&hColorIspHandle));
		//Init code can be modified in common.hpp
		//NOTE: Should set RGB image format & size before init ISP
		ASSERT_OK(ColorIspInitSetting(hColorIspHandle, hDevice));
		//You can  call follow function to show  color isp supported features
#if 0
		ColorIspShowSupportedFeatures(hColorIspHandle);
#endif
		//You can turn on auto exposure function as follow ,but frame rate may reduce .
		//Device may be casually stucked  1~2 seconds while software is trying to adjust device exposure time value
#if 0
		ASSERT_OK(ColorIspInitAutoExposure(hColorIspHandle, hDevice));
#endif
	}

	if (allComps & TY_COMPONENT_IR_CAM_LEFT) {
		LOGD("Has IR left camera, open IR left cam");
		ASSERT_OK(TYEnableComponents(hDevice, TY_COMPONENT_IR_CAM_LEFT));
	}

	if (allComps & TY_COMPONENT_IR_CAM_RIGHT) {
		LOGD("Has IR right camera, open IR right cam");
		ASSERT_OK(TYEnableComponents(hDevice, TY_COMPONENT_IR_CAM_RIGHT));
	}

	//try to enable depth map
	LOGD("Configure components, open depth cam");
	DepthViewer depthViewer("Depth");
	if (allComps & TY_COMPONENT_DEPTH_CAM) {
		std::vector<TY_ENUM_ENTRY> image_mode_list;
		ASSERT_OK(get_feature_enum_list(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, image_mode_list));
		for (int idx = 0; idx < image_mode_list.size(); idx++) {
			TY_ENUM_ENTRY &entry = image_mode_list[idx];
			//try to select a VGA resolution
			if (TYImageWidth(entry.value) == 1280 || TYImageHeight(entry.value) == 960) {
				LOGD("Select Depth Image Mode: %s", entry.description);
				int err = TYSetEnum(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, entry.value);
				ASSERT(err == TY_STATUS_OK || err == TY_STATUS_NOT_PERMITTED);
				break;
			}
		}
		ASSERT_OK(TYEnableComponents(hDevice, TY_COMPONENT_DEPTH_CAM));
		//depth map pixel format is uint16_t ,which default unit is  1 mm
		//the acutal depth (mm)= PxielValue * ScaleUnit 
		float scale_unit = 1.;
		TYGetFloat(hDevice, TY_COMPONENT_DEPTH_CAM, TY_FLOAT_SCALE_UNIT, &scale_unit);
		depthViewer.depth_scale_unit = scale_unit;
	}



	// LOGD("Prepare image buffer");
	uint32_t frameSize;
	ASSERT_OK(TYGetFrameBufferSize(hDevice, &frameSize));
	// LOGD("     - Get size of framebuffer, %d", frameSize);

	// LOGD("     - Allocate & enqueue buffers");
	char* frameBuffer[2];
	frameBuffer[0] = new char[frameSize];
	frameBuffer[1] = new char[frameSize];
	// LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[0], frameSize);
	ASSERT_OK(TYEnqueueBuffer(hDevice, frameBuffer[0], frameSize));
	// LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[1], frameSize);
	ASSERT_OK(TYEnqueueBuffer(hDevice, frameBuffer[1], frameSize));

	// LOGD("Register event callback");
	ASSERT_OK(TYRegisterEventCallback(hDevice, eventCallback, NULL));

	bool hasTrigger;
	ASSERT_OK(TYHasFeature(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &hasTrigger));
	if (hasTrigger) {
		// LOGD("Disable trigger mode");
		TY_TRIGGER_PARAM trigger;
		trigger.mode = TY_TRIGGER_MODE_OFF;
		ASSERT_OK(TYSetStruct(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &trigger, sizeof(trigger)));
	}

	LOGD("Start capture");
	ASSERT_OK(TYStartCapture(hDevice));

	bool saveFrame = false;
	int saveIdx = 0;
	cv::Mat depth;
	cv::Mat color;
	cv::moveWindow("color", 0, 0);

	LOGD("While loop to fetch frame");
	bool exit_main = false;
	TY_FRAME_DATA frame;
	while (!exit_main) {
		int err = TYFetchFrame(hDevice, &frame, -1);
		if (err == TY_STATUS_OK) {
			int fps = get_fps();

			cv::Mat depth, irl, irr, color;
			parseFrame(frame, &depth, &irl, &irr, &color, hColorIspHandle);
			if (!depth.empty()) {
				depthViewer.show(depth);
			}
			if (!color.empty()) { 
				if (fps > 0)
					cv::putText(color, "fps: ", cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(23, 23, 200), 1, 4);
				cv::putText(color, "press button 's' to get img ", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(23, 23, 200), 1, 4);
				cv::putText(color, "press button 'q/Esc' to get out", cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(23, 23, 200), 1, 4);

				cv::imshow("color", color);
			}

			if (saveFrame && !depth.empty() && !color.empty()) {
				if (saveIdx < 10) {
					if (depth_or)
						cv::imwrite(str + "depth_0" + std::to_string(saveIdx) + ".png", depth);
					cv::imwrite(str + "rgb_0" + std::to_string(saveIdx) + ".png", color);
				}
				else {
					if (depth_or)
						cv::imwrite(str + "depth_" + std::to_string(saveIdx) + ".png", depth);
					cv::imwrite(str + "rgb_" + std::to_string(saveIdx) + ".png", color);
				}

				LOGD("Get frame %d", saveIdx++);
				saveFrame = false;
			}

			int key = cv::waitKey(1);
			switch (key & 0xff) {
			case 0xff:
				break;
			case 's':
				saveFrame = true;
				break;
			case 'q':
				exit_main = true;
				break;
			case 27:
				exit_main = true;
				break;
			default:
				LOGD("Unmapped key %d", key);
			}

			TYISPUpdateDevice(hColorIspHandle);
			// LOGD("Re-enqueue buffer(%p, %d)", frame.userBuffer, frame.bufferSize);
			ASSERT_OK(TYEnqueueBuffer(hDevice, frame.userBuffer, frame.bufferSize));
		}
	}

	cv::destroyAllWindows();
	ASSERT_OK(TYStopCapture(hDevice));
	ASSERT_OK(TYCloseDevice(hDevice));
	ASSERT_OK(TYCloseInterface(hIface));
	ASSERT_OK(TYISPRelease(&hColorIspHandle));
	ASSERT_OK(TYDeinitLib());
	delete frameBuffer[0];
	delete frameBuffer[1];
}


bool ImgProcess_TY::getPixelDepth(cv::Point pixel, std::string depth_path, double &depth) {
	cv::Mat deoth_image = cv::imread(depth_path, CV_16U);
	if (deoth_image.type() != CV_16U || deoth_image.total() == 0) {
		return false;
	}
	double depth_scale_unit = 1.;
	depth = deoth_image.at<cv::uint16_t>(pixel.x, pixel.y)*depth_scale_unit;

	return true;
}


// 参考链接：https://blog.csdn.net/xholes/article/details/80599802
void ImgProcess_TY::ImgPixel2CameraPosition(const std::vector<cv::Point>& pixels, const Camera_Intrinsics& cam_in,
							                const double distCoeffD[5], std::vector<cv::Point2d>& camera_xy) {
	//相机内参
	/*
	Eigen::MatrixXd camera_ins = Eigen::MatrixXd::Zero(3, 4);
	camera_ins << cam_in.FLX, 0, cam_in.PPX, 0,
		0, cam_in.FLY, cam_in.PPY, 0,
		0, 0, 1, 0;
	std::cout << "camera_ins:" << camera_ins << std::endl;
	*/
	
	for (int i = 0; i < pixels.size(); ++i) {
		cv::Point2d p;
		p.x = (pixels[i].x - cam_in.PPX) / cam_in.FLX;
		p.y = (pixels[i].y - cam_in.PPY) / cam_in.FLY;

		double r_2 = p.x*p.x + p.y*p.y;
		cv::Point2d p_ = (1 + distCoeffD[0] * r_2 + distCoeffD[1] * r_2*r_2 + distCoeffD[2] * pow(r_2, 3)) * p
			+ cv::Point2d(2 * distCoeffD[3] * p.x*p.y + distCoeffD[4] * (r_2 + 2 * p.x*p.x),
				2 * distCoeffD[3] * (r_2 + 2 * p.y*p.y) + 2 * distCoeffD[4] * p.x *p.y);

		camera_xy.push_back(p_);
	}
	
}


bool align_rectangle(std::vector<cv::Point3d> src, std::vector<cv::Point3d> dst, cv::Mat& rotate_) {
	
	cv::Mat vec_src(src[1] - src[0]);
	cv::Mat vec_dst(dst[1] - dst[0]);

	double theta = 0.;

	if (cv::norm(vec_src) != 0 && cv::norm(vec_dst) != 0) {
		theta = acosf(vec_src.dot(vec_dst) / (cv::norm(vec_src)*cv::norm(vec_dst)));
		if (theta > (PI - theta))
			theta = PI - theta;
	}
	else {
		std::cout << "ERROR: 输入抓取矩形不正确！" << std::endl;
		return false;
	}

	std::cout << "theta: " << theta << std::endl;

	double mat_[4] = { cos(theta), sin(theta), -sin(theta), cos(theta) };
	cv::Mat rotate_mat = cv::Mat(2, 2, CV_64FC1, mat_);

	double tmp = vec_dst.dot(rotate_mat*vec_src);
	if (tmp - 1 < 0.0000001 && tmp - 1 > -0.0000001) {

	}
	else {
		double mat_[4] = { cos(PI / 2 - theta), sin(PI / 2 - theta), -sin(PI / 2 - theta), cos(PI / 2 - theta) };
		rotate_mat = cv::Mat(2, 2, CV_64FC1, mat_);
	}

	rotate_ = rotate_mat.clone();
	return true;
}
