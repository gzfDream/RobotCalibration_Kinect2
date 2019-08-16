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

	LOGD("=== Init lib");
	int loop_index = 1;
	bool loop_exit = false;

	ASSERT_OK(TYInitLib());
	TY_VERSION_INFO ver;
	ASSERT_OK(TYLibVersion(&ver));
	LOGD("     - lib version: %d.%d.%d", ver.major, ver.minor, ver.patch);

	// 打开设备
	std::string ID, IP;
	TY_INTERFACE_HANDLE hIface;
	TY_DEV_HANDLE hDevice;
	std::vector<TY_DEVICE_BASE_INFO> selected;

	int ret = 0;
	ASSERT_OK(selectDevice(TY_INTERFACE_ALL, ID, IP, 1, selected));
	/*ret = selectDevice(TY_INTERFACE_ALL, ID, IP, 1, selected);
	if (ret < 0) {
		MSleep(2000);
		continue;
	}*/

	ASSERT(selected.size() > 0);
	TY_DEVICE_BASE_INFO& selectedDev = selected[0];

	LOGD("=== Open interface: %s", selectedDev.iface.id);
	ASSERT_OK(TYOpenInterface(selectedDev.iface.id, &hIface));

	LOGD("=== Open device: %s", selectedDev.id);
	ret = TYOpenDevice(hIface, selectedDev.id, &hDevice);
	if (ret < 0) {
		LOGD("Failed!");
		ASSERT_OK(TYCloseInterface(hIface));
		return;
	}

	// 查询设备支持的组件
	int32_t allComps;
	ret = TYGetComponentIDs(hDevice, &allComps);
	if (ret < 0) {
		LOGD("Failed!");
		ASSERT_OK(TYCloseDevice(hDevice));
		ASSERT_OK(TYCloseInterface(hIface));
		return;
	}

	// 打开RGB相机
	if (allComps & TY_COMPONENT_RGB_CAM) {
		LOGD("=== Has RGB camera, open RGB cam");
		ret = TYEnableComponents(hDevice, TY_COMPONENT_RGB_CAM);
		if (ret < 0) {
			LOGD("Failed!");
			ASSERT_OK(TYCloseDevice(hDevice));
			ASSERT_OK(TYCloseInterface(hIface));
			return;
		}
	}

	// 打开depth相机
	LOGD("=== Configure components, open depth cam");
	int32_t componentIDs = TY_COMPONENT_DEPTH_CAM;
	ret = TYEnableComponents(hDevice, componentIDs);
	if (ret < 0) {
		LOGD("Failed!");
		ASSERT_OK(TYCloseDevice(hDevice));
		ASSERT_OK(TYCloseInterface(hIface));
		return;
	}

	// 图像的格式和分辨率
	LOGD("=== Configure feature, set resolution.");
	ret = TYSetEnum(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, TY_IMAGE_MODE_DEPTH16_1280x960);//TY_IMAGE_MODE_DEPTH16_640x480
	if (ret < 0) {
		LOGD("Failed!");
		ASSERT_OK(TYCloseDevice(hDevice));
		ASSERT_OK(TYCloseInterface(hIface));
		return;
	}

	// 获取当前设备配置需要的帧缓冲空间大小。同一相机设备，工作在不同图像数据输出模式下，需要的帧缓冲区大小不同。
	LOGD("=== Prepare image buffer");
	uint32_t frameSize;
	ret = TYGetFrameBufferSize(hDevice, &frameSize);
	if (ret < 0) {
		LOGD("Failed!");
		ASSERT_OK(TYCloseDevice(hDevice));
		ASSERT_OK(TYCloseInterface(hIface));
		return;
	}

	// 把分配的帧缓冲推入缓冲队列。
	LOGD("     - Allocate & enqueue buffers");
	char* frameBuffer[2];
	frameBuffer[0] = new char[frameSize];
	frameBuffer[1] = new char[frameSize];
	LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[0], frameSize);
	ASSERT_OK(TYEnqueueBuffer(hDevice, frameBuffer[0], frameSize));
	LOGD("     - Enqueue buffer (%p, %d)", frameBuffer[1], frameSize);
	ASSERT_OK(TYEnqueueBuffer(hDevice, frameBuffer[1], frameSize));

	bool device_offline = false;;
	LOGD("=== Register event callback");
	LOGD("Note: Callback may block internal data receiving,");
	LOGD("      so that user should not do long time work in callback.");
	ASSERT_OK(TYRegisterEventCallback(hDevice, eventCallback, &device_offline));

	//  配置相机设备工作在模式0，连续输出图像数据。
	LOGD("=== Disable trigger mode");
	TY_TRIGGER_PARAM trigger;
	trigger.mode = TY_TRIGGER_MODE_OFF;
	ret = TYSetStruct(hDevice, TY_COMPONENT_DEVICE, TY_STRUCT_TRIGGER_PARAM, &trigger, sizeof(trigger));
	if (ret < 0) {
		LOGD("Failed!");
		ASSERT_OK(TYCloseDevice(hDevice));
		ASSERT_OK(TYCloseInterface(hIface));
		delete frameBuffer[0];
		delete frameBuffer[1];
		return;
	}

	// 如果深度相机工作在触发接入模式（TY_TRIGGER_MODE_TRIG_SLAVE）下，可以使用软件触发接口函数TYSendSoftTrigger()，
	// 通过USB或者以太网发送指令，控制相机图像采集的时机。
	LOGD("=== Start capture");
	ret = TYStartCapture(hDevice);
	if (ret < 0) {
		LOGD("Failed!");
		ASSERT_OK(TYCloseDevice(hDevice));
		ASSERT_OK(TYCloseInterface(hIface));
		delete frameBuffer[0];
		delete frameBuffer[1];
		return;
	}

	bool saveFrame = false;
	int saveIdx = 0;
	cv::Mat depth;
	cv::Mat leftIR;
	cv::Mat rightIR;
	cv::Mat color;

	LOGD("=== Wait for callback");
	bool exit_main = false;
	DepthViewer depthViewer("Depth");

	cv::moveWindow("color", 0, 0);

	int count = 0;
	TY_FRAME_DATA frame;
	while (!exit_main) {
		// 主动获取深度数据模式下，应用可调用该接口获取深度数据。注意回调函数模式下不需要调用。 
		// 获取数据后，用户程序进行运算处理时，应采用独立线程，避免堵塞图像获取线程的运转。
		ret = TYFetchFrame(hDevice, &frame, 1000);
		if (ret == TY_STATUS_OK) {
			LOGD("=== Get frame %d", ++count);
			// parseFrame(frame, &depth, &leftIR, &rightIR, &color);
			parseFrame(frame, &depth, 0, 0, &color);

			if (!color.empty()) {
				LOGI("Color format is %s", colorFormatName(TYImageInFrame(frame, TY_COMPONENT_RGB_CAM)->pixelFormat));
			}

			LOGD("=== Callback: Re-enqueue buffer(%p, %d)", frame.userBuffer, frame.bufferSize);
			ASSERT_OK(TYEnqueueBuffer(hDevice, frame.userBuffer, frame.bufferSize));

			if (!depth.empty()) {
				depthViewer.show(depth);
			}
			if (!color.empty()) {
				cv::putText(color, "press button 's' to get img ", cv::Point(50, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(23, 23, 200), 2, 8);//在图片上写文字
				cv::putText(color, "press button 'x/Esc' to get out", cv::Point(50, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(23, 23, 200), 2, 8);//在图片上写文字

				cv::imshow("color", color);
			}

			if (saveFrame && !depth.empty()) {
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

				saveIdx++;
				saveFrame = false;
			}
		}

		if (device_offline) {
			LOGI("Found device offline");
			break;
		}

		int key = cv::waitKey(10);
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
	}

	if (device_offline) {
		LOGI("device offline release resource");
	}
	else {
		LOGI("normal exit");
	}
	// 停止

	LOGD("=== Deinit lib");
	ASSERT_OK(TYDeinitLib());

	cv::destroyAllWindows();
	ASSERT_OK(TYStopCapture(hDevice));
	ASSERT_OK(TYCloseDevice(hDevice));
	ASSERT_OK(TYCloseInterface(hIface));
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
