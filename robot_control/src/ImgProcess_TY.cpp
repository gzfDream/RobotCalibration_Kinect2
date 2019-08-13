#include "ImgProcess_TY.h"

ImgProcess_TY::ImgProcess_TY() {
	LOGD("=== Init lib");
	int loop_index = 1;
	bool loop_exit = false;

	ASSERT_OK(TYInitLib());
	TY_VERSION_INFO ver;
	ASSERT_OK(TYLibVersion(&ver));
	LOGD("     - lib version: %d.%d.%d", ver.major, ver.minor, ver.patch);
}
ImgProcess_TY::~ImgProcess_TY() {
	LOGD("=== Deinit lib");
	ASSERT_OK(TYDeinitLib());
}

void eventCallback(TY_EVENT_INFO *event_info, void *userdata)
{
	if (event_info->eventId == TY_EVENT_DEVICE_OFFLINE) {
		LOGD("=== Event Callback: Device Offline!");
		*(bool*)userdata = true;
	}
	else if (event_info->eventId == TY_EVENT_LICENSE_ERROR) {
		LOGD("=== Event Callback: License Error!");
	}
}

void ImgProcess_TY::getImage(std::string str) {
	bool loop_exit = false;

	while (!loop_exit) {
		LOGD("=================================");
		LOGD("===== start capturing image =====");
		LOGD("=================================");

		// ���豸
		std::string ID, IP;

		TY_INTERFACE_HANDLE hIface;
		TY_DEV_HANDLE hDevice;
		std::vector<TY_DEVICE_BASE_INFO> selected;

		int ret = 0;
		ret = selectDevice(TY_INTERFACE_ALL, ID, IP, 1, selected);
		if (ret < 0) {
			MSleep(2000);
			continue;
		}

		ASSERT(selected.size() > 0);
		TY_DEVICE_BASE_INFO& selectedDev = selected[0];

		LOGD("=== Open interface: %s", selectedDev.iface.id);
		ASSERT_OK(TYOpenInterface(selectedDev.iface.id, &hIface));

		LOGD("=== Open device: %s", selectedDev.id);
		ret = TYOpenDevice(hIface, selectedDev.id, &hDevice);
		if (ret < 0) {
			LOGD("Failed!");
			ASSERT_OK(TYCloseInterface(hIface));
			continue;
		}

		// ��ѯ�豸֧�ֵ����
		int32_t allComps;
		ret = TYGetComponentIDs(hDevice, &allComps);
		if (ret < 0) {
			LOGD("Failed!");
			ASSERT_OK(TYCloseDevice(hDevice));
			ASSERT_OK(TYCloseInterface(hIface));
			continue;
		}

		// ��RGB���
		if (allComps & TY_COMPONENT_RGB_CAM) {
			LOGD("=== Has RGB camera, open RGB cam");
			ret = TYEnableComponents(hDevice, TY_COMPONENT_RGB_CAM);
			if (ret < 0) {
				LOGD("Failed!");
				ASSERT_OK(TYCloseDevice(hDevice));
				ASSERT_OK(TYCloseInterface(hIface));
				continue;
			}
		}

		// ��depth���
		LOGD("=== Configure components, open depth cam");
		int32_t componentIDs = TY_COMPONENT_DEPTH_CAM;
		ret = TYEnableComponents(hDevice, componentIDs);
		if (ret < 0) {
			LOGD("Failed!");
			ASSERT_OK(TYCloseDevice(hDevice));
			ASSERT_OK(TYCloseInterface(hIface));
			continue;
		}

		// ͼ��ĸ�ʽ�ͷֱ���
		LOGD("=== Configure feature, set resolution.");
		ret = TYSetEnum(hDevice, TY_COMPONENT_DEPTH_CAM, TY_ENUM_IMAGE_MODE, TY_IMAGE_MODE_DEPTH16_1280x960);//TY_IMAGE_MODE_DEPTH16_640x480
		if (ret < 0) {
			LOGD("Failed!");
			ASSERT_OK(TYCloseDevice(hDevice));
			ASSERT_OK(TYCloseInterface(hIface));
			continue;
		}

		// ��ȡ��ǰ�豸������Ҫ��֡����ռ��С��ͬһ����豸�������ڲ�ͬͼ���������ģʽ�£���Ҫ��֡��������С��ͬ��
		LOGD("=== Prepare image buffer");
		uint32_t frameSize;
		ret = TYGetFrameBufferSize(hDevice, &frameSize);
		if (ret < 0) {
			LOGD("Failed!");
			ASSERT_OK(TYCloseDevice(hDevice));
			ASSERT_OK(TYCloseInterface(hIface));
			continue;
		}

		// �ѷ����֡�������뻺����С�
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

		//  ��������豸������ģʽ0���������ͼ�����ݡ�
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
			continue;
		}

		// ��������������ڴ�������ģʽ��TY_TRIGGER_MODE_TRIG_SLAVE���£�����ʹ����������ӿں���TYSendSoftTrigger()��
		// ͨ��USB������̫������ָ��������ͼ��ɼ���ʱ����
		LOGD("=== Start capture");
		ret = TYStartCapture(hDevice);
		if (ret < 0) {
			LOGD("Failed!");
			ASSERT_OK(TYCloseDevice(hDevice));
			ASSERT_OK(TYCloseInterface(hIface));
			delete frameBuffer[0];
			delete frameBuffer[1];
			continue;
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

		cv::moveWindow("color",0, 0);

		int count = 0;
		TY_FRAME_DATA frame;
		while (!exit_main) {
			// ������ȡ�������ģʽ�£�Ӧ�ÿɵ��øýӿڻ�ȡ������ݡ�ע��ص�����ģʽ�²���Ҫ���á� 
			// ��ȡ���ݺ��û�����������㴦��ʱ��Ӧ���ö����̣߳��������ͼ���ȡ�̵߳���ת��
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
					cv::putText(color, "press button 's' to get img ", cv::Point(50, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(23, 23, 200), 2, 8);//��ͼƬ��д����
					cv::putText(color, "press button 'x/Esc' to get out", cv::Point(50, 90), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(23, 23, 200), 2, 8);//��ͼƬ��д����

					cv::imshow("color", color);
				}

				if (saveFrame && !depth.empty()) {
					if (saveIdx<10) {
						// cv::imwrite(str + "0" + std::to_string(saveIdx) + ".png", depth);
						cv::imwrite(str + "0" + std::to_string(saveIdx) + ".png", color);
					}
					else {
						// cv::imwrite(str + std::to_string(saveIdx) + ".png", depth);
						cv::imwrite(str + std::to_string(saveIdx) + ".png", color);
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
				/*case 'q':
					exit_main = true;
					break;*/
			case 's':
				saveFrame = true;
				break;
			case 'x':
				exit_main = true;
				loop_exit = true;
				break;
			case 27:
				exit_main = true;
				loop_exit = true;
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
		// ֹͣ
		ASSERT_OK(TYStopCapture(hDevice));
		ASSERT_OK(TYCloseDevice(hDevice));
		ASSERT_OK(TYCloseInterface(hIface));
		delete frameBuffer[0];
		delete frameBuffer[1];
	}
}
