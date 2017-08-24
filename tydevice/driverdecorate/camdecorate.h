#ifndef  NEOLIX_CAMDECORATE_H
#define  NEOLIX_CAMDECORATE_H

#include "../common/common.hpp"
#include "base.h"

namespace neolix
{

	typedef struct CallbackData {
		int             index;
		TY_DEV_HANDLE   hDevice;
		DepthRender*    render;
		bool            saveFrame;
		int             saveIdx;
		cv::Mat         depth;
		cv::Mat         leftIR;
		cv::Mat         rightIR;
		cv::Mat         leftRGB;
		cv::Mat         rightRGB;
		cv::Mat         point3D;
	} deviceDataBase;
	
	class Capturer
	{
	public:
		enum CaptuteModel
		{
			CALLBACK_MODEL   = 0,
			FETCH_MODEL      = 1,
			TRIGGER_MODEL    = 2,
			REGISTRATE_MODEL = 3,
		};
		//���ú���Ͷ��level[0-100]
		void setlaserPower(uint32_t value);
		Capturer& operator>>(deviceDataBase *frame);
		deviceDataBase* getFrame()
		{
			return &(this->cb_data);
		}
		NEOLIX_STATUS open(uint32_t componentIDs = TY_COMPONENT_DEPTH_CAM, uint32_t enum_model = TY_ENUM_IMAGE_MODE, uint32_t enum_model_value = TY_IMAGE_MODE_640x480,CaptuteModel capModel = FETCH_MODEL);
		bool isOpen()
		{
			return this->hasOpen;
		}
		Capturer(uint32_t componentIDs = TY_COMPONENT_DEPTH_CAM, uint32_t enum_model = TY_ENUM_IMAGE_MODE, uint32_t enum_model_value = TY_IMAGE_MODE_640x480,CaptuteModel capModel = FETCH_MODEL);
		virtual ~Capturer();
		TY_DEV_HANDLE& gethDevice()
		{
			return this->hDevice;
		}
	protected:

		NEOLIX_STATUS camInit();
	private:

		int deviceNumber;//�豸����
		TY_VERSION_INFO * pVer ;//api�汾
		TY_DEVICE_BASE_INFO* pDeviceBaseInfo;//�豸�Ļ�����Ϣ
		TY_DEV_HANDLE hDevice;//�豸�ľ��
		int32_t componentIDs;//���IDs
		int32_t enum_model;
		int32_t enum_model_value;
		bool printLog;
		std::string logFile;
		deviceDataBase cb_data;
		DepthRender render;//��Ҫ��������ɼ������ݵĹ��ܣ��������ͼת���ɲ�ɫͼ��
		CaptuteModel capModel;
		char* frameBuffer[2];
		TY_FRAME_DATA frame;
		bool hasOpen;
		static void frameHandler(TY_FRAME_DATA* frame, void* userdata);
		void fectModel();
		

	};
}

#endif