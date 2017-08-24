#include "camdecorate.h"

namespace neolix{

#include <iostream>
#define CHECK(x) do \
	{\
	int err = x;\
	if(x != TY_STATUS_OK){ std::cout<<"fail"<<std::endl;return NEOLIX_FALSE;}\
} while (0);

	NEOLIX_STATUS  Capturer::camInit()
	{
		//CHECK(TYInitLib());//初始化API
		ASSERT_OK( TYInitLib() );
		//获取API版本
		this->pVer = new TY_VERSION_INFO;
		ASSERT_OK(TYLibVersion(pVer));
		LOGD("     - lib version: %d.%d.%d", pVer->major, pVer->minor, pVer->patch);

		//获取设备数量
		ASSERT_OK(TYGetDeviceNumber(&deviceNumber));
		LOGD("     - device number %d", deviceNumber);

		//获得设备基本信息
		this->pDeviceBaseInfo = new TY_DEVICE_BASE_INFO[100];
		ASSERT_OK(TYGetDeviceList(pDeviceBaseInfo,100,&deviceNumber));
		if (deviceNumber == 0)	LOGERROR("没有查找到摄像设备");
		//打开设备
		ASSERT_OK(TYOpenDevice(pDeviceBaseInfo[0].id, &hDevice));
		//使能组件
		
		cb_data.index = 0;
		cb_data.hDevice = hDevice;
		cb_data.render = &render;
		if (capModel == FETCH_MODEL) fectModel();
		else if(capModel == CALLBACK_MODEL)  ;
		else if(capModel == TRIGGER_MODEL)   ;
		else if(capModel == REGISTRATE_MODEL)  ;
		else return NEOLIX_FALSE;
		hasOpen = true;
		return NEOLIX_SUCCESS;
	}

	void Capturer::fectModel()
	{
		TY_FEATURE_INFO info ;
		TY_STATUS ty_status;
		int32_t frameSize;

		ASSERT_OK(TYEnableComponents(hDevice,componentIDs));
		//设置摄像头参数
		//qTYSetEnum(hDevice,componentIDs,enum_model,enum_model_value);
		//TYSetEnum(hDevice,TY_COMPONENT_RGB_CAM_LEFT,TY_ENUM_IMAGE_MODE,TY_IMAGE_MODE_640x480);
		TYSetEnum(hDevice,TY_COMPONENT_DEPTH_CAM,TY_ENUM_IMAGE_MODE,TY_IMAGE_MODE_640x480);

		if (frameBuffer[0] != nullptr) delete[] frameBuffer[0];
		if (frameBuffer[1] != nullptr) delete[] frameBuffer[1];
		//获得一帧数据的大小
		TYGetFrameBufferSize(hDevice,&frameSize);
		frameBuffer[0] = new char[frameSize];
		frameBuffer[1] = new char[frameSize];
		//===Enqueue buffer =====
		TYEnqueueBuffer(hDevice,frameBuffer[0],frameSize);
		TYEnqueueBuffer(hDevice,frameBuffer[1],frameSize);
		//=====disable trigger model====
		ty_status = TYGetFeatureInfo(hDevice,TY_COMPONENT_DEVICE,TY_BOOL_TRIGGER_MODE,&info);
		if ((info.accessMode & TY_ACCESS_WRITABLE) && (ty_status == TY_STATUS_OK)) {
			ASSERT_OK(TYSetBool(hDevice, TY_COMPONENT_DEVICE, TY_BOOL_TRIGGER_MODE, false));
		}
		//=====start capture====
		TYStartCapture(hDevice);
		
	}
	void Capturer::setlaserPower(uint32_t value)
	{
		uint32_t componentID = TY_COMPONENT_LASER;
		TY_FEATURE_ID_LIST feature   = TY_INT_LASER_POWER;
		TY_FEATURE_INFO info;
		TYGetFeatureInfo(hDevice,componentID,feature,&info);
//		ASSERT_OK(TYEnableComponents(hDevice,componentID));
	//	TYSetEnum(hDevice,componentID,feature,value);
		TYSetInt(hDevice,componentID,feature,value);
	}

	Capturer& Capturer::operator>>(deviceDataBase *framedate)
	{
		
		int err = TYFetchFrame(hDevice, &frame, -1);
		if( err != TY_STATUS_OK ) {
			LOGD("... Drop one frame");
		} else {
			frameHandler(&frame, &cb_data);
		}
		return *this;
	
	}

	void Capturer::frameHandler(TY_FRAME_DATA* frame, void* userdata)
	{
		deviceDataBase* pData = (deviceDataBase*)userdata;
		parseFrame(*frame,&(pData->depth),&(pData->leftIR),&(pData->rightIR),&(pData->leftRGB),&(pData->rightRGB),&(pData->point3D));
		ASSERT_OK( TYEnqueueBuffer(pData->hDevice, frame->userBuffer, frame->bufferSize) );
	}


	Capturer::Capturer(uint32_t componentIDs /* = TY_COMPONENT_DEPTH_CAM */,
						uint32_t enum_model /* = TY_ENUM_IMAGE_MODE */,
						uint32_t enum_model_value /* = TY_IMAGE_MODE_640x480 */
						,CaptuteModel capModel /* = FETCH_MODEL */):hasOpen(false)
	{
		this->componentIDs     = componentIDs;
		this->enum_model       = enum_model;
		this->enum_model_value = enum_model_value;
		this->capModel         = capModel;
		this->pVer             = new TY_VERSION_INFO;
		this->pDeviceBaseInfo  = new TY_DEVICE_BASE_INFO;
		frameBuffer[0]         = nullptr;
		frameBuffer[1]         = nullptr;
	}
	Capturer::~Capturer()
	{
		ASSERT_OK( TYStopCapture(hDevice) );
		ASSERT_OK( TYCloseDevice(hDevice) );
		ASSERT_OK( TYDeinitLib() );
		if (pVer != nullptr) delete pVer;
		if(pDeviceBaseInfo != nullptr) delete pDeviceBaseInfo;
		if(frameBuffer[0] != nullptr) delete[] frameBuffer[0];
		if(frameBuffer[1] != nullptr) delete[] frameBuffer[1];
		
	}

	NEOLIX_STATUS Capturer::open(uint32_t componentIDs /* = TY_COMPONENT_DEPTH_CAM */,
								uint32_t enum_model /* = TY_ENUM_IMAGE_MODE */,
								uint32_t enum_model_value /* = TY_IMAGE_MODE_640x480 */,
								CaptuteModel capModel /* = FETCH_MODEL */)
	{
		this->componentIDs     = componentIDs;
		this->enum_model       = enum_model;
		this->enum_model_value = enum_model_value;
		this->capModel         = capModel;
		this->pVer = new TY_VERSION_INFO;
		this->pDeviceBaseInfo = new TY_DEVICE_BASE_INFO;
		return camInit();
	}
}