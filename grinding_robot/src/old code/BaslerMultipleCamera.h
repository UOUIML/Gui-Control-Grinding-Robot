
#pragma once

#include <pylon/PylonIncludes.h>
#ifdef PYLON_WIN_BUILD
# include <pylon/PylonGUI.h>
#endif
#include <iostream>

#include <pylon/InstantCameraArray.h>
#include <pylon/InstantCamera.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <pylon/gige/BaslerGigEInstantCameraArray.h>

//#include <pylon/HardwareTriggerConfiguration.h>

#include <pylon/usb/BaslerUsbInstantCamera.h>
#include <pylon/usb/BaslerUsbInstantCameraArray.h>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

enum TRIGGERSOURCE { SOFTWARE = 1, HARDWARE = 2 };
enum CAMERATYPE {USB3 =1, GiGE =2};
using namespace Pylon;
using namespace GenApi;

#define GigEcam

static const size_t c_maxCamerasToUse = 2;

class BaslerMultipleCamera
{
	public:
		BaslerMultipleCamera(TRIGGERSOURCE trigger,CAMERATYPE camtype);
		~BaslerMultipleCamera();

		bool initCam();
		bool startGrab();
		bool stopGrab();
		bool softwareTrigger();
		void destroyCam();

		int getNumberofCam();
		bool checkSerialNumber(Pylon::String_t serial);

		template<typename ObjectEvent>
			void imageEventHandleIndex(ObjectEvent* rObject,int id);

		template<typename ObjectEvent>
			void imageEventHandleSerial(ObjectEvent* rObject, Pylon::String_t serial);

		virtual void OnImageGrabbed(Pylon::CBaslerGigEInstantCamera & camera, const Pylon::CBaslerUsbGrabResultPtr & ptrGrabResult);
		std::vector<Pylon::String_t> _serial;
	private:
		TRIGGERSOURCE _trigger;
		CAMERATYPE _cameraType;
		bool isConnected_ = false;
		uint numberofCam = 0;

#ifdef GigEcam
		CBaslerGigEInstantCameraArray _cameras;
#else
		CBaslerUsbInstantCameraArray _cameras;
#endif // GigEcam

};

	template<typename ObjectEvent>
inline void BaslerMultipleCamera::imageEventHandleIndex(ObjectEvent * rObject,int id)
{
	_cameras[id].RegisterImageEventHandler(rObject, Pylon::RegistrationMode_Append, Pylon::Cleanup_Delete);
}

	template<typename ObjectEvent>
inline void BaslerMultipleCamera::imageEventHandleSerial(ObjectEvent * rObject, Pylon::String_t serial)
{
	for (int i = 0; i < _serial.size(); i++) {
		if (_serial[i] == serial) {
			_cameras[i].RegisterImageEventHandler(rObject, Pylon::RegistrationMode_Append, Pylon::Cleanup_Delete);
			break;
		}
	}
}

