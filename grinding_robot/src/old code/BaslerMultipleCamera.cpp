#include "BaslerMultipleCamera.h"

BaslerMultipleCamera::BaslerMultipleCamera(TRIGGERSOURCE trigger, CAMERATYPE camtype):_trigger(trigger),_cameraType(camtype)
{

}

BaslerMultipleCamera::~BaslerMultipleCamera()
{
	destroyCam();
}
bool BaslerMultipleCamera::initCam(){
	Pylon::PylonInitialize();
	numberofCam = 0;
	try
	{

		// Get the transport layer factory.
		CTlFactory& tlFactory = CTlFactory::GetInstance();
		// Get all attached devices and exit application if no device is found.
		DeviceInfoList_t devices;
		if (tlFactory.EnumerateDevices(devices) == 0)
		{
			throw RUNTIME_EXCEPTION("No camera present.");
		}
		_cameras.Initialize(std::min(devices.size(), c_maxCamerasToUse));

		// Create and attach all Pylon Devices.
		for (size_t i = 0; i < _cameras.GetSize(); ++i)
		{
#ifdef GigEcam
			_cameras[i].Attach(tlFactory.CreateDevice(devices[i]));
			// Print the model name of the camera.
			std::cout << "Using device " << _cameras[i].GetDeviceInfo().GetModelName() << std::endl;
			Pylon::String_t serial = _cameras[i].GetDeviceInfo().GetSerialNumber();
			std::cout << "Serial Number: " << serial << std::endl;
			_serial.push_back(serial);
			_cameras[i].Open();
			_cameras[i].MaxNumBuffer = 1;
			_cameras[i].AcquisitionMode.SetValue(Basler_GigECameraParams::AcquisitionMode_Continuous);
			_cameras[i].TriggerSelector.SetValue(Basler_GigECameraParams::TriggerSelector_FrameStart);
			_cameras[i].TriggerMode.SetValue(Basler_GigECameraParams::TriggerMode_On);
			if (_trigger == TRIGGERSOURCE::HARDWARE) {
				_cameras[i].TriggerSource.SetValue(Basler_GigECameraParams::TriggerSource_Line1);
			}
			else {
				_cameras[i].AcquisitionStatusSelector.SetValue(Basler_GigECameraParams::AcquisitionStatusSelector_FrameTriggerWait);
				_cameras[i].TriggerSource.SetValue(Basler_GigECameraParams::TriggerSource_Software);
			}
			_cameras[i].ExposureMode.SetValue(Basler_GigECameraParams::ExposureMode_Timed);
			_cameras[i].ExposureTimeAbs.SetValue(16666.0);
			isConnected_ = true;
			numberofCam += 1;
#else
			_cameras[i].Attach(tlFactory.CreateDevice(devices[i]));
			std::cout << "Using device " << _cameras[i].GetDeviceInfo().GetModelName() << std::endl;
			Pylon::String_t serial = _cameras[i].GetDeviceInfo().GetSerialNumber();
			std::cout << "Serial Number: " << serial << std::endl;
			_serial.push_back(serial);
			_cameras[i].Open();
			_cameras[i].MaxNumBuffer = 1;
			_cameras[i].AcquisitionMode.SetValue(Basler_UsbCameraParams::AcquisitionMode_Continuous);
			_cameras[i].TriggerSelector.SetValue(Basler_UsbCameraParams::TriggerSelector_FrameStart);
			_cameras[i].TriggerMode.SetValue(Basler_UsbCameraParams::TriggerMode_On);
			if (_trigger == TRIGGERSOURCE::HARDWARE) {
				_cameras[i].TriggerSource.SetValue(Basler_UsbCameraParams::TriggerSource_Line1);
			}
			else {
				_cameras[i].AcquisitionStatusSelector.SetValue(Basler_UsbCameraParams::AcquisitionStatusSelector_FrameTriggerWait);
				_cameras[i].TriggerSource.SetValue(Basler_UsbCameraParams::TriggerSource_Software);
			}
			_cameras[i].ExposureMode.SetValue(Basler_UsbCameraParams::ExposureMode_Timed);
			_cameras[i].ExposureTimeAbs.SetValue(16666.0);
			isConnected_ = true;
			numberofCam += 1;
#endif
		}
	}
	catch (const  GenericException &e)
	{
		std::cerr << "[ERROR] an exception occured :" << e.GetDescription() << std::endl;
		isConnected_ = 0;
	}

	return isConnected_;
}





bool BaslerMultipleCamera::startGrab(){
	std::cerr << "************[INFO] Grabbing images end***********" << std::endl;
	_cameras.StartGrabbing(Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByInstantCamera);

	for (int i = 0; i < numberofCam; i++) {
		try {
			_cameras[i].AcquisitionStart.Execute();
		}
		catch (const Pylon::GenericException & e) {
			std::cerr << "[ERROR] an exception occured : " << e.GetDescription() << std::endl;
		}
	}

	return false;
}

bool BaslerMultipleCamera::stopGrab()
{
	std::cerr << "************[INFO] Grabbing images end***********" << std::endl;

	for (int i = 0; i < numberofCam; i++) {
		try {
			_cameras[i].AcquisitionStop.Execute();
		}
		catch (const Pylon::GenericException & e) {
			std::cerr << "[ERROR] an exception occured : " << e.GetDescription() << std::endl;
		}
	}


	return false;
}

bool BaslerMultipleCamera::softwareTrigger()
{
	bool isWaitFrame = false;
	if (!isConnected_) return false;

	for (int i = 0; i < numberofCam; i++) {
		if (!_cameras[i].IsGrabbing()) return false;
		isWaitFrame = _cameras[i].WaitForFrameTriggerReady(100, Pylon::TimeoutHandling_ThrowException);

		if (isWaitFrame == true) {
			_cameras[i].TriggerSoftware.Execute();
		}
	}

	return isWaitFrame;
}

void BaslerMultipleCamera::destroyCam()
{
	std::cerr << "************[INFO] Grabbing images end***********" << std::endl;

	for (int i = 0; i < numberofCam; i++) {
		try {
			_cameras[i].Close();
		}
		catch (const Pylon::GenericException & e) {
			std::cerr << "[ERROR] an exception occured : " << e.GetDescription() << std::endl;
		}
	}

	Pylon::PylonTerminate();
}

int BaslerMultipleCamera::getNumberofCam()
{
	return numberofCam;
}

bool BaslerMultipleCamera::checkSerialNumber(Pylon::String_t serial)
{
	bool flag = false;
	for (int i = 0; i < _serial.size(); i++) {
		if (_serial[i] == serial) {
			flag = true;
		}
	}
	return flag;
}

void BaslerMultipleCamera::OnImageGrabbed(Pylon::CBaslerGigEInstantCamera & camera, const Pylon::CBaslerUsbGrabResultPtr & ptrGrabResult)
{

}


