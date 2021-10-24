// -*- C++ -*-
/*!
 * @file  VIVEController.cpp
 * @brief VIVEController
 * @date $Date$
 *
 * $Id$
 */

#include "VIVEController.h"

// Module specification
// <rtc-template block="module_spec">
static const char* vivecontroller_spec[] =
  {
    "implementation_id", "VIVEController",
    "type_name",         "VIVEController",
    "description",       "VIVEController",
    "version",           "1.0.0",
    "vendor",            "rsdlab",
    "category",          "VR",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
VIVEController::VIVEController(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_hmdImageIn("HmdImage", m_hmdImage),
    m_controllerOut("Controller", m_controller),
    m_trackerOut("Tracker", m_tracker),
    m_hmdOut("Hmd", m_hmd)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
VIVEController::~VIVEController()
{
}



RTC::ReturnCode_t VIVEController::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("HmdImage", m_hmdImageIn);
  
  // Set OutPort buffer
  addOutPort("Controller", m_controllerOut);
  addOutPort("Tracker", m_trackerOut);
  addOutPort("Hmd", m_hmdOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t VIVEController::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VIVEController::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VIVEController::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t VIVEController::onActivated(RTC::UniqueId ec_id)
{
	fopen_s(&fp, "vive.csv", "w");

	int error = vive.openVive();
	if (error)
	{
		printf_s("VIVE_INITIALIZE_ERROR:%d\n");
	}

	initOpenglFlag = true;

	return RTC::RTC_OK;
}


RTC::ReturnCode_t VIVEController::onDeactivated(RTC::UniqueId ec_id)
{
	fclose(fp);
	vive.closeVive();

	return RTC::RTC_OK;
}


RTC::ReturnCode_t VIVEController::onExecute(RTC::UniqueId ec_id)
{
	//�摜�f�[�^�̎擾
	if (m_hmdImageIn.isNew())
	{
		m_hmdImageIn.read();

		width = m_hmdImage.data.image.width;
		height = m_hmdImage.data.image.height;

		channels = (m_hmdImage.data.image.format == 1) ? 1 :
			(m_hmdImage.data.image.format == 2 || m_hmdImage.data.image.format == 4 || m_hmdImage.data.image.format == 3) ? 3 :
			(m_hmdImage.data.image.raw_data.length() / width / height);
		RTC_TRACE(("Capture image size %d x %d", width, height));
		RTC_TRACE(("Channels %d", channels));

		if (channels == 3)
			image.create(height, width, CV_8UC3);
		else
			image.create(height, width, CV_8UC1);

		long data_length = m_hmdImage.data.image.raw_data.length();

		if (m_hmdImage.data.image.format == Img::CF_RGB || m_hmdImage.data.image.format == Img::CF_GRAY)
		{
			for (int i = 0; i < height; ++i)
				memcpy(&image.data[i*image.step], &m_hmdImage.data.image.raw_data[i*width*channels], sizeof(unsigned char)*width*channels);
			if (channels == 3)
				cv::cvtColor(image, image, CV_RGB2BGR);
		}
		else if (m_hmdImage.data.image.format == Img::CF_JPEG || m_hmdImage.data.image.format == Img::CF_PNG)
		{
			std::vector<uchar> compressed_image = std::vector<uchar>(data_length);
			memcpy(&compressed_image[0], &m_hmdImage.data.image.raw_data[0], sizeof(unsigned char)* data_length);

			cv::Mat decoded_image;
			if (channels == 3)
			{
				decoded_image = cv::imdecode(cv::Mat(compressed_image), CV_LOAD_IMAGE_COLOR);
				cv::cvtColor(decoded_image, image, CV_RGB2BGR);
			}
			else
			{
				decoded_image = cv::imdecode(cv::Mat(compressed_image), CV_LOAD_IMAGE_GRAYSCALE);
				image = decoded_image;
			}
		}
	}

	//�ڑ�����Ă���f�o�C�X���̍X�V
	vive.countupConnectedDeviceNum();
	ViveDeviceData* pHmd = new ViveDeviceData[vive.returnNumHmd()];
	ControllerData* pController = new ControllerData[vive.returnNumController()];
	ViveDeviceData* pTracker = new ViveDeviceData[vive.returnNumTracker()];

	if (image.empty())
	{
		// �g���b�L���O�̂�
		vive.getAllViveData(pHmd, pController, pTracker);
	}
	else
	{
		if (initOpenglFlag)
		{
			initOpenglFlag = false;
			vive.initOpenGL(image);
		}

		// �g���b�L���O��HMD�ւ̕`��
		vive.render(image, pHmd, pController, pTracker);
	}


	//////////////////////////////////////
	// VIVE �w�b�h�}�E���g�f�B�X�v���C
	//////////////////////////////////////
	if (vive.returnNumHmd())
	{
		m_hmd.data.length(vive.returnNumHmd());

		for (int i = 0; i < vive.returnNumHmd(); i++)
		{
			// �ʒu
			m_hmd.data[i].pose.position.x = pHmd[i].position.v[0];
			m_hmd.data[i].pose.position.y = pHmd[i].position.v[1];
			m_hmd.data[i].pose.position.z = pHmd[i].position.v[2];
			// �p��
			m_hmd.data[i].pose.orientation.r = pHmd[i].rotation.x;
			m_hmd.data[i].pose.orientation.p = pHmd[i].rotation.y;
			m_hmd.data[i].pose.orientation.y = pHmd[i].rotation.z;
			// ���x
			m_hmd.data[i].velocities.vx = pHmd[i].velocity.v[0];
			m_hmd.data[i].velocities.vy = pHmd[i].velocity.v[1];
			m_hmd.data[i].velocities.vz = pHmd[i].velocity.v[2];
			// �p���x
			m_hmd.data[i].velocities.vr = pHmd[i].angularVelocity.v[0];
			m_hmd.data[i].velocities.vp = pHmd[i].angularVelocity.v[1];
			m_hmd.data[i].velocities.va = pHmd[i].angularVelocity.v[2];
		}

		m_hmdOut.write();
	}

	///////////////////////////////////
	// VIVE�R���g���[��
	///////////////////////////////////
	if (vive.returnNumController())
	{
		m_controller.data.length(vive.returnNumController());

		for (int i = 0; i < vive.returnNumController(); i++)
		{
			//�{�^��
			m_controller.data[i].systemButton = pController[i].systemButton;
			m_controller.data[i].applicationMenuButton = pController[i].applicationMenuButton;
			m_controller.data[i].gripButton = pController[i].gripButton;
			////�g���K�[
			m_controller.data[i].trigger = pController[i].trigger;
			//// �p�b�h
			m_controller.data[i].padx = pController[i].padx;
			m_controller.data[i].pady = pController[i].pady;
			// �ʒu
			m_controller.data[i].controllerPoseVel.pose.position.x = pController[i].pose.position.v[0];
			m_controller.data[i].controllerPoseVel.pose.position.y = pController[i].pose.position.v[1];
			m_controller.data[i].controllerPoseVel.pose.position.z = pController[i].pose.position.v[2];
			// �p��
			m_controller.data[i].controllerPoseVel.pose.orientation.r = pController[i].pose.rotation.x;
			m_controller.data[i].controllerPoseVel.pose.orientation.p = pController[i].pose.rotation.y;
			m_controller.data[i].controllerPoseVel.pose.orientation.y = pController[i].pose.rotation.z;
			// ���x
			m_controller.data[i].controllerPoseVel.velocities.vx = pController[i].pose.velocity.v[0];
			m_controller.data[i].controllerPoseVel.velocities.vy = pController[i].pose.velocity.v[1];
			m_controller.data[i].controllerPoseVel.velocities.vz = pController[i].pose.velocity.v[2];
			// �p���x
			m_controller.data[i].controllerPoseVel.velocities.vr = pController[i].pose.angularVelocity.v[0];
			m_controller.data[i].controllerPoseVel.velocities.vp = pController[i].pose.angularVelocity.v[1];
			m_controller.data[i].controllerPoseVel.velocities.va = pController[i].pose.angularVelocity.v[2];
		}

		m_controllerOut.write();
	}

	//////////////////////////////////////
	// VIVE �g���b�J�[
	//////////////////////////////////////
	if (vive.returnNumTracker())
	{
		m_tracker.data.length(vive.returnNumTracker());

		for (int i = 0; i < vive.returnNumTracker(); i++)
		{
			// �ʒu
			m_tracker.data[i].pose.position.x = pTracker[i].position.v[0];
			m_tracker.data[i].pose.position.y = pTracker[i].position.v[1];
			m_tracker.data[i].pose.position.z = pTracker[i].position.v[2];
			// �p��
			m_tracker.data[i].pose.orientation.r = pTracker[i].rotation.x;
			m_tracker.data[i].pose.orientation.p = pTracker[i].rotation.y;
			m_tracker.data[i].pose.orientation.y = pTracker[i].rotation.z;
			// ���x
			m_tracker.data[i].velocities.vx = pTracker[i].velocity.v[0];
			m_tracker.data[i].velocities.vy = pTracker[i].velocity.v[1];
			m_tracker.data[i].velocities.vz = pTracker[i].velocity.v[2];
			// �p���x
			m_tracker.data[i].velocities.vr = pTracker[i].angularVelocity.v[0];
			m_tracker.data[i].velocities.vp = pTracker[i].angularVelocity.v[1];
			m_tracker.data[i].velocities.va = pTracker[i].angularVelocity.v[2];
		}

		m_trackerOut.write();

	}

	GetSystemTime(&time);
	GetLocalTime(&time);
	fprintf_s(fp, "%2d,%2d,%2d,%3d,%f,%f,%f,",
		time.wHour,
		time.wMinute,
		time.wSecond,
		time.wMilliseconds,
		// HMD
		m_hmd.data[0].pose.position.x,
		m_hmd.data[0].pose.position.y,
		m_hmd.data[0].pose.position.z,
		m_hmd.data[0].pose.orientation.r,
		m_hmd.data[0].pose.orientation.p,
		m_hmd.data[0].pose.orientation.y,
		// Controller
		m_controller.data[0].controllerPoseVel.pose.position.x,
		m_controller.data[0].controllerPoseVel.pose.position.y,
		m_controller.data[0].controllerPoseVel.pose.position.z,
		m_controller.data[0].controllerPoseVel.pose.orientation.r,
		m_controller.data[0].controllerPoseVel.pose.orientation.p,
		m_controller.data[0].controllerPoseVel.pose.orientation.y,
		// Tracker 1
		m_tracker.data[0].pose.position.x,
		m_tracker.data[0].pose.position.y,
		m_tracker.data[0].pose.position.z,
		m_tracker.data[0].pose.orientation.r,
		m_tracker.data[0].pose.orientation.p,
		m_tracker.data[0].pose.orientation.y,
		// Tracker 2
		m_tracker.data[1].pose.position.x,
		m_tracker.data[1].pose.position.y,
		m_tracker.data[1].pose.position.z,
		m_tracker.data[1].pose.orientation.r,
		m_tracker.data[1].pose.orientation.p,
		m_tracker.data[1].pose.orientation.y
		);

	fprintf_s(fp, "\n");

	delete pHmd, pController, pTracker;

	return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t VIVEController::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VIVEController::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VIVEController::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VIVEController::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VIVEController::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void VIVEControllerInit(RTC::Manager* manager)
  {
    coil::Properties profile(vivecontroller_spec);
    manager->registerFactory(profile,
                             RTC::Create<VIVEController>,
                             RTC::Delete<VIVEController>);
  }
  
};


