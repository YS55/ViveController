
#ifndef VIVECONTROL_H_
#define VIVECONTROL_H_

#include <openvr.h>
#include <iostream>

//#include <opencv2\opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


//#include<GL\glew.h>
//#include<GLFW\glfw3.h>
#include<glew.h>
#include<glfw3.h>


typedef struct{
	vr::HmdVector3_t position;
	vr::HmdQuaternion_t rotation;
	vr::HmdVector3_t velocity;
	vr::HmdVector3_t angularVelocity;
}ViveDeviceData;

typedef struct{
	ViveDeviceData pose;
	bool gripButton;
	bool applicationMenuButton;
	bool systemButton;
	float trigger;
	float padx;
	float pady;
}ControllerData;


class Vive
{
public:

	/*
	*	�R���X�g���N�^
	*	�����F�Ȃ�
	*	�Ԃ�l�F�Ȃ�
	*/
	Vive();

	/*
	*	VIVE�̏�����
	*	�����F�Ȃ�
	*	�Ԃ�l�FVIVE�̏������̐���(0)/���s(-1)
	*/
	int openVive();

	/*
	*	OpenGL�̏�����
	*	�����F�\������摜�f�[�^
	*	�Ԃ�l�FOpenGL�̏����������F0
	*		  �F���C�u�����̏��������s�F-1
	*	      �FPC��ʏ�ɕ\������E�B���h�E�̍쐬���s�F-2
	*/
	int initOpenGL(cv::Mat image);

	/*
	*	Vive�̏I��
	*	�����F�Ȃ�
	*	�Ԃ�l�F�Ȃ�
	*/
	void closeVive();

	/*
	*	OpenGL�̏����̏I��
	*	�����F�Ȃ�
	*	�Ԃ�l�F�Ȃ�
	*/
	void finish();

	/*
	*	�ڑ�����Ă���VIVE�f�o�C�X���̎擾
	*	�����F�Ȃ�
	*	�Ԃ�l�F�Ȃ�
	*/
	void countupConnectedDeviceNum();

	/*
	*	�e�f�o�C�X�̐ڑ�����Ԃ�
	*	�����F�Ȃ�
	*	�Ԃ�l�F�e�f�o�C�X�̐ڑ���
	*/
	int returnNumHmd(){ return numHmd; }
	int returnNumController(){ return numController; }
	int returnNumTracker(){ return numTracker; }

	/*
	*	�S�Ẵf�o�C�X�̃f�[�^�̎擾
	*	�����F�ڑ�����Ă���S�Ẵf�o�C�X�̃f�[�^
	*	�Ԃ�l�F�Ȃ�
	*/
	void getAllViveData(ViveDeviceData* hmd, ControllerData* controller, ViveDeviceData* tracker);

	/*
	*	�S�Ẵf�o�C�X�̃f�[�^�擾�Ɖ摜�f�[�^�̕\��
	*	�����F�ڑ�����Ă���S�Ẵf�o�C�X�̃f�[�^ / �\������摜�f�[�^
	*	�Ԃ�l�F�Ȃ�
	*/
	void render(cv::Mat image, ViveDeviceData* hmd, ControllerData* controller, ViveDeviceData* tracker);

	/*
	*	�C�x���g�̊m�F
	*	�����F�Ȃ�
	*	�Ԃ�l�F�Ȃ�
	*/
	void checkEvent();

private:
	vr::IVRSystem *m_pHMD;

	//�ڑ�����Ă���VIVE�f�o�C�X�̌�
	vr::TrackedDeviceIndex_t numDevice;	//�S�f�o�C�X
	int numHmd;			// VIVE HMD
	int numController;	// VIVE�R���g���[��
	int numTracker;		// VIVE�g���b�J�[

	//�R���g���[���̃{�^���E�p�b�h�E�g���K�[�̏�Ԃ̊i�[
	vr::VRControllerState_t controllerState;

	//VIVE�f�o�C�X�̈ʒu�p���E���x�̊i�[
	vr::TrackedDevicePose_t* trackedDevicePose;

	GLFWwindow* window;
	GLuint g_texID;
	uint32_t m_nRenderWidth, m_nRenderHeight;

	/*
	*	�ʒu�x�N�g���̎擾
	*	�����F�s��
	*	�Ԃ�l�F�Ȃ�
	*/
	vr::HmdVector3_t getPosition(vr::HmdMatrix34_t matrix);

	/*
	*	��]�N�I�[�e�[�V�����̎擾
	*	�����F�s��
	*	�Ԃ�l�F�Ȃ�
	*/
	vr::HmdQuaternion_t getRotation(vr::HmdMatrix34_t matrix);
};

#endif VIVECONTROL_H_ 