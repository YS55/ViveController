
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
	*	コンストラクタ
	*	引数：なし
	*	返り値：なし
	*/
	Vive();

	/*
	*	VIVEの初期化
	*	引数：なし
	*	返り値：VIVEの初期化の成功(0)/失敗(-1)
	*/
	int openVive();

	/*
	*	OpenGLの初期化
	*	引数：表示する画像データ
	*	返り値：OpenGLの初期化成功：0
	*		  ：ライブラリの初期化失敗：-1
	*	      ：PC画面上に表示するウィンドウの作成失敗：-2
	*/
	int initOpenGL(cv::Mat image);

	/*
	*	Viveの終了
	*	引数：なし
	*	返り値：なし
	*/
	void closeVive();

	/*
	*	OpenGLの処理の終了
	*	引数：なし
	*	返り値：なし
	*/
	void finish();

	/*
	*	接続されているVIVEデバイス数の取得
	*	引数：なし
	*	返り値：なし
	*/
	void countupConnectedDeviceNum();

	/*
	*	各デバイスの接続数を返す
	*	引数：なし
	*	返り値：各デバイスの接続数
	*/
	int returnNumHmd(){ return numHmd; }
	int returnNumController(){ return numController; }
	int returnNumTracker(){ return numTracker; }

	/*
	*	全てのデバイスのデータの取得
	*	引数：接続されている全てのデバイスのデータ
	*	返り値：なし
	*/
	void getAllViveData(ViveDeviceData* hmd, ControllerData* controller, ViveDeviceData* tracker);

	/*
	*	全てのデバイスのデータ取得と画像データの表示
	*	引数：接続されている全てのデバイスのデータ / 表示する画像データ
	*	返り値：なし
	*/
	void render(cv::Mat image, ViveDeviceData* hmd, ControllerData* controller, ViveDeviceData* tracker);

	/*
	*	イベントの確認
	*	引数：なし
	*	返り値：なし
	*/
	void checkEvent();

private:
	vr::IVRSystem *m_pHMD;

	//接続されているVIVEデバイスの個数
	vr::TrackedDeviceIndex_t numDevice;	//全デバイス
	int numHmd;			// VIVE HMD
	int numController;	// VIVEコントローラ
	int numTracker;		// VIVEトラッカー

	//コントローラのボタン・パッド・トリガーの状態の格納
	vr::VRControllerState_t controllerState;

	//VIVEデバイスの位置姿勢・速度の格納
	vr::TrackedDevicePose_t* trackedDevicePose;

	GLFWwindow* window;
	GLuint g_texID;
	uint32_t m_nRenderWidth, m_nRenderHeight;

	/*
	*	位置ベクトルの取得
	*	引数：行列
	*	返り値：なし
	*/
	vr::HmdVector3_t getPosition(vr::HmdMatrix34_t matrix);

	/*
	*	回転クオーテーションの取得
	*	引数：行列
	*	返り値：なし
	*/
	vr::HmdQuaternion_t getRotation(vr::HmdMatrix34_t matrix);
};

#endif VIVECONTROL_H_ 