#pragma once

///////////////////////////////////////////////////////////////////////////////////
/* 必要なライブラリの#defineのコメントアウトを外し指示がある場合はそれに従う
/* 必要であればインクルードフォルダのパスを通す
///////////////////////////////////////////////////////////////////////////////////

/* OpenCV */
// <TODO>
// 必要に応じて追加のインクルードディレクトリに"C:\opencv249\opencv\build\include"を追加
// ラベリングを行う場合はcvblob.h, cvblob.cpp, cvlabel.cppをプロジェクトに加える(namespace cvb)
// </TODO>
#define OPENCV

/* KinectSDK */
/// <TODO>
/// 必要に応じて追加のインクルードディレクトリに"$(KINECTSDK10_DIR)\inc"を追加
/// </TODO>
#define KINECTSDK

/* ARToolKit*/
// <TODO>
// Releaseでコンパイルすること
// Releaseのディレクトリに"DSVL.dll", "libARvideo.dll"を追加
// カメラパラメータを用意したファイルなどをDataなどのディレクトリに入れて用意する
// </TODO>
//#define ARTOOLKIT

///////////////////////////////////////////////////////////////////////////////////



#ifdef OPENCV
	#include <opencv2/opencv.hpp>

	// バージョン取得
	#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

	// ビルドモード
	#ifdef _DEBUG
	#define CV_EXT_STR "d.lib"
	#else
	#define CV_EXT_STR ".lib"
	#endif

	/// <Summary>
	/// ライブラリのリンク（不要な物はコメントアウト）
	///　</Summary>
	//　作業PC用
	#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\opencv\\build\\x86\\vc12\\lib\\opencv_core"       CV_VERSION_STR CV_EXT_STR)
	#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\opencv\\build\\x86\\vc12\\lib\\opencv_highgui"    CV_VERSION_STR CV_EXT_STR)
	#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\opencv\\build\\x86\\vc12\\lib\\opencv_imgproc"    CV_VERSION_STR CV_EXT_STR)
	#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\opencv\\build\\x86\\vc12\\lib\\opencv_calib3d"    CV_VERSION_STR CV_EXT_STR)
	#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\opencv\\build\\x86\\vc12\\lib\\opencv_gpu"        CV_VERSION_STR CV_EXT_STR)
	#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\opencv\\build\\x86\\vc12\\lib\\opencv_video"      CV_VERSION_STR CV_EXT_STR)
	#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\opencv\\build\\x86\\vc12\\lib\\opencv_objdetect"  CV_VERSION_STR CV_EXT_STR)
	#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\opencv\\build\\x86\\vc12\\lib\\opencv_features2d" CV_VERSION_STR CV_EXT_STR)
	#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\opencv\\build\\x86\\vc12\\lib\\opencv_flann"      CV_VERSION_STR CV_EXT_STR)
	#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\opencv\\build\\x86\\vc12\\lib\\opencv_ts"         CV_VERSION_STR CV_EXT_STR)
	#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\opencv\\build\\x86\\vc12\\lib\\opencv_contrib"    CV_VERSION_STR CV_EXT_STR)
	#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\opencv\\build\\x86\\vc12\\lib\\opencv_ml"         CV_VERSION_STR CV_EXT_STR)
	#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\opencv\\build\\x86\\vc12\\lib\\opencv_legacy"     CV_VERSION_STR CV_EXT_STR)
	// ミーティング室用
	//#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\build\\x86\\vc11\\lib\\opencv_core"       CV_VERSION_STR CV_EXT_STR)
	//#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\build\\x86\\vc11\\lib\\opencv_highgui"    CV_VERSION_STR CV_EXT_STR)
	//#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\build\\x86\\vc11\\lib\\opencv_imgproc"    CV_VERSION_STR CV_EXT_STR)
	//#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\build\\x86\\vc11\\lib\\opencv_calib3d"    CV_VERSION_STR CV_EXT_STR)
	////#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\build\\x86\\vc11\\lib\\opencv_gpu"        CV_VERSION_STR CV_EXT_STR)
	////#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\build\\x86\\vc11\\lib\\opencv_video"      CV_VERSION_STR CV_EXT_STR)
	//#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\build\\x86\\vc11\\lib\\opencv_objdetect"  CV_VERSION_STR CV_EXT_STR)
	////#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\build\\x86\\vc11\\lib\\opencv_features2d" CV_VERSION_STR CV_EXT_STR)
	////#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\build\\x86\\vc11\\lib\\opencv_flann"      CV_VERSION_STR CV_EXT_STR)
	////#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\build\\x86\\vc11\\lib\\opencv_ffmpeg"     CV_VERSION_STR CV_EXT_STR)
	////#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\build\\x86\\vc11\\lib\\opencv_ts"         CV_VERSION_STR CV_EXT_STR)
	////#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\build\\x86\\vc11\\lib\\opencv_contrib"    CV_VERSION_STR CV_EXT_STR)
	////#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\build\\x86\\vc11\\lib\\opencv_ml"         CV_VERSION_STR CV_EXT_STR)
	////#pragma comment(lib, "C:\\opencv" CV_VERSION_STR "\\build\\x86\\vc11\\lib\\opencv_legacy"     CV_VERSION_STR CV_EXT_STR)




	/////////////////////////////////
	// Additional option

	// キー入力
	#define KEY_ESC		0x1b
	#define KEY_SPACE	0x20


	/// マウスイベント
	//コールバック関数からのイベント情報を取得するための構造体
	typedef struct MouseParam{
		unsigned int x;
		unsigned int y;
		int event;
		int flags;
	} MouseParam;

#endif


#ifdef KINECTSDK

	// NuiApi.hの前にWindows.hをインクルードする
	#include <Windows.h>
	#include <NuiApi.h>

	// ライブラリのリンク（不要な物はコメントアウト）
	#pragma comment(lib, "C:\\Program Files\\Microsoft SDKs\\Kinect\\v1.8\\lib\\x86\\Kinect10.lib")

	// なんかバグ回避用
	#ifdef min
	#undef min
	#endif
	#ifdef max
	#undef max
	#endif

#endif

#ifdef ARTOOLKIT
	#include <GL/glut.h>
	#include <AR/config.h>
	#include <AR/video.h>
	#include <AR/param.h>			// arParamDisp()
	#include <AR/ar.h>
	#include <AR/gsub_lite.h>

	#ifdef _DEBUG
	#define ARTOOLKIT_EXT_STR "d.lib"
	#else
	#define ARTOOLKIT_EXT_STR ".lib"
	#endif

	#pragma comment(lib, "C:\\Program Files (x86)\\ARToolKit\\lib\\glut32.lib")
	#pragma comment(lib, "C:\\Program Files (x86)\\ARToolKit\\lib\\libAR"			ARTOOLKIT_EXT_STR)
	#pragma comment(lib, "C:\\Program Files (x86)\\ARToolKit\\lib\\libARgsub"		ARTOOLKIT_EXT_STR)
	#pragma comment(lib, "C:\\Program Files (x86)\\ARToolKit\\lib\\libARgsub_lite"	ARTOOLKIT_EXT_STR)
	#pragma comment(lib, "C:\\Program Files (x86)\\ARToolKit\\lib\\libARgsubUtil"	ARTOOLKIT_EXT_STR)
	#pragma comment(lib, "C:\\Program Files (x86)\\ARToolKit\\lib\\libARMulti"		ARTOOLKIT_EXT_STR)
	#pragma comment(lib, "C:\\Program Files (x86)\\ARToolKit\\lib\\libARvideo"		ARTOOLKIT_EXT_STR)
	#pragma comment(lib, "C:\\Program Files (x86)\\ARToolKit\\lib\\libARvrml"		ARTOOLKIT_EXT_STR)
#endif