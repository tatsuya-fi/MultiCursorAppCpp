#pragma once

///////////////////////////////////////////////////////////////////////////////////
/* �K�v�ȃ��C�u������#define�̃R�����g�A�E�g���O���w��������ꍇ�͂���ɏ]��
/* �K�v�ł���΃C���N���[�h�t�H���_�̃p�X��ʂ�
///////////////////////////////////////////////////////////////////////////////////

/* OpenCV */
// <TODO>
// �K�v�ɉ����Ēǉ��̃C���N���[�h�f�B���N�g����"C:\opencv249\opencv\build\include"��ǉ�
// ���x�����O���s���ꍇ��cvblob.h, cvblob.cpp, cvlabel.cpp���v���W�F�N�g�ɉ�����(namespace cvb)
// </TODO>
#define OPENCV

/* KinectSDK */
/// <TODO>
/// �K�v�ɉ����Ēǉ��̃C���N���[�h�f�B���N�g����"$(KINECTSDK10_DIR)\inc"��ǉ�
/// </TODO>
#define KINECTSDK

/* ARToolKit*/
// <TODO>
// Release�ŃR���p�C�����邱��
// Release�̃f�B���N�g����"DSVL.dll", "libARvideo.dll"��ǉ�
// �J�����p�����[�^��p�ӂ����t�@�C���Ȃǂ�Data�Ȃǂ̃f�B���N�g���ɓ���ėp�ӂ���
// </TODO>
//#define ARTOOLKIT

/* OpenGL */
/// <TODO>
/// glut�̓��� http://www.natural-science.or.jp/article/20101130220646.php
/// </TODO>
#define OPENGL_GLUT

///////////////////////////////////////////////////////////////////////////////////



#ifdef OPENCV
	#include <opencv2/opencv.hpp>

	// �o�[�W�����擾
	#define CV_VERSION_STR CVAUX_STR(CV_MAJOR_VERSION) CVAUX_STR(CV_MINOR_VERSION) CVAUX_STR(CV_SUBMINOR_VERSION)

	// �r���h���[�h
	#ifdef _DEBUG
	#define CV_EXT_STR "d.lib"
	#else
	#define CV_EXT_STR ".lib"
	#endif

	/// <Summary>
	/// ���C�u�����̃����N�i�s�v�ȕ��̓R�����g�A�E�g�j
	///�@</Summary>
	//�@���PC�p
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
	// �~�[�e�B���O���p
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

	// �L�[����
	#define KEY_ESC		0x1b
	#define KEY_SPACE	0x20


	/// �}�E�X�C�x���g
	//�R�[���o�b�N�֐�����̃C�x���g�����擾���邽�߂̍\����
	typedef struct MouseParam{
		unsigned int x;
		unsigned int y;
		int event;
		int flags;
	} MouseParam;

#endif


#ifdef KINECTSDK

	// NuiApi.h�̑O��Windows.h���C���N���[�h����
	#include <Windows.h>
	#include <Ole2.h>
	#include <NuiApi.h>

	// ���C�u�����̃����N�i�s�v�ȕ��̓R�����g�A�E�g�j
	#pragma comment(lib, "C:\\Program Files\\Microsoft SDKs\\Kinect\\v1.8\\lib\\x86\\Kinect10.lib")

	// �Ȃ񂩃o�O���p
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


#ifdef OPENGL_GLUT
	// #include <GL/glut.h>
	#include <gl/glut.h>
#endif