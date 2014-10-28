// MatCalclation.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"


int _tmain(int argc, _TCHAR* argv[])
{
	const char* nameTD_C = "data\\T_KD2C.xml";
	const char* nameTC_M = "data\\T_Kinect2Marker.xml";
	const char* nameTM_D = "data\\T_Marker2Display.xml";

	CalcMat app;


	app.SetMat(nameTD_C, "mat_array");
	app.SetMat(nameTC_M, "mat_array");
	app.SetMat(nameTM_D, "T_M2D");

	app.MultiMat("data\\T_Kinect2Display.xml");

	getchar();
	return 0;
}

