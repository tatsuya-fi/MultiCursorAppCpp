#pragma once

///
/// <Settings> 調整可能なパラメータ
/// 

#define NEAR_MODE		// nearモードを使う場合はコメントを外す

// The resolution of the kinect depth camera
const NUI_IMAGE_RESOLUTION KINECT_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;

// The height of Kinect which is set on the celling
const int KINECT_HEIGHT = 2700;
// The threshold which detect users in the first step (The distance from the floor)
// 最初にユーザを検出する時のしきい値(地面からの高さ)[mm]
const int USER_HEIGHT_THRESHOLD = 1000;
// Maximum height of the users
const int HEAD_HEIGHT_MAX = 2400;
// The height of the desk (This separate objects and users)
const int DESK_HEIGHT = 700;
// The length of the user's shoulder [mm]
const int SHOULDER_LENGTH = 280;
// The lenth of the user's head [mm]
const int HEAD_LENGTH = 220;

// 手を検出するための, 頭を中心とした球の半径 [mm]
const float SENCIG_CIRCLE_RADIUS = 0.6f;

// 書く座標変換行列
const Mat T_KinectCameraToWorld = (cv::Mat_<float>(4,4) <<  
	0, 1, 0, 3.4,
	0, 0, 1, -0.30,
	1, 0, 0, -2.4,
	0, 0, 0, 1);
const Mat T_WorldToScreen = (cv::Mat_<float>(4, 4) << 
	1, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, 1, 0,
	0, 0, 0, 1);

///
/// </Settings>
///

#define COLOR_IMAGE_WINDOW_NAME "rgbImage"
#define DEPTH_IMAGE_WINDOW_NAME "depthImage"

#ifdef NEAR_MODE
const int SENCEING_MIN = 400;		// 深度画像に表示する最小距離[mm]
const int SENCEING_MAX = 3000;		// 深度画像に表示する最大距離[mm]
#else
const int SENCEING_MIN = 800;		// 深度画像に表示する最小距離[mm]
const int SENCEING_MAX = 4000;		// 深度画像に表示する最大距離[mm]
#endif

#define ERROR_CHECK( ret )										\
  if ( ret != S_OK ) {											\
    std::stringstream ss;										\
    ss << "failed " #ret " " << std::hex << ret << std::endl;	\
    throw std::runtime_error( ss.str().c_str() );				\
  }

class MultiCursorAppCpp
{
public:
	MultiCursorAppCpp();
	~MultiCursorAppCpp();

	void initialize();
	void run();

private:
	// Handles
	INuiSensor* kinect;
	HANDLE imageStreamHandle;
	HANDLE depthStreamHandle;
	HANDLE streamEvent;

	// Window size (depth and color)
	DWORD width;
	DWORD height;

	/* Each pixel or 3D point data */
	Mat	userAreaMat;	// Areas of each users
	Mat point3fMatrix;	// 3D points of the observed points
	Mat heightMatrix;	// Heights of each pixel from the floor

	Mat rgbImage;		// A image from kinect color camera

	// Data for each user
	typedef struct {
		INT headHeight;

		INT headX2d;
		INT headY2d;

		FLOAT headX3d;
		FLOAT headY3d;
		FLOAT headZ3d;

		FLOAT handX3d;
		FLOAT handY3d;
		FLOAT handZ3d;
	} UserData;

	UserData *userData;

	/* Functions */
	void createInstance();
	void getFrameData();
	void getDepthImage();
	void getRgbImage();
	
	CvBlobs labelingUserArea(Mat& mat);
	void detectHeadPosition(CvBlobs blobs);
	void detectHandPosition(CvBlobs blobs);
	void drawCursor(CvBlobs blobs);

	void MouseControl(Point cursor, int id);


};