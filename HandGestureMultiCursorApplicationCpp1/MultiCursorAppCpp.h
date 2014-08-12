#pragma once

///
/// <Settings> 調整可能なパラメータ
/// 

#define NEAR_MODE		// nearモードを使う場合はコメントを外す

// Screen resolution
const int WINDOW_WIDTH  = 2560;
const int WINDOW_HEIGHT = 1024;

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
const int SHOULDER_LENGTH = 100;
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

#define COLOR_IMAGE_WINDOW_NAME "RBG Image"
#define DEPTH_IMAGE_WINDOW_NAME "Depth Image"

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

	void initKinect();
	void run();
	void runGL();

	// OpenGL callback function
	void display(void);
	void reshape(int w, int h);
	void idle(void);
	void keyboard(unsigned char key, int x, int y);
	//void mouse(int button, int state, int mouse_x, int mouse_y);

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
		bool isDataFound;	// 前フレームのデータとして参照するとき対応するblobが見つかったかどうか

		int headHeight;

		Point2i head2i;
		Point3f head3f;

		Point3f hand3f;

		int centroidX;
		int centroidY;

		/* Cursor params */
		Point2f cursorPos;
		bool isShownCursor;
		bool isClicking;
	
	} UserData;

	//UserData userData;
	vector<UserData> userData;
	vector<UserData> preUserData;
	vector<UserData>::iterator p;
	
	/* Functions */
	void createInstance();
	void getFrameData();
	void getDepthImage();
	void getRgbImage();
	
	CvBlobs labelingUserArea(Mat& mat);
	void detectHeadPosition(CvBlobs blobs);
	void detectHandPosition(CvBlobs blobs);
	void setCursor(CvBlobs blobs);

	void MouseControl(float x, float y);
};

// GLにコールバック関数を登録する用
static MultiCursorAppCpp appSub;

void sdisplay();
void sreshape(int w, int h);
void sidle(void);
void skeyboard(unsigned char key, int x, int y);
//void smouse(int button, int state, int mouse_x, int mouse_y)