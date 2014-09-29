#pragma once

///
/// <Settings> �����\�ȃp�����[�^
/// 

//#define USE_KINECT_V1		// Kinect v1��p����ꍇ�̓R�����g���O��
#define NEAR_MODE		// near���[�h���g���ꍇ�̓R�����g���O��(Kinect v1�̂�)

// Screen resolution
const int WINDOW_WIDTH  = 2560;
const int WINDOW_HEIGHT = 1024;

// The resolution of the kinect depth camera
const NUI_IMAGE_RESOLUTION KINECT_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;

// The height of Kinect which is set on the celling
//const int KINECT_HEIGHT = 2700;		// �~�[�e�B���O���[���p
const int KINECT_HEIGHT = 1800;		// ��Ə�p

// The threshold which detect users in the first step (The distance from the floor)
// �ŏ��Ƀ��[�U�����o���鎞�̂������l(�n�ʂ���̍���)[mm]
//const int USER_HEIGHT_THRESHOLD = 1000;
const int USER_HEIGHT_THRESHOLD = 800;

// Maximum height of the users
const int HEAD_HEIGHT_MAX = 2400;

// The height of the desk (This separate objects and users)
const int DESK_HEIGHT = 700;

// The length of the user's shoulder [mm]
const int SHOULDER_LENGTH = 100;

// The lenth of the user's head [mm]
const int HEAD_LENGTH = 220;

// ������o���邽�߂�, ���𒆐S�Ƃ������̔��a [mm]
const float SENCIG_CIRCLE_RADIUS = 0.55f;

// �������W�ϊ��s��
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
const int SENCEING_MIN = 400;		// �[�x�摜�ɕ\������ŏ�����[mm]
const int SENCEING_MAX = 3000;		// �[�x�摜�ɕ\������ő勗��[mm]
#else
const int SENCEING_MIN = 800;		// �[�x�摜�ɕ\������ŏ�����[mm]
const int SENCEING_MAX = 4000;		// �[�x�摜�ɕ\������ő勗��[mm]
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

	void run();

	void initKinectV1();
	void initKinectV2();

	void initGL(int argc, char* argv[]);
	void runGL();

	// OpenGL callback function
	void display(void);
	void reshape(int w, int h);
	void idle(void);
	void keyboard(unsigned char key, int x, int y);
	//void mouse(int button, int state, int mouse_x, int mouse_y);

	void showHelp();

private:
	// Window size (depth and color)
	int CAMERA_WIDTH;
	int CAMERA_HEIGHT;

	/* Each pixel or 3D point data */
	Mat	userAreaMat;	// Areas of each users
	Mat point3fMatrix;	// 3D points of the observed points
	Mat heightMatrix;	// Heights of each pixel from the floor

	Mat depthImage;		// Image from kinect depth camera
	Mat rgbImage;		// Image from kinect color camera

	// Data for each user
	typedef struct {
		bool isDataFound;	// �O�t���[���̃f�[�^�Ƃ��ĎQ�Ƃ���Ƃ��Ή�����blob�������������ǂ���

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

	/* Handles for kinect v1 */
	INuiSensor* kinect;
	HANDLE imageStreamHandle;
	HANDLE depthStreamHandle;

	HANDLE streamEvent;
	/* Functions for kinect v1 */
	void createInstance();
	bool getDepthImageV1();
	void getRgbImageV1();

	/* Functions for kinect v2*/
	bool getDepthImageV2();
	void getRgbImageV2();

	/* Other Functions */
	bool getFrameData();
	CvBlobs labelingUserArea(Mat& mat);
	void detectHeadPosition(CvBlobs blobs);
	void detectHandPosition(CvBlobs blobs);
	void setCursor(CvBlobs blobs);

	/* For showing results */
	bool isShowDebugWindows = true;
	void showDebugWindows();


	void MouseControl(float x, float y);
};

// �N���X�錾
extern MultiCursorAppCpp app;

/* Kinect_v2 */
// Sensor
static IKinectSensor* pSensor;
// Reader: Depth data��ۊǂ���X�g���[��
static IDepthFrameReader* pDepthReader;
// CoordinateMapper
static ICoordinateMapper* pCoordinateMapper;
// Reader: Color data��ۊǂ���X�g���[��
static IColorFrameReader* pColorReader;

void sdisplay();
void sreshape(int w, int h);
void sidle(void);
void skeyboard(unsigned char key, int x, int y);
//void smouse(int button, int state, int mouse_x, int mouse_y)