#pragma once

///////////////////////////////////////////////////////////////////
/// <Settings> �����\�ȃp�����[�^
/////////////////////////////////////////////////////////////////// 

//#define USE_KINECT_V1		// Kinect v1��p����ꍇ�̓R�����g���O��
#define NEAR_MODE		// near���[�h���g���ꍇ�̓R�����g���O��(Kinect v1�̂�)

#define TRACK_GESTURE_BY_AREA	// �����define�������͎�̈�̐��K�������ʐςɉ����ăW�F�X�`�����肷��

// #define USE_COLOR_V2






// The height of Kinect which is set on the celling
const int KINECT_HEIGHT = 2700;		// �~�[�e�B���O���[���p
//const int KINECT_HEIGHT = 1800;		// ��Ə�p

// The threshold which detect users in the first step (The distance from the floor)
// �ŏ��Ƀ��[�U�����o���鎞�̂������l(�n�ʂ���̍���)[mm]
const int USER_HEIGHT_THRESHOLD = 900;	// �~�[�e�B���O���[���p
//const int USER_HEIGHT_THRESHOLD = 800;	// ��Ə�p

// Maximum height of the users
const int HEAD_HEIGHT_MAX = 2400;

// The height of the desk (This separate objects and users)
const int DESK_HEIGHT = KINECT_HEIGHT - 1.9944671684835250e+003;

// The length of the user's shoulder [mm]
const int SHOULDER_LENGTH = 300;

// The lenth of the user's head [mm]
const int HEAD_LENGTH = 150;

// ������o���邽�߂�, ���𒆐S�Ƃ������̔��a [mm]
const float SENCIG_CIRCLE_RADIUS = 0.45;

// �e���W�ϊ��s��
const static vector<char*> DISP_INFO_FILENAMES = { 
		{"calibData/DispInfo1.xml"}
		//{"calibData/DispInfo2.xml"}
};


#ifdef USE_KINECT_V1
// The resolution of the kinect depth camera
const NUI_IMAGE_RESOLUTION KINECT_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;

const static Mat T_KinectCameraToWorld = (cv::Mat_<float>(4,4) <<  
	0, 1, 0, 3.4,
	0, 0, 1, -0.30,
	1, 0, 0, -2.4,
	0, 0, 0, 1);
const static Mat T_WorldToScreen = (cv::Mat_<float>(4, 4) << 
	1, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, 1, 0,
	0, 0, 0, 1);
#endif

///////////////////////////////////////////////////////////////////
/// </Settings>
///////////////////////////////////////////////////////////////////

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

	// ���C�����[�v�֐�
	void run();

	// �eKinect�������֐�
	void initKinectV1();
	void initKinectV2();

	// ���W�ϊ��s��̓ǂݍ���
	void loadCalibData();

	// GL������
	void initGL(int argc, char* argv[]);

	// OpenGL callback function
	void display(void);
	void reshape(int w, int h);
	void idle(void);
	void keyboard(unsigned char key, int x, int y);
	void mouse(int button, int state, int mouse_x, int mouse_y);

	void showHelp();

private:
	// Screen resolution
	vector<int> VEC_WIN_WIDTH;
	vector<int> VEC_WIN_HEIGHT;

#ifdef USE_KINECT_V1
	// Window size (depth)
	int CAMERA_WIDTH;
	int CAMERA_HEIGHT;
#endif
	// Each pixel or 3D point data
	Mat	userAreaMat;	// Areas of each users
	Mat point3fMatrix;	// 3D points of the observed points
	Mat heightMatrix;	// Heights of each pixel from the floor
	Mat labelMat;		// Label of each pixels
	Mat preLabelMat;	// Label of each pixels in pre-frame

	Mat depthImage;		// Image from kinect depth camera
	Mat rgbImage;		// Image from kinect color camera

	// ���W�ϊ��s��
	vector<Mat> TKinect2Display;
	vector<Mat> TDisplay2Pixel;
	vector<int> windowOffsetX;	// �}���`�f�B�X�v���C�\���̍ہC���̃f�B�X�v���C���l���������W�l�����߂邽�߂Ɏg��

	// ���Ɋւ�����
	typedef struct {
		Point2i depthPoint;
		Point3f cameraPoint;

		int height;
	} HeadInfo;

	// ��Ɋւ�����
	typedef struct {
		Point3f cameraPoint;
		float area;
		bool isTracked;
	} HandInfo;

	// �J�[�\���Ɋւ�����
	typedef struct {
		int displayNum;
		Point2f position;
		bool isShownCursor;
		bool isClicking;
	} CursorInfo;
	

	// Data for each user
	typedef struct {
		
		bool isDataFound;	// �O�t���[���̃f�[�^�Ƃ��ĎQ�Ƃ���Ƃ��Ή�����blob�������������ǂ���

		HeadInfo headInfo;

		HandInfo handInfo;

		// �d�S
		Point2i centroid;
		unsigned long labelID;

		CursorInfo cursorInfo;

	} UserData;

	//UserData userData;
	vector<UserData> userData;
	vector<UserData> preUserData;

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
	void detectHandGesture(CvBlobs blobs);

	/* For showing results */
	bool isShowDebugWindows = true;
	void showDebugWindows();


	/* OpenGL */
	int* WinIDs;

	void MouseControl(float x, float y);

};

// �N���X�錾
extern MultiCursorAppCpp app;
extern KinectV2Basics kinectBasics;

/* Kinect_v2 */
// Sensor
//static IKinectSensor* pSensor;
//// Reader: Depth data��ۊǂ���X�g���[��
//static IDepthFrameReader* pDepthReader;
//// CoordinateMapper
//static ICoordinateMapper* pCoordinateMapper;

#ifdef USE_COLOR_V2
// Reader: Color data��ۊǂ���X�g���[��
static IColorFrameReader* pColorReader;
#endif

void sdisplay();
void sreshape(int w, int h);
void sidle(void);
void skeyboard(unsigned char key, int x, int y);
void smouse(int button, int state, int mouse_x, int mouse_y);