#pragma once
class KinectV2Basics
{
	

public:
	KinectV2Basics();
	~KinectV2Basics();

	// �e�T�C�Y
	const static int widthDepth   = 512;
	const static int heightDepth  = 424;
	const static int widthColor  = 1920;
	const static int heightColor = 1080;

	// Select using data (Call before "bool SetupKinectV2()")
	void SelectUsingData(const bool isUseDepthStream, const bool isUseColorStream);

	// Set up KinectV2
	bool SetupKinectV2();

	// Getters
	IKinectSensor*		GetSensor();
	IDepthFrameSource*	GetSourceDepth();
	IDepthFrameReader*	GetReaderDepth();
	IDepthFrame*		GetFrameDepth();
	IColorFrameSource*	GetSourceColor();
	IColorFrameReader*	GetReaderColor();
	IColorFrame*		GetFrameColor();
	ICoordinateMapper*	GetMapper();
	

#ifdef OPENCV
	// outDepth8UC3:   ���t���[���̐[�x�摜(imshow()�\���p)
	// outDepth16S:    �s�N�Z�����Ƃ�Z���������i�[�����s��
	// outPoints32FC3: �[�x�J�������W�n�ɂ�����O�����_
	bool GetDepthMat(cv::Mat& outDepth16S);
	bool GetDepthMat(cv::Mat& outDepth8UC3, cv::Mat& outDepth16S);
	bool GetDepthMat(cv::Mat& outDepth8UC3, cv::Mat& outDepth16S, cv::Mat& outPoints32FC3);

	// �[�x�J�������W�n�̓_�Q�擾
	bool GetPointsMat(cv::Mat& pointsMat);

	// ���t���[���̃J���[�摜(Mat)�擾
	bool GetColorMat(cv::Mat& outColor);
	bool GetColorMat(cv::Mat& outColor, float scale);	// �摜�T�C�Y���傫���̂ŃX�P�[�������\�ɂ���
#endif

private:
	// �f�[�^�擾����܂ł̊e�|�C���^
	IKinectSensor*		pSensor;
	// Depth
	IDepthFrameSource*	pDepthSource;
	IDepthFrameReader*	pDepthReader;
	IDepthFrame*		pDepthFrame;
	// Color
	IColorFrameSource*	pColorSource;
	IColorFrameReader*	pColorReader;
	IColorFrame*		pColorFrame;
	// Mapper
	ICoordinateMapper*	pCoordinateMapper;

	// �e�f�[�^�̃I���I�t
	bool isUseDepth, isUseColor;
	

#ifdef OPENCV


#endif
};