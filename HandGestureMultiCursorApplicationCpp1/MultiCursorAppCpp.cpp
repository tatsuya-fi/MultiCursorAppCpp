// HandGestureMultiCursorApplicationCpp1.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "stdafx.h"
#include "MultiCursorAppCpp.h"

MultiCursorAppCpp::MultiCursorAppCpp()
{
}

MultiCursorAppCpp::~MultiCursorAppCpp()
{
	// 終了処理
	if (kinect != 0) {
		kinect->NuiShutdown();
		kinect->Release();
	}
}

void MultiCursorAppCpp::initialize()
{
	createInstance();

	// Kinectの設定を初期化する
	ERROR_CHECK(kinect->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH));

	// RGBカメラを初期化する
	ERROR_CHECK(kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, KINECT_RESOLUTION,
		0, 2, 0, &imageStreamHandle));

	// 距離カメラを初期化する
	ERROR_CHECK(kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, KINECT_RESOLUTION,
		0, 2, 0, &depthStreamHandle));

#ifdef NEAR_MODE
	// Nearモード
	ERROR_CHECK(kinect->NuiImageStreamSetImageFrameFlags(
		depthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE));
#endif

	// フレーム更新イベントのハンドルを作成する
	streamEvent = ::CreateEvent(0, TRUE, FALSE, 0);
	ERROR_CHECK(kinect->NuiSetFrameEndEvent(streamEvent, 0));

	// 指定した解像度の、画面サイズを取得する
	::NuiImageResolutionToSize(KINECT_RESOLUTION, width, height);
}

void MultiCursorAppCpp::run()
{
	// Main loop
	while (1) {
		/* 1. Get frame data and prepear data needed */
		getFrameData();

		/* 2. Detect users' head positions */
		CvBlobs blobs = labelingUserArea();

		

		// Key check for quit
		int key = waitKey(10);
		if (key == 'q' || key == KEY_ESC) {
			destroyAllWindows();
			break;
		}
	}
}


void MultiCursorAppCpp::createInstance()
{
	// 接続されているKinectの数を取得する
	int count = 0;
	ERROR_CHECK(::NuiGetSensorCount(&count));
	if (count == 0) {
		throw std::runtime_error("Kinect を接続してください");
	}

	// 最初のKinectのインスタンスを作成する
	ERROR_CHECK(::NuiCreateSensorByIndex(0, &kinect));

	// Kinectの状態を取得する
	HRESULT status = kinect->NuiStatus();
	if (status != S_OK) {
		throw std::runtime_error("Kinect が利用可能ではありません");
	}
}

void MultiCursorAppCpp::getFrameData()
{
	// Wait for updating frame data
	DWORD ret = ::WaitForSingleObject(streamEvent, INFINITE);
	::ResetEvent(streamEvent);
		
	// Get depth image
	getDepthImage();

	// Get color image
	getRgbImage();
}

void MultiCursorAppCpp::getDepthImage()
{
	// Initialize matrix for each data
	userAreaMat = Mat::zeros(height, width, CV_8UC3);
	point3fMatrix = Mat::zeros(height, width, CV_32FC3);
	heightMatrix = Mat::zeros(height, width, CV_16U);


	// Get the frame data of the depth camera
	NUI_IMAGE_FRAME depthFrame = { 0 };
	ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(depthStreamHandle, INFINITE, &depthFrame));

	// Get the actual depth data
	NUI_LOCKED_RECT depthData = { 0 };
	depthFrame.pFrameTexture->LockRect(0, &depthData, 0, 0);

	USHORT* depth = (USHORT*)depthData.pBits;
	for (int i = 0; i < (depthData.size / sizeof(USHORT)); ++i) {
		USHORT distance = ::NuiDepthPixelToDepth(depth[i]);

		LONG depthX = i % width;
		LONG depthY = i / width;

		int index = ((depthY * width) + depthX) * 3;
		UCHAR* dataDepth = &userAreaMat.data[index];
		USHORT heightFromFloor;
		distance == 0 ? heightFromFloor = 0 : heightFromFloor = KINECT_HEIGHT - distance;

		// 高さ情報を記録 / Set the height from floor
		heightMatrix.at<USHORT>(depthY, depthX) = heightFromFloor;

		// ユーザ領域を記憶 / Define user area
		if (USER_HEIGHT_THRESHOLD <= heightFromFloor && heightFromFloor <= HEAD_HEIGHT_MAX) {
			dataDepth[0] = 255;
			dataDepth[1] = 255;
			dataDepth[2] = 255;
		}
		else {
			dataDepth[0] = 0;
			dataDepth[1] = 0;
			dataDepth[2] = 0;
		}

		// ポイントクラウドを記憶 / Set 3D point data
		Vector4 realPoint = NuiTransformDepthImageToSkeleton(depthX, depthY, distance, KINECT_RESOLUTION);
		point3fMatrix.at<Vec3f>(depthY, depthX)[0] = realPoint.x;
		point3fMatrix.at<Vec3f>(depthY, depthX)[1] = realPoint.y;
		point3fMatrix.at<Vec3f>(depthY, depthX)[2] = realPoint.z;
	}

	// Release each data
	ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(depthStreamHandle, &depthFrame));

	// Debug: Show depth image
	//imshow(DEPTH_IMAGE_WINDOW_NAME, userAreaMat);
}

void MultiCursorAppCpp::getRgbImage()
{
	// RGBカメラのフレームデータを取得する
	NUI_IMAGE_FRAME imageFrame = { 0 };
	ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(imageStreamHandle, INFINITE, &imageFrame));

	// 画像データを取得する
	NUI_LOCKED_RECT colorData;
	imageFrame.pFrameTexture->LockRect(0, &colorData, 0, 0);

	// 画像データをコピーする
	rgbImage = cv::Mat(height, width, CV_8UC4, colorData.pBits);

	// フレームデータを解放する
	ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(imageStreamHandle, &imageFrame));

	imshow(COLOR_IMAGE_WINDOW_NAME, rgbImage);
}

CvBlobs MultiCursorAppCpp::labelingUserArea()
{

	// Make image dilating for stable labeling
	dilate(userAreaMat, userAreaMat, Mat(), Point(-1, -1), 3);

	/* Use IplImage (Labeling for Mat is not fully implemented) */
	// Convert to IplImage
	IplImage srcIpl = (IplImage)userAreaMat;
	// Convert to gray scale
	IplImage *srcIplBinary = cvCreateImage(cvGetSize(&srcIpl), IPL_DEPTH_8U, 1);
	cvCvtColor(&srcIpl, srcIplBinary, CV_BGR2GRAY);
	// Get binary image
	cvThreshold(srcIplBinary, srcIplBinary, 100, 255, CV_THRESH_BINARY);
	// Get blobs
	IplImage *labelImg = cvCreateImage(cvGetSize(srcIplBinary), IPL_DEPTH_LABEL, 1);

	CvBlobs blobs;
	UINT result = cvLabel(srcIplBinary, labelImg, blobs);

	cvRenderBlobs(labelImg, blobs, &srcIpl, &srcIpl);

	// Free unused IplImages
	cvReleaseImage(&labelImg);
	cvReleaseImage(&srcIplBinary);

	// Debug: Show depth image
	imshow(DEPTH_IMAGE_WINDOW_NAME, userAreaMat);

	return blobs;
}


void main()
{

	try {
		MultiCursorAppCpp app;
		app.initialize();
		app.run();
		exit(0);
	}
	catch (std::exception& ex) {
		std::cout << ex.what() << std::endl;
	}
}