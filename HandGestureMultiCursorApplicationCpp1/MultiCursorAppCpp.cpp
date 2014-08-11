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
	const NUI_IMAGE_RESOLUTION KINECT_RGB_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;
	ERROR_CHECK(kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, KINECT_RGB_RESOLUTION,
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
		CvBlobs blobs = labelingUserArea(userAreaMat);

		/* 3. Detect users' head postiions */
		detectHeadPosition(blobs);

		/* 4. Detect users' hand positions */
		detectHandPosition(blobs);

		/* 5. Draw cursors position */
		drawCursor(blobs);

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
		Vector4 realPoint = NuiTransformDepthImageToSkeleton(depthX, depthY, distance << 3, KINECT_RESOLUTION);
		point3fMatrix.at<Vec3f>(depthY, depthX)[0] = realPoint.x;
		point3fMatrix.at<Vec3f>(depthY, depthX)[1] = realPoint.y;
		point3fMatrix.at<Vec3f>(depthY, depthX)[2] = realPoint.z;
	}

	// Release each data
	ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(depthStreamHandle, &depthFrame));
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

	// Debug: Show rgb image
	imshow(COLOR_IMAGE_WINDOW_NAME, rgbImage);
}

CvBlobs MultiCursorAppCpp::labelingUserArea(Mat& src)
{

	// Make image dilating for stable labeling
	dilate(src, src, Mat(), Point(-1, -1), 3);

	/* Use IplImage (Labeling for Mat is not fully implemented) */
	// Convert to IplImage
	IplImage srcIpl = (IplImage)src;
	// Convert to gray scale
	IplImage* srcIplBinary = cvCreateImage(cvGetSize(&srcIpl), IPL_DEPTH_8U, 1);
	cvCvtColor(&srcIpl, srcIplBinary, CV_BGR2GRAY);
	// Get binary image
	cvThreshold(srcIplBinary, srcIplBinary, 100, 255, CV_THRESH_BINARY);
	// Get blobs
	IplImage* labelImg = cvCreateImage(cvGetSize(srcIplBinary), IPL_DEPTH_LABEL, 1);

	CvBlobs blobs;
	UINT result = cvLabel(srcIplBinary, labelImg, blobs);

	// Filter small label
	cvFilterByArea(blobs, 1000, 1000000);

	// Render blobs
	cvRenderBlobs(labelImg, blobs, &srcIpl, &srcIpl);

	// Free unused IplImages
	cvReleaseImage(&labelImg);
	cvReleaseImage(&srcIplBinary);

	return blobs;
}

void MultiCursorAppCpp::detectHeadPosition(CvBlobs blobs)
{
	// Prepare list of data
	userData = new UserData[blobs.size()];

	INT blobID = 0;
	// Find the highest point of each user area
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it) {
		for (int y = it->second->miny; y <= it->second->maxy; y++) {
			for (int x = it->second->minx; x <= it->second->maxx; x++) {
				if (0 <= blobID && blobID < blobs.size()) {
					if (userData[blobID].headHeight < heightMatrix.at<USHORT>(y, x) && heightMatrix.at<USHORT>(y, x) < HEAD_HEIGHT_MAX) {
						userData[blobID].headHeight = heightMatrix.at<USHORT>(y, x);
						userData[blobID].headX2d = x;
						userData[blobID].headY2d = y;
					}

				}
			}
		}
		// Debug: Show the highest point of each users
		//circle(userAreaMat, Point(userData[blobID].headX2d, userData[blobID].headY2d), 5, Scalar(255, 0, 255), 2);

		blobID++;
	}

	// Define users' head positions
	Point2i* headPositions = new Point2i[blobs.size()];
	INT* numHeadPoints = new INT[blobs.size()];
	blobID = 0;
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it) {
		numHeadPoints[blobID] = 0;
		for (int y = it->second->miny; y <= it->second->maxy; y++) {
			for (int x = it->second->minx; x <= it->second->maxx; x++) {
				if (0 <= blobID && blobID < blobs.size()) {
					if ((userData[blobID].headHeight - HEAD_LENGTH) < heightMatrix.at<USHORT>(y, x) && heightMatrix.at<USHORT>(y, x) < HEAD_HEIGHT_MAX) {
						headPositions[blobID].x += x;
						headPositions[blobID].y += y;
						numHeadPoints[blobID]++;
					}
				}
			}
		}
		blobID++;
	}

	// Make avarage pixel value of each head positions the head positions of users
	for (int i = 0; i < blobs.size(); i++)
	{
		if (numHeadPoints[i] != 0)
		{
			// Set head position in 2D pixel
			userData[i].headX2d = headPositions[i].x / numHeadPoints[i];
			userData[i].headY2d = headPositions[i].y / numHeadPoints[i];
			// Set head position in 3D point
			userData[i].headX3d = point3fMatrix.at<Vec3f>(userData[i].headY2d, userData[i].headX2d)[0];
			userData[i].headY3d = point3fMatrix.at<Vec3f>(userData[i].headY2d, userData[i].headX2d)[1];
			userData[i].headZ3d = point3fMatrix.at<Vec3f>(userData[i].headY2d, userData[i].headX2d)[2];

			// Debug: Show the head point
			circle(userAreaMat, Point(userData[i].headX2d, userData[i].headY2d), 7, Scalar(255, 0, 0), 3);
		}
	}
}

void MultiCursorAppCpp::detectHandPosition(CvBlobs blobs)
{
	FLOAT offset = 0.001f;
	INT blobID = 0;
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it) {
		int numIntersectionPoints = 0;
		Vector4 handPosition;
		handPosition.w = 1;
		handPosition.x = 0.0f;
		handPosition.y = 0.0f;
		handPosition.z = 0.0f;
		Point3f center3f = Point3_<FLOAT>(userData[blobID].headX3d, userData[blobID].headY3d, userData[blobID].headZ3d);

		for (int y = it->second->miny; y <= it->second->maxy; y++) {
			for (int x = it->second->minx; x <= it->second->maxx; x++)
			{
				float length = sqrt(
					pow(center3f.x - point3fMatrix.at<Vec3f>(y, x)[0], 2)
					+ pow(center3f.y - point3fMatrix.at<Vec3f>(y, x)[1], 2)
					+ pow(center3f.z - point3fMatrix.at<Vec3f>(y, x)[2], 2)
					);
				// Define the intersection point of the sphere which its center is head and the hand as the hand position 
				if (SENCIG_CIRCLE_RADIUS - offset < length && length < SENCIG_CIRCLE_RADIUS + offset
					&& heightMatrix.at<USHORT>(y, x) > userData[blobID].headHeight - HEAD_LENGTH - SHOULDER_LENGTH) // Don't include desk
				{
					handPosition.x += point3fMatrix.at<Vec3f>(y, x)[0];
					handPosition.y += point3fMatrix.at<Vec3f>(y, x)[1];
					handPosition.z += point3fMatrix.at<Vec3f>(y, x)[2];

					circle(userAreaMat, Point(x, y), 3, Scalar(255, 255, 0), -1);
					numIntersectionPoints++;
				}
			}
		}

		if (numIntersectionPoints > 0) {
			handPosition.x /= numIntersectionPoints;
			handPosition.y /= numIntersectionPoints;
			handPosition.z /= numIntersectionPoints;

			userData[blobID].handX3d = handPosition.x;
			userData[blobID].handY3d = handPosition.y;
			userData[blobID].handZ3d = handPosition.z;

			// Show the hand positions on the depth image
			LONG handPositionX2d;
			LONG handPositionY2d;
			USHORT dis;
			NuiTransformSkeletonToDepthImage(handPosition, &handPositionX2d, &handPositionY2d, &dis, KINECT_RESOLUTION);
			circle(userAreaMat, Point(handPositionX2d, handPositionY2d), 7, Scalar(0, 200, 0), 3);
		}
		else
		{
			userData[blobID].handX3d = 0.0f;
			userData[blobID].handY3d = 0.0f;
			userData[blobID].handZ3d = 0.0f;
		}
		blobID++;
	}
	// Debug: Show depth image
	imshow(DEPTH_IMAGE_WINDOW_NAME, userAreaMat);
}

void MultiCursorAppCpp::drawCursor(CvBlobs blobs)
{
	INT blobID = 0;
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it) {
		Mat handPoint = (cv::Mat_<float>(4, 1) << userData[blobID].handX3d, userData[blobID].handY3d, userData[blobID].handZ3d, 1);
		Mat headPoint = (cv::Mat_<float>(4, 1) << userData[blobID].headX3d, userData[blobID].headY3d, userData[blobID].headZ3d, 1);

		Mat handPointScreen = T_WorldToScreen * T_KinectCameraToWorld * handPoint;
		Mat headPointScreen = T_WorldToScreen * T_KinectCameraToWorld * headPoint;

		// Caliculate the intersection point of vector and screen
		float xvec = handPointScreen.at<float>(0, 0) - headPointScreen.at<float>(0, 0);
		float yvec = handPointScreen.at<float>(1, 0) - headPointScreen.at<float>(1, 0);
		float zvec = handPointScreen.at<float>(2, 0) - headPointScreen.at<float>(2, 0);

		float val = -handPointScreen.at<float>(2, 0) / zvec;

		// Calculate cursor position in real scall
		Point3f cursorScreen3d;
		cursorScreen3d.x = val * xvec + headPointScreen.at<float>(0, 0);
		cursorScreen3d.y = val * yvec + headPointScreen.at<float>(1, 0);
		cursorScreen3d.z = 0.0f;
		// Calculate cursor position in pixel coordinate
		float screen3dTo2d = 246 / 0.432f;
		Point cursorScreen2d;
		cursorScreen2d.x = (int)(cursorScreen3d.x * screen3dTo2d);
		cursorScreen2d.y = (int)(cursorScreen3d.y * screen3dTo2d);

		// Set the mouse cursor
		MouseControl(cursorScreen2d, blobID);
	}
}

void MultiCursorAppCpp::MouseControl(Point cursor, int id)
{
	cout << cursor.x << ", " << cursor.y << endl;
	// スクリーン座標をmouse_event()用の座標変換
	DWORD dwX = cursor.x * 65535 / ::GetSystemMetrics(SM_CXSCREEN);
	DWORD dwY = cursor.y * 65535 / ::GetSystemMetrics(SM_CYSCREEN);

	// Set mouse cursor position
	::mouse_event(MOUSEEVENTF_ABSOLUTE | MOUSEEVENTF_MOVE, dwX, dwY, NULL, NULL);
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