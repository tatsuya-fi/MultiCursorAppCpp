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
#ifdef USE_KINECT_V1
	if (kinect != 0) {
		//kinect->NuiShutdown();
		kinect->Release();
	}
//#else
//	SafeRelease(pDepthReader);
//	SafeRelease(pCoordinateMapper);
//
//	if (pSensor)
//	{
//		pSensor->Close();
//	}
//	SafeRelease(pSensor);
#endif

	// CVのウィンドウ破棄（念のため)
	destroyAllWindows();
}


#pragma region KINECT V1
#ifdef USE_KINECT_V1

void MultiCursorAppCpp::createInstance()
{
	// 接続されているKinectの数を取得する
	int count = 0;
	if (S_OK != ::NuiGetSensorCount(&count)) {
		cout << "Error: NuiGetSensorCount()" << endl;
		exit(0);
	}
	if (count == 0) {
		throw std::runtime_error("Kinect を接続してください");
	}

	// 最初のKinectのインスタンスを作成する
	if (S_OK != ::NuiCreateSensorByIndex(0, &kinect)) {
		cout << "Error: NuiCreateSensorByIndex" << endl;
		exit(0);
	}

	// Kinectの状態を取得する
	HRESULT status = kinect->NuiStatus();
	if (status != S_OK) {
		throw std::runtime_error("Kinect が利用可能ではありません");
	}
}

void MultiCursorAppCpp::initKinectV1()
{
	createInstance();

	// Kinectの設定を初期化する
	if (S_OK != kinect->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH)) {
		cout << "Error: NuiInitialize() " << endl;
		exit(0);
	}

	// RGBカメラを初期化する
	const NUI_IMAGE_RESOLUTION KINECT_RGB_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;
	if (S_OK != (kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, KINECT_RGB_RESOLUTION,
		0, 2, 0, &imageStreamHandle))) {
		cout << "Error: NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR)" << endl;
		exit(0);
	}

	// 距離カメラを初期化する
	if (S_OK != (kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, KINECT_RESOLUTION,
		0, 2, 0, &depthStreamHandle))) {
		cout << "Error: NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH)" << endl;
		exit(0);
	}

#ifdef NEAR_MODE
	// Nearモード
	if (S_OK != (kinect->NuiImageStreamSetImageFrameFlags(
		depthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE))) {
		cout << "Error: NuiImageStreamSetImageFrameFlags()" << endl;
		exit(0);
	}
#endif

	// フレーム更新イベントのハンドルを作成する
	streamEvent = ::CreateEvent(0, TRUE, FALSE, 0);
	if (S_OK != (kinect->NuiSetFrameEndEvent(streamEvent, 0))) {
		cout << "Error: NuiSetFrameEndEvent()" << endl;
		exit(0);
	}

	// 指定した解像度の、画面サイズを取得する
	DWORD width, height;
	::NuiImageResolutionToSize(KINECT_RESOLUTION, width, height);
	CAMERA_WIDTH = (int)width;
	CAMERA_HEIGHT = (int)height;
}

bool MultiCursorAppCpp::getDepthImageV1()
{
	// Initialize matrix for each data
	userAreaMat = Mat::zeros(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3);
	point3fMatrix = Mat::zeros(CAMERA_HEIGHT, CAMERA_WIDTH, CV_32FC3);
	heightMatrix = Mat::zeros(CAMERA_HEIGHT, CAMERA_WIDTH, CV_16U);


	// Get the frame data of the depth camera
	NUI_IMAGE_FRAME depthFrame = { 0 };
	if (kinect->NuiImageStreamGetNextFrame(depthStreamHandle, 0, &depthFrame) < 0) {
		return false;
	}

	// Get the actual depth data
	NUI_LOCKED_RECT depthData = { 0 };
	depthFrame.pFrameTexture->LockRect(0, &depthData, 0, 0);

	USHORT* depth = (USHORT*)depthData.pBits;
	for (int i = 0; i < (depthData.size / sizeof(USHORT)); ++i) {
		USHORT distance = ::NuiDepthPixelToDepth(depth[i]);

		LONG depthX = i % CAMERA_WIDTH;
		LONG depthY = i / CAMERA_WIDTH;

		int index = ((depthY * CAMERA_WIDTH) + depthX) * 3;
		UCHAR* dataDepth = &userAreaMat.data[index];

		// 高さ情報を記録 / Set the height from floor
		USHORT heightFromFloor;
		(0 < distance && distance < KINECT_HEIGHT) ? heightFromFloor = KINECT_HEIGHT - distance : heightFromFloor = 0;
		*heightMatrix.ptr<USHORT>(depthY, depthX) = heightFromFloor;

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
		point3fMatrix.ptr<float>(depthY, depthX)[0] = realPoint.x;
		point3fMatrix.ptr<float>(depthY, depthX)[1] = realPoint.y;
		point3fMatrix.ptr<float>(depthY, depthX)[2] = realPoint.z;
	}

	// Release each data
	if (S_OK != (kinect->NuiImageStreamReleaseFrame(depthStreamHandle, &depthFrame))) {
		cout << "Error: NuiImageStreamReleaseFrame()" << endl;
		exit(0);
	}

	return true;
}

void MultiCursorAppCpp::getRgbImageV1()
{
	// RGBカメラのフレームデータを取得する
	NUI_IMAGE_FRAME imageFrame = { 0 };
	if (kinect->NuiImageStreamGetNextFrame(imageStreamHandle, 0, &imageFrame) < 0) {
		return;
	}

	// 画像データを取得する
	NUI_LOCKED_RECT colorData = { 0 };
	imageFrame.pFrameTexture->LockRect(0, &colorData, NULL, 0);

	// 画像データをコピーする
	rgbImage = cv::Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC4, colorData.pBits);

	// フレームデータを解放する
	if (S_OK != (kinect->NuiImageStreamReleaseFrame(imageStreamHandle, &imageFrame))){
		cout << "Error: NuiImageStreamReleaseFrame()" << endl;
		exit(0);
	}
}

#endif
#pragma endregion


#pragma region KINECT V2
//void MultiCursorAppCpp::initKinectV2()
//{
//	// Find Sensor
//	IKinectSensor* pSensor;
//	HRESULT hResult = S_OK;
//	hResult = GetDefaultKinectSensor(&pSensor);
//	if (FAILED(hResult))
//	{
//		cerr << "Error : GetDefaultKinectSensor()" << endl;
//		exit(0);
//	}
//
//	hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
//	if (FAILED(hResult))
//	{
//		cerr << "Error : IKinectSensor::get_CoordinateMapper()" << endl;
//		exit(0);
//	}
//
//	// Open stream of the sensor
//	hResult = pSensor->Open();
//	if (FAILED(hResult))
//	{
//		cerr << "Error : IKinectSensor::Open()" << endl;
//		exit(0);
//	}
//
//	/* Depth */
//	// Sensor to Source 
//	IDepthFrameSource* pDepthSource;
//	hResult = pSensor->get_DepthFrameSource(&pDepthSource);
//	if (FAILED(hResult))
//	{
//		cerr << "Error : IKinectSensor::get_DepthFrameSource()" << endl;
//		exit(0);
//	}
//
//	// Get depth image size
//	IFrameDescription* depthFrameDescription;
//	hResult = pDepthSource->get_FrameDescription(&depthFrameDescription);
//	if (FAILED(hResult))
//	{
//		cerr << "Error : IKinectSensor::get_FrameDescription()" << endl;
//		exit(0);
//	}
//	depthFrameDescription->get_Width(&CAMERA_WIDTH);
//	depthFrameDescription->get_Height(&CAMERA_HEIGHT);
//
//
//	// Source to Reader
//	hResult = pDepthSource->OpenReader(&pDepthReader);
//	if (FAILED(hResult))
//	{
//		cerr << "Error : IDepthFrameSource::OpenReader()" << endl;
//		exit(0);
//	}
//	SafeRelease(pDepthSource);
//
//#ifdef USE_COLOR_V2
//	/* Color */
//	// Sensor to Source
//	IColorFrameSource* pColorSource;
//	hResult = pSensor->get_ColorFrameSource(&pColorSource);
//	if (FAILED(hResult)){
//		cerr << "Error : IKinectSensor::get_ColorFrameSource()" << endl;
//		exit(0);
//	}
//
//	// Reader
//	hResult = pColorSource->OpenReader(&pColorReader);
//	if (FAILED(hResult)){
//		std::cerr << "Error : IColorFrameSource::OpenReader()" << std::endl;
//		exit(0);
//	}
//#endif
//}

bool MultiCursorAppCpp::getDepthImageV2()
{
	// Get depth frame data
	if (!kinectBasics.GetDepthMat(depthImage, heightMatrix, point3fMatrix)) { return false; }
	// init
	userAreaMat = Mat::zeros(kinectBasics.heightDepth, kinectBasics.widthDepth, CV_8UC3);

	for (int y = 0; y < kinectBasics.heightDepth; ++y)
	{
		for (int x = 0; x < kinectBasics.widthDepth; ++x)
		{
			// 床からの高さを求める
			USHORT distance = *heightMatrix.ptr<USHORT>(y, x);
			// 机より高い点のみユーザエリアとして記録
			USHORT offset = 200;
			if (0 < distance && distance < (USHORT)*TKinect2Marker.ptr<float>(2, 3) - offset)
			{
				// 床からの高さ
				*heightMatrix.ptr<USHORT>(y, x) = (USHORT)(KINECT_HEIGHT - distance);
				// ユーザエリア
				//userAreaMat.ptr<Vec3b>(y, x)[0] = 255;
				//userAreaMat.ptr<Vec3b>(y, x)[1] = 255;
				//userAreaMat.ptr<Vec3b>(y, x)[2] = 255;
				int index = ((y * kinectBasics.widthDepth) + x) * 3;
				UCHAR* dataDepth = &userAreaMat.data[index];
				dataDepth[0] = 255;
				dataDepth[1] = 255;
				dataDepth[2] = 255;
			}
			else
			{
				*heightMatrix.ptr<USHORT>(y, x) = 0;
			}
		}
	}


	// Initialize matrix for each data
	//userAreaMat = Mat::zeros(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC3);
	//point3fMatrix = Mat::zeros(CAMERA_HEIGHT, CAMERA_WIDTH, CV_32FC3);
	//heightMatrix = Mat::zeros(CAMERA_HEIGHT, CAMERA_WIDTH, CV_16UC1);

	//// センサからのフレーム情報を一旦保管する
	//unsigned int bufferSize = CAMERA_WIDTH * CAMERA_HEIGHT * sizeof(unsigned short);
	//Mat bufferMat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_16SC1);
	//depthImage = Mat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1);
	//Mat depthMat(CAMERA_HEIGHT, CAMERA_WIDTH, CV_8UC1);

	//UINT16 *pDepthBuffer = NULL;

	//// Frame
	//IDepthFrame* pDepthFrame = nullptr;
	//HRESULT hResult = S_OK, hResult2 = S_OK;
	//pDepthReader != NULL ? hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame) : hResult = E_POINTER;
	//if (SUCCEEDED(hResult))
	//{
	//	hResult = pDepthFrame->AccessUnderlyingBuffer(&bufferSize, reinterpret_cast<UINT16**>(&bufferMat.data));
	//	hResult2 = pDepthFrame->AccessUnderlyingBuffer(&bufferSize, &pDepthBuffer);
	//	if (SUCCEEDED(hResult) && SUCCEEDED(hResult))
	//	{
	//		bufferMat.convertTo(depthImage, CV_8U, -255.0f / 4500.0f, 255.0f);
	//	}

	//	for (int y = 0; y < CAMERA_HEIGHT; y++)
	//	{
	//		for (int x = 0; x < CAMERA_WIDTH; x++)
	//		{
	//			// Set heights from the floor
	//			short distance = *bufferMat.ptr<short>(y, x);
	//			short heightFromFloor;
	//			(0 < distance && distance < KINECT_HEIGHT) ? heightFromFloor = KINECT_HEIGHT - distance : heightFromFloor = 0;
	//			*heightMatrix.ptr<USHORT>(y, x) = (USHORT)heightFromFloor;

	//			// Set user area
	//			int index = ((y * CAMERA_WIDTH) + x) * 3;
	//			UCHAR* dataDepth = &userAreaMat.data[index];
	//			if (USER_HEIGHT_THRESHOLD <= heightFromFloor && heightFromFloor <= HEAD_HEIGHT_MAX) {
	//				dataDepth[0] = 255;
	//				dataDepth[1] = 255;
	//				dataDepth[2] = 255;
	//			}
	//			else {
	//				dataDepth[0] = 0;
	//				dataDepth[1] = 0;
	//				dataDepth[2] = 0;
	//			}
	//		}
	//	}

	//	// Set point cloud
	//	UINT pointCount = CAMERA_WIDTH * CAMERA_HEIGHT;
	//	CameraSpacePoint* cameraSpacePoint = new CameraSpacePoint[CAMERA_WIDTH * CAMERA_HEIGHT];	// Points in camera coordinate [m]
	//	hResult = pCoordinateMapper->MapDepthFrameToCameraSpace(pointCount, pDepthBuffer, pointCount, cameraSpacePoint);
	//	if (!SUCCEEDED(hResult))
	//	{
	//		cout << "Error: MapDepthFrameToCameraSpace()" << endl;
	//	}
	//	for (int y = 0; y < CAMERA_HEIGHT; ++y)
	//	{
	//		for (int x = 0; x < CAMERA_WIDTH; ++x)
	//		{
	//			int index = y * CAMERA_WIDTH + x;
	//			point3fMatrix.ptr<float>(y, x)[0] = cameraSpacePoint[index].X;
	//			point3fMatrix.ptr<float>(y, x)[1] = cameraSpacePoint[index].Y;
	//			point3fMatrix.ptr<float>(y, x)[2] = cameraSpacePoint[index].Z;
	//		}
	//	}
	//	delete[] cameraSpacePoint;		// メモリ開放忘れない

	//}
	//else {
	//	return false;
	//}
	//SafeRelease(pDepthFrame);

	return true;
}

#ifdef USE_COLOR_V2
void MultiCursorAppCpp::getRgbImageV2()
{
	int width = 1920;
	int height = 1080;
	unsigned int bufferSize = width * height * 4 * sizeof(unsigned char);
	cv::Mat bufferMat(height, width, CV_8UC4);
	rgbImage = Mat(height / 2, width / 2, CV_8UC4);

	// Frame
	IColorFrame* pColorFrame = nullptr;
	HRESULT hResult = S_OK;
	hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
	if (SUCCEEDED(hResult)){
		hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(bufferMat.data), ColorImageFormat_Bgra);
		if (SUCCEEDED(hResult)){
			cv::resize(bufferMat, rgbImage, cv::Size(), 0.5, 0.5);
		}
	}
	SafeRelease(pColorFrame);
}
#endif

#pragma endregion


// Main loop
void MultiCursorAppCpp::run()
{
	/* 1. Get frame data and prepear data needed */
	bool isGetFrameData = getFrameData();

	if (isGetFrameData)
	{
		/* 2. Labeling users' area */
		CvBlobs blobs = labelingUserArea(userAreaMat);

		/* 3. Detect users' head postiions */
		detectHeadPosition(blobs);

		/* 4. Detect users' hand positions */
		detectHandPosition(blobs);

		/* 5. Draw cursors position */
		setCursor(blobs);

		/* 6. Detect hand gesture */
		detectHandGesture(blobs);

		/* Show images */
		isShowDebugWindows ? showDebugWindows() : destroyAllWindows();
	}
}

void MultiCursorAppCpp::showDebugWindows()
{
	if (!userAreaMat.empty()) { imshow("Hand/Head detection", userAreaMat); }

#ifndef USE_KINECT_V1
	if (!depthImage.empty()) { imshow("Depth image", depthImage); }	// 今はKinect2のみ
#endif

#ifdef USE_COLOR_V2
	if (!rgbImage.empty()) { imshow("Color image", rgbImage); }
#endif
}

void MultiCursorAppCpp::detectHandGesture(CvBlobs blobs)
{
	INT blobID = 0;
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it) {


		++blobID;
	}
}

bool MultiCursorAppCpp::getFrameData()
{
#ifdef USE_KINECT_V1
	// Wait for updating frame data
	DWORD ret = ::WaitForSingleObject(streamEvent, 0);
	::ResetEvent(streamEvent);

	if (ret != WAIT_TIMEOUT)
	{
		// Get depth image
		if (!getDepthImageV1()) { return false; }

		// Get color image
		getRgbImageV1();
	}
#else
	// Get depth image from kinect v2
	if (!getDepthImageV2()) { return false; }

#ifdef USE_COLOR_V2
	getRgbImageV2();
#endif

#endif
	return true;
}


CvBlobs MultiCursorAppCpp::labelingUserArea(Mat& src)
{
#ifdef USE_KINECT_V1
	// Make image dilating for stable labeling
	dilate(src, src, Mat(), Point(-1, -1), 3);
#else
	dilate(src, src, Mat(), Point(-1, -1), 1);
#endif

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
	
	// Filter noise / ノイズ点の消去
	cvFilterByArea(blobs, 1000, 1000000);

	// 面積が小さすぎるものは消去
	//for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ) {
	//	if (it->second->area / point3fMatrix.ptr<float>(it->second->centroid.y, it->second->centroid.x)[2] < 1000){
	//		it = blobs.erase(it);
	//	}
	//	else {
	//		++it;
	//	}
	//}

	// Render blobs
	cvRenderBlobs(labelImg, blobs, &srcIpl, &srcIpl);

	// Free unused IplImages
	cvReleaseImage(&labelImg);
	cvReleaseImage(&srcIplBinary);
	
	return blobs;
}

void MultiCursorAppCpp::detectHeadPosition(CvBlobs blobs)
{
	// Initialize userData
	userData.clear();

	// 重心の位置を元に全フレームとのユーザ領域の対応を取る
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it)
	{
		UserData newUserData;
		newUserData.isDataFound = false;
		for (p = preUserData.begin(); p != preUserData.end(); ++p)
		{
			if (p->isDataFound)	{ continue; }	// 既に見つかった領域は無視する
			float distance = sqrt(pow(it->second->centroid.x - p->centroid.x, 2) + pow(it->second->centroid.y - p->centroid.y, 2));
			float distanceZ = abs(*heightMatrix.ptr<USHORT>(it->second->centroid.y, it->second->centroid.x) - *heightMatrix.ptr<USHORT>(p->centroid.y, p->centroid.x));
			cout << distance << endl;
			if (distance < 30.0f && distanceZ < 200.0f)	// 十分に距離が近いかチェック
			{
				p->isDataFound = true;

				newUserData.isDataFound = true;
				newUserData.headInfo.height = p->headInfo.height;
				newUserData.headInfo.depthPoint.x = p->headInfo.depthPoint.x;
				newUserData.headInfo.depthPoint.y = p->headInfo.depthPoint.y;
				break;
			}
		}
		newUserData.centroid.x = it->second->centroid.x;
		newUserData.centroid.y = it->second->centroid.y;
		userData.push_back(newUserData);
	}

	// Find the highest point of each user area
	USHORT* headHeights = new USHORT[blobs.size()];
	Point2i* newHighestPositions = new Point2i[blobs.size()];
	int blobID = 0;
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); it++)
	{
		headHeights[blobID] = 0;
		newHighestPositions[blobID].x = 0;
		newHighestPositions[blobID].y = 0;
		for (int y = it->second->miny; y <= it->second->maxy; y++)
		{
			for (int x = it->second->minx; x <= it->second->maxx; x++)
			{
				if (0 <= blobID && blobID < blobs.size())
				{
					USHORT height = *heightMatrix.ptr<USHORT>(y, x);
					if (headHeights[blobID] < height && height < HEAD_HEIGHT_MAX)
					{
						headHeights[blobID] = height;
						newHighestPositions[blobID].x = x;
						newHighestPositions[blobID].y = y;
					}
				}
			}
		}
		// Debug: Show the highest point of each users
		//circle(userAreaMat, Point(newHighestPositions[blobID].x, newHighestPositions[blobID].y), 5, Scalar(255, 0, 255), 3);
		
		blobID++;
	}	


	// Define users' head positions
	Point2i* newHeadPositions = new Point2i[blobs.size()];
	INT* numHeadPoints = new INT[blobs.size()];
	blobID = 0;
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it)
	{
		// Set the highest position as a base point of searching
		userData[blobID].headInfo.depthPoint.x = newHighestPositions[blobID].x;
		userData[blobID].headInfo.depthPoint.y = newHighestPositions[blobID].y;
		userData[blobID].headInfo.height = headHeights[blobID];

		if (userData[blobID].isDataFound)
		{
			// Check distance between 2d positions in current frame and in preframe
			float distance = sqrt(
				pow(userData[blobID].headInfo.depthPoint.x - preUserData[blobID].headInfo.depthPoint.x, 2)
				+ pow(userData[blobID].headInfo.depthPoint.y - preUserData[blobID].headInfo.depthPoint.y, 2)
				);
			//cout << distance << endl;
			// If the point is far from predata, just use pre-data	/ もし前回のフレームより大きく頭の位置がずれていたら前回の値を使う
			if (distance > 50.0f)
			{
				userData[blobID].headInfo.height = preUserData[blobID].headInfo.height;
				userData[blobID].headInfo.depthPoint.x = preUserData[blobID].headInfo.depthPoint.x;
				userData[blobID].headInfo.depthPoint.y = preUserData[blobID].headInfo.depthPoint.y;
			}
		}


		// Estimate exact head positions (Get average)
		numHeadPoints[blobID] = 0;
		newHeadPositions[blobID].x = 0;
		newHeadPositions[blobID].y = 0;
#if 1
		int offset_head = 50;	// ミーティングルーム用
#else
		int offset_head = 100;  // 作業場用
#endif
		int minY = userData[blobID].headInfo.depthPoint.y - offset_head;  if (minY < 0) minY = 0;
		int maxY = userData[blobID].headInfo.depthPoint.y + offset_head;  if (maxY > kinectBasics.heightDepth) maxY = kinectBasics.heightDepth;
		int minX = userData[blobID].headInfo.depthPoint.x - offset_head;  if (minX < 0) minX = 0;
		int maxX = userData[blobID].headInfo.depthPoint.x + offset_head;  if (maxX > kinectBasics.widthDepth) maxX = kinectBasics.widthDepth;
		for (int y = minY; y <= maxY; y++)
		{
			for (int x = minX; x <= maxX; x++)
			{
				USHORT height = *heightMatrix.ptr<USHORT>(y, x);
				if ((userData[blobID].headInfo.height - HEAD_LENGTH) < height && height < HEAD_HEIGHT_MAX)
				{
					newHeadPositions[blobID].x += x;
					newHeadPositions[blobID].y += y;
					numHeadPoints[blobID]++;
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
			// Calculate head position in 2D pixel
			userData[i].headInfo.depthPoint.x = newHeadPositions[i].x / numHeadPoints[i];
			userData[i].headInfo.depthPoint.y = newHeadPositions[i].y / numHeadPoints[i];

		}
		else
		{
			// 点が見つからなかった場合は最も高い点を頭にする
			userData[i].headInfo.depthPoint.x = newHighestPositions[i].x;
			userData[i].headInfo.depthPoint.y = newHighestPositions[i].y;
		}
		// Calculate head position in 3D point
		float* headPosition = point3fMatrix.ptr<float>(userData[i].headInfo.depthPoint.y, userData[i].headInfo.depthPoint.x);
		userData[i].headInfo.cameraPoint.x = headPosition[0];
		userData[i].headInfo.cameraPoint.y = headPosition[1];
		userData[i].headInfo.cameraPoint.z = headPosition[2];

		// Debug: Show the head point
		circle(userAreaMat, Point(userData[i].headInfo.depthPoint.x, userData[i].headInfo.depthPoint.y), 7, Scalar(255, 0, 0), 3);
	}

}

void MultiCursorAppCpp::detectHandPosition(CvBlobs blobs)
{
	FLOAT offset = 0.01f;
	INT blobID = 0;
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it) {
		int numIntersectionPoints = 0;
		Vector4 handPosition;
		handPosition.w = 1;
		handPosition.x = 0.0f;
		handPosition.y = 0.0f;
		handPosition.z = 0.0f;
		Point3f center3f = Point3_<FLOAT>(userData[blobID].headInfo.cameraPoint.x, userData[blobID].headInfo.cameraPoint.y, userData[blobID].headInfo.cameraPoint.z);

		// ユーザの領域内を探索
		for (int y = it->second->miny; y <= it->second->maxy; y++) {
			for (int x = it->second->minx; x <= it->second->maxx; x++)
			{
				float length = sqrt(
					pow(center3f.x - point3fMatrix.ptr<float>(y, x)[0], 2)
					+ pow(center3f.y - point3fMatrix.ptr<float>(y, x)[1], 2)
					//+ pow(center3f.z - point3fMatrix.ptr<float>(y, x)[2], 2)
					);
				// Define the intersection point of the sphere which its center is head and the hand as the hand position 
				if (*heightMatrix.ptr<USHORT>(y, x) > userData[blobID].headInfo.height - HEAD_LENGTH - SHOULDER_LENGTH	// 肩より高い点か
					&& 0 < point3fMatrix.ptr<float>(y, x)[2]*1000 && point3fMatrix.ptr<float>(y, x)[2] * 1000 < KINECT_HEIGHT	// Kinectの検出できる範囲の値か
					) // Don't include desk
				{
					if (SENCIG_CIRCLE_RADIUS - offset < length && length < SENCIG_CIRCLE_RADIUS + offset) {	// 注目点が球と交差しているかどうか
						handPosition.x += point3fMatrix.ptr<float>(y, x)[0];
						handPosition.y += point3fMatrix.ptr<float>(y, x)[1];
						handPosition.z += point3fMatrix.ptr<float>(y, x)[2];

						circle(userAreaMat, Point(x, y), 2, Scalar(255, 255, 0), -1);
						numIntersectionPoints++;
					}
					else if (length > SENCIG_CIRCLE_RADIUS) {
#ifdef TRACK_GESTURE_BY_AREA
						++userData[blobID].handInfo.area;	// 手の面積を点の数で表す

#ifdef USE_KINECT_V1
						LONG handPositionX2d;
						LONG handPositionY2d;
						USHORT dis;
						NuiTransformSkeletonToDepthImage(handPosition, &handPositionX2d, &handPositionY2d, &dis, KINECT_RESOLUTION);
						circle(userAreaMat, Point(handPositionX2d, handPositionY2d), 7, Scalar(0, 200, 0), 3);
#else
						DepthSpacePoint depthPoint;
						CameraSpacePoint cameraPoint;
						cameraPoint.X = point3fMatrix.ptr<float>(y, x)[0];
						cameraPoint.Y = point3fMatrix.ptr<float>(y, x)[1];
						cameraPoint.Z = point3fMatrix.ptr<float>(y, x)[2];
						kinectBasics.GetMapper()->MapCameraPointToDepthSpace(cameraPoint, &depthPoint);

						int index = ((y * kinectBasics.widthDepth) + x) * 3;
						UCHAR* dataDepth = &userAreaMat.data[index];
						dataDepth[0] = 0;
						dataDepth[1] = 200;
						dataDepth[2] = 255;
						//circle(userAreaMat, Point(depthPoint.X, depthPoint.Y), 1, Scalar(0, 255, 255), 3);
#endif
#endif
					}
				}
			}
		}


		if (numIntersectionPoints > 0)
		{
			userData[blobID].cursorInfo.isShownCursor = true;

			handPosition.x /= numIntersectionPoints;
			handPosition.y /= numIntersectionPoints;
			handPosition.z /= numIntersectionPoints;

			userData[blobID].handInfo.cameraPoint.x = handPosition.x;
			userData[blobID].handInfo.cameraPoint.y = handPosition.y;
			userData[blobID].handInfo.cameraPoint.z = handPosition.z;

			// Show the hand positions on the depth image
#ifdef USE_KINECT_V1
			LONG handPositionX2d;
			LONG handPositionY2d;
			USHORT dis;
			NuiTransformSkeletonToDepthImage(handPosition, &handPositionX2d, &handPositionY2d, &dis, KINECT_RESOLUTION);
			circle(userAreaMat, Point(handPositionX2d, handPositionY2d), 7, Scalar(0, 200, 0), 3);
#else
			DepthSpacePoint depthPoint;
			CameraSpacePoint cameraPoint;
			cameraPoint.X = handPosition.x;
			cameraPoint.Y = handPosition.y;
			cameraPoint.Z = handPosition.z;
			kinectBasics.GetMapper()->MapCameraPointToDepthSpace(cameraPoint, &depthPoint);
			circle(userAreaMat, Point(depthPoint.X, depthPoint.Y), 7, Scalar(0, 200, 0), 3);
#endif
		}
		else
		{
			userData[blobID].cursorInfo.isShownCursor = false;

			userData[blobID].handInfo.cameraPoint.x = 0.0f;
			userData[blobID].handInfo.cameraPoint.y = 0.0f;
			userData[blobID].handInfo.cameraPoint.z = 0.0f;
		}

#ifdef TRACK_GESTURE_BY_AREA
		// 面積で手の開閉をチェック
		if (userData[blobID].handInfo.area != 0) {
			float distance = sqrt(pow(handPosition.x, 2) + pow(handPosition.y, 2) + pow(handPosition.z, 2));
			userData[blobID].handInfo.area /= distance;
		}
		// cout << "area: " << userData[blobID].handInfo.area << endl;
#endif


		blobID++;
	}

	// Replace preUserData by current userData	/ 前フレームのデータを現フレームのデータで置き換える
	preUserData.clear();
	preUserData = userData; // 参照で大丈夫かな？
	for (p = preUserData.begin(); p != preUserData.end(); p++)
	{
		p->isDataFound = false; // 初期化
	}
}

void MultiCursorAppCpp::setCursor(CvBlobs blobs)
{
	INT blobID = 0;
	for (CvBlobs::const_iterator it = blobs.begin(); it != blobs.end(); ++it) {
		if (userData[blobID].cursorInfo.isShownCursor)
		{
			Mat handPoint = (cv::Mat_<float>(4, 1) << userData[blobID].handInfo.cameraPoint.x*1000, userData[blobID].handInfo.cameraPoint.y*1000, userData[blobID].handInfo.cameraPoint.z*1000, 1);
			Mat headPoint = (cv::Mat_<float>(4, 1) << userData[blobID].headInfo.cameraPoint.x*1000, userData[blobID].headInfo.cameraPoint.y*1000, userData[blobID].headInfo.cameraPoint.z*1000, 1);

#ifdef USE_KINECT_V1
			Mat handPointScreen = T_WorldToScreen * T_KinectCameraToWorld * handPoint;
			Mat headPointScreen = T_WorldToScreen * T_KinectCameraToWorld * headPoint;
#else
			Mat handPointScreen = TMarker2Display * TKinect2Marker * TKinectDepth2Color * handPoint;
			Mat headPointScreen = TMarker2Display * TKinect2Marker * TKinectDepth2Color * headPoint;
#endif

			// Caliculate the intersection point of vector and screen
			float xvec = *handPointScreen.ptr<float>(0, 0) - *headPointScreen.ptr<float>(0, 0);
			float yvec = *handPointScreen.ptr<float>(1, 0) - *headPointScreen.ptr<float>(1, 0);
			float zvec = *handPointScreen.ptr<float>(2, 0) - *headPointScreen.ptr<float>(2, 0);

			float val = - *handPointScreen.ptr<float>(2, 0) / zvec;

			// Calculate cursor position in real scall
			Point3f cursorScreen3d;
			cursorScreen3d.x = val * xvec + *headPointScreen.ptr<float>(0, 0);
			cursorScreen3d.y = val * yvec + *headPointScreen.ptr<float>(1, 0);
			cursorScreen3d.z = 0.0f;

			// Calculate cursor position in pixel coordinate
#ifdef USE_KINECT_V1
			// Calculate cursor position in pixel coordinate
			float screen3dTo2d = 246 / 0.432f;
			userData[blobID].cursorInfo.position.x = cursorScreen3d.x * screen3dTo2d;
			userData[blobID].cursorInfo.position.y = cursorScreen3d.y * screen3dTo2d;
#else
			userData[blobID].cursorInfo.position.x =  (*TDisplay2Pixel.ptr<float>(0, 0) * cursorScreen3d.x + *TDisplay2Pixel.ptr<float>(0, 1) * cursorScreen3d.y + *TDisplay2Pixel.ptr<float>(0, 2));
			userData[blobID].cursorInfo.position.y =  (*TDisplay2Pixel.ptr<float>(1, 0) * cursorScreen3d.x + *TDisplay2Pixel.ptr<float>(1, 1) * cursorScreen3d.y + *TDisplay2Pixel.ptr<float>(1, 2));
#endif
		}
		++blobID;
	}
}

void MultiCursorAppCpp::MouseControl(float x, float y)
{
	// スクリーン座標をmouse_event()用の座標変換
	DWORD dwX = x * 65535 / ::GetSystemMetrics(SM_CXSCREEN);
	DWORD dwY = y * 65535 / ::GetSystemMetrics(SM_CYSCREEN);

	// Set mouse cursor position
	::mouse_event(MOUSEEVENTF_ABSOLUTE | MOUSEEVENTF_MOVE, dwX, dwY, NULL, NULL);
}

#pragma region OpenGL

void MultiCursorAppCpp::initGL(int argc, char* argv[])
{
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutCreateWindow("MultiCursorAppCpp");

	//// Register callback functions
	//glutReshapeFunc(sreshape);
	glutDisplayFunc(sdisplay);
	glutIdleFunc(sidle);
	glutKeyboardFunc(skeyboard);
	//glutMouseFunc(smouse);

	glClearColor(1.0, 1.0, 1.0, 1.0);

	/* Camera setup */
	glViewport(0, 0, kinectBasics.widthDepth, kinectBasics.heightDepth);
	glLoadIdentity();

	/* GLのウィンドウをフルスクリーンに */
	//GLのデバイスコンテキストハンドル取得
	HDC glDc = wglGetCurrentDC();
	//ウィンドウハンドル取得
	HWND hWnd = WindowFromDC(glDc);
	//ウィンドウの属性と位置変更
	SetWindowLong(hWnd, GWL_STYLE, WS_POPUP);
	SetWindowPos(hWnd, HWND_TOP, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, SWP_SHOWWINDOW);
}

void MultiCursorAppCpp::display(void)
{
	glClear(GL_COLOR_BUFFER_BIT);
	if (userData.size() > 0)
	{
		for (int i = 0; i < userData.size(); ++i)
		{
			if (userData[i].cursorInfo.isShownCursor)
			{
#ifdef USE_KINECT_V1
				Point2f cursorPos((userData[i].cursorInfo.position.x - WINDOW_WIDTH / 2) / WINDOW_WIDTH, -(userData[i].cursorInfo.position.y - WINDOW_HEIGHT / 2) / WINDOW_HEIGHT);
#else
				Point2f cursorPos((userData[i].cursorInfo.position.x - WINDOW_WIDTH / 2) / WINDOW_WIDTH, (userData[i].cursorInfo.position.y - WINDOW_HEIGHT / 2) / WINDOW_HEIGHT);
#endif
				glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
				glBegin(GL_QUADS);
				float offset = 40 * 5;
				float offsetX = (offset / (float)WINDOW_WIDTH);
				float offsetY = (offset / (float)WINDOW_HEIGHT);
				glVertex2f(cursorPos.x, cursorPos.y);
				glVertex2f(cursorPos.x + offsetX, cursorPos.y);
				glVertex2f(cursorPos.x + offsetX, cursorPos.y + offsetY);
				glVertex2f(cursorPos.x, cursorPos.y + offsetY);
				glEnd();
			}
		}
	}

	glFlush();
	glutSwapBuffers();

}

void MultiCursorAppCpp::idle(void)
{
	this->run();

	glutPostRedisplay();
}

void MultiCursorAppCpp::keyboard(unsigned char key, int x, int y)
{
	switch (key) {
	case 'q':
	case 'Q':
	case '\033':	// ESC
		exit(0);
		break;
	case 'd':
	case 'D':
		isShowDebugWindows = !isShowDebugWindows;
		break;
	default:
		break;
	}
}

static void sdisplay()
{
	app.display();
}

static void sidle(void)
{
	app.idle();
}

static void skeyboard(unsigned char key, int x, int y)
{
	app.keyboard(key, x, y);
}

#pragma endregion

void MultiCursorAppCpp::loadCalibData()
{
	FileStorage cvfsTd2c(KINECT_D2C_TRANS_FILENAME, CV_STORAGE_READ);
	FileNode nodeTd2c(cvfsTd2c.fs, NULL);
	FileNode fnTd2c = nodeTd2c[string("mat_array")];
	read(fnTd2c[0], TKinectDepth2Color);
	cout << TKinectDepth2Color << endl;

	FileStorage cvfsTk2m(KINECT_TRANS_FILENAME, CV_STORAGE_READ);
	FileNode nodeTk2m(cvfsTk2m.fs, NULL);
	FileNode fnTk2m = nodeTk2m[string("T_K2M")];
	read(fnTk2m[0], TKinect2Marker);
	//cout << TKinect2Marker << endl;

	FileStorage cvfsTm2d(MARKER_TRANS_FILENAME, CV_STORAGE_READ);
	FileNode nodeTm2d(cvfsTm2d.fs, NULL);
	FileNode fnTm2d = nodeTm2d[string("T_M2D")];
	read(fnTm2d[0], TMarker2Display);
	//cout << TMarker2Display << endl;

	FileStorage cvfsTd2p(DISP_TRANS_FILENAME, CV_STORAGE_READ);
	FileNode nodeTd2p(cvfsTd2p.fs, NULL);
	FileNode fnTd2p = nodeTd2p[string("disp1")];
	read(fnTd2p[0], TDisplay2Pixel);
	//cout << TDisplay2Pixel << endl;
}

void MultiCursorAppCpp::showHelp()
{
	cout << "Use ";
#ifdef USE_KINECT_V1
	cout << "Kinect V1" << endl;
#else
	cout << "Kinect V2" << endl;
#endif
	cout << "<Help>" << endl;
	cout << "p, P, ESC: Quit" << endl;
	cout << "d, D     : Toggle showing debug windows" << endl;
}

// クラス定義
MultiCursorAppCpp app;
KinectV2Basics kinectBasics;

int main(int argc, char* argv[])
{
	app.showHelp();
	
	app.loadCalibData();
	/* OpenGL */
	app.initGL(argc, argv);
#ifdef USE_KINECT_V1
	app.initKinect();
#else
	//app.initKinectV2();
	kinectBasics.SetupKinectV2();
#endif
	glutMainLoop();

	return 0;
}




// 今のところ使わない

//void MultiCursorAppCpp::mouse(int button, int state, int mouse_x, int mouse_y)
//{
//}

//void smouse(int button, int state, int mouse_x, int mouse_y)
//{
//	app.mouse(button, state, mouse_x, mouse_y);
//}

//static void sreshape(int w, int h)
//{
//	appSub.reshape(w, h);
//}

//void MultiCursorAppCpp::reshape(int w, int h)
//{
//	// Make the whole window as a viewport
//	glViewport(0, 0, w, h);
//	// Initialize transformation matrix
//	glLoadIdentity();
//}