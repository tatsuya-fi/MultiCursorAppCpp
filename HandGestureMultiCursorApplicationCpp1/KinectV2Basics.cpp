#include "stdafx.h"
#include "KinectV2Basics.h"

using namespace std;
using namespace cv;

KinectV2Basics::KinectV2Basics()
{
}


KinectV2Basics::~KinectV2Basics()
{
	// �e�|�C���^���J��
	SafeRelease(pDepthSource);
	SafeRelease(pDepthReader);
	SafeRelease(pDepthFrame);
	SafeRelease(pCoordinateMapper);

	if (pSensor)
	{
		pSensor->Close();
	}
	SafeRelease(pSensor);
}

bool KinectV2Basics::SetupKinectV2()
{
	HRESULT hResult = S_OK;
	// Sensor
	hResult = GetDefaultKinectSensor(&pSensor);
	if (FAILED(hResult))
	{
		cerr << "Error : GetDefaultKinectSensor" << endl;
		return false;
	}
	hResult = pSensor->Open();
	if (FAILED(hResult))
	{
		cerr << "Error : IKinectSensor::Open()" << endl;
		return false;
	}

	// Source(Depth)
	hResult = pSensor->get_DepthFrameSource(&pDepthSource);
	if (FAILED(hResult))
	{
		cerr << "Error : IKinectSensor::get_DepthFrameSource()" << endl;
		return false;
	}

	// Reader(Depth)
	hResult = pDepthSource->OpenReader(&pDepthReader);
	if (FAILED(hResult))
	{
		cerr << "Error : IDepthFrameSource::OpenReader()" << endl;
		return false;
	}

	// Source(Color)
	hResult = pSensor->get_ColorFrameSource(&pColorSource);
	if (FAILED(hResult))
	{
		cerr << "Error : IKinectSensor::get_ColorFrameSource()" << endl;
		return false;
	}

	// Reader(Color)
	hResult = pColorSource->OpenReader(&pColorReader);
	if (FAILED(hResult))
	{
		cerr << "Error : IColorFrameSource::OpenReader()" << endl;
		return false;
	}

	// Mapper
	hResult = pSensor->get_CoordinateMapper(&pCoordinateMapper);
	if (FAILED(hResult))
	{
		cerr << "Error : IKinectSensor::get_CoordinateMapper()" << endl;
		return false;
	}

	return true;
}

//--------------------------------------------------
// Getter

IKinectSensor* KinectV2Basics::GetSensor()
{
	return pSensor;
}

IDepthFrameSource* KinectV2Basics::GetSourceDepth()
{
	return pDepthSource;
}

IDepthFrameReader* KinectV2Basics::GetReaderDepth()
{
	return pDepthReader;
}

IDepthFrame* KinectV2Basics::GetFrameDepth()
{
	return pDepthFrame;
}

IColorFrameSource* KinectV2Basics::GetSourceColor()
{
	return pColorSource;
}

IColorFrameReader* KinectV2Basics::GetReaderColor()
{
	return pColorReader;
}

IColorFrame* KinectV2Basics::GetFrameColor()
{
	return pColorFrame;
}

ICoordinateMapper* KinectV2Basics::GetMapper()
{
	return pCoordinateMapper;
}

//--------------------------------------------------
// OpenCV��p�������\�b�h

#ifdef OPENCV

bool KinectV2Basics::GetDepthMat(Mat& outDepth16U)
{
	unsigned int bufferSize = widthDepth * heightDepth * sizeof(unsigned short);
	Mat bufferMat(heightDepth, widthDepth, CV_16SC1);

	HRESULT hResult = S_OK;
	SafeRelease(pDepthFrame);
	hResult = pDepthReader->AcquireLatestFrame(&pDepthFrame);
	if (SUCCEEDED(hResult))
	{
		hResult = pDepthFrame->AccessUnderlyingBuffer(&bufferSize, reinterpret_cast<UINT16**>(&bufferMat.data));
		// y���ɑ΂��Ĕ��]...���Ȃ��I
		Mat bufferMatFlip = bufferMat.clone();
		//flip(bufferMat, bufferMatFlip, 1);
		if (SUCCEEDED(hResult))
		{
			// Z���������i�[
			outDepth16U = bufferMatFlip;
		}
		else
		{
			//cerr << "Error : IDepthFrameSource::AccessUnderlyingBuffer()" << endl;
			return false;
		}
	}
	else
	{
		//cerr << "Error : IDepthFrame::AcquireLatestFrame()" << endl;
		return false;
	}

	return true;
}

bool KinectV2Basics::GetDepthMat(Mat& outDepth8UC3, Mat& outDepth16U)
{
	if (GetDepthMat(outDepth16U))
	{
		outDepth16U.convertTo(outDepth8UC3, CV_8U, -225.0f / 4500.0f, 255.0f);
		return true;
	}
	else
	{
		return false;
	}
}

bool KinectV2Basics::GetDepthMat(cv::Mat& outDepth8UC3, cv::Mat& outDepth16U, cv::Mat& outPoints32FC3)
{
	if (GetDepthMat(outDepth8UC3, outDepth16U))
	{
		// �O�����_�𓾂�
		outPoints32FC3 = Mat::zeros(heightDepth, widthDepth, CV_32FC3);
		// �������擾 (pDepthBuffer)
		unsigned int bufferSize = widthDepth * heightDepth * sizeof(unsigned short);
		UINT16* pDepthBuffer = NULL;
		HRESULT hResult = pDepthFrame->AccessUnderlyingBuffer(&bufferSize, &pDepthBuffer);
		if (FAILED(hResult))
		{
			cout << "Error: AccessUnderlyingBuffer(). You need to call GetDepthMat() first" << endl;
			return false;
		}

		// �[�x�J�������W�n�ł̎O�������W�����߂�
		UINT pointNum = widthDepth * heightDepth;
		CameraSpacePoint* cameraSpacePoint = new CameraSpacePoint[widthDepth * heightDepth];	// Points in camera coordinate [m]
		hResult = pCoordinateMapper->MapDepthFrameToCameraSpace(pointNum, pDepthBuffer, pointNum, cameraSpacePoint);
		if (FAILED(hResult))
		{
			cout << "Error: MapDepthFrameToCameraSpace()" << endl;
			return false;
		}
		for (int y = 0; y < heightDepth; ++y)
		{
			for (int x = 0; x < widthDepth; ++x)
			{
				int index = y * widthDepth + x;
				outPoints32FC3.ptr<float>(y, x)[0] = cameraSpacePoint[index].X;
				outPoints32FC3.ptr<float>(y, x)[1] = cameraSpacePoint[index].Y;
				outPoints32FC3.ptr<float>(y, x)[2] = cameraSpacePoint[index].Z;
			}
		}
		delete[] cameraSpacePoint;	// �������J���K�v

		return true;
	}
	else
	{
		return false;
	}
}

bool KinectV2Basics::GetPointsMat(Mat& pointsMat)
{
	// Init
	pointsMat = Mat::zeros(heightDepth, widthDepth, CV_32FC3);

	// �������擾 (pDepthBuffer)
	unsigned int bufferSize = widthDepth * heightDepth * sizeof(unsigned short);
	UINT16* pDepthBuffer = NULL;
	HRESULT hResult = pDepthFrame->AccessUnderlyingBuffer(&bufferSize, &pDepthBuffer);
	if (FAILED(hResult))
	{
		cout << "Error: AccessUnderlyingBuffer(). You need to call GetDepthMat() first" << endl;
		return false;
	}

	// �[�x�J�������W�n�ł̎O�������W�����߂�
	UINT pointNum = widthDepth * heightDepth;
	CameraSpacePoint* cameraSpacePoint = new CameraSpacePoint[widthDepth * heightDepth];	// Points in camera coordinate [m]
	hResult = pCoordinateMapper->MapDepthFrameToCameraSpace(pointNum, pDepthBuffer, pointNum, cameraSpacePoint);
	if (FAILED(hResult))
	{
		cout << "Error: MapDepthFrameToCameraSpace()" << endl;
		return false;
	}
	for (int y = 0; y < heightDepth; ++y)
	{
		for (int x = 0; x < widthDepth; ++x)
		{
			int index = y * widthDepth + x;
			pointsMat.ptr<float>(y, x)[0] = cameraSpacePoint[index].X;
			pointsMat.ptr<float>(y, x)[1] = cameraSpacePoint[index].Y;
			pointsMat.ptr<float>(y, x)[2] = cameraSpacePoint[index].Z;
		}
	}
	delete[] cameraSpacePoint;	// �������J���K�v

	return true;
}

bool KinectV2Basics::GetColorMat(cv::Mat& outColor)
{
	unsigned int bufferSize = widthColor * heightColor * 4 * sizeof(unsigned char);
	Mat colorMat(heightColor, widthColor, CV_8UC4);

	HRESULT hResult = pColorReader->AcquireLatestFrame(&pColorFrame);
	if (SUCCEEDED(hResult))
	{
		hResult = pColorFrame->CopyConvertedFrameDataToArray(bufferSize, reinterpret_cast<BYTE*>(colorMat.data), ColorImageFormat_Bgra);
		Mat colorMat2;
		// y���ɑ΂��Ĕ��]������
		flip(colorMat, colorMat2, 1);
		if (SUCCEEDED(hResult))
		{
			// �J���[�摜���i�[
			outColor = colorMat2;

			SafeRelease(pColorFrame);
			return true;

		}
		else
		{
			//cerr << "Error: IColorFrame::CopyConvertedFrameDataToArray()" << endl;
			return false;
		}
	}
	else
	{
		//cerr << "Error: IColorFrameReader::AcquireLatestFrame()" << endl;
		return false;
	}
}

bool KinectV2Basics::GetColorMat(cv::Mat& outColor, float scale)
{
	if (GetColorMat(outColor))
	{
		resize(outColor, outColor, Size(), scale, scale);
		return true;
	}
	else
	{
		return false;
	}
}

#endif
