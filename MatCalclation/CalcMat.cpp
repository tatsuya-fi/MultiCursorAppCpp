#include "stdafx.h"
#include "CalcMat.h"

using namespace std;
using namespace cv;

CalcMat::CalcMat()
{
}


CalcMat::~CalcMat()
{
}

void CalcMat::SetMat(const cv::Mat& src)
{
	if (!src.empty())
	{
		// ‘O‚Ìs—ñ‚ÆŒvZ‚Å‚«‚é‚©”äŠr
		// if ()

		// vector‚É’Ç‰Á
		vecMat.push_back(src);
	}
	else
	{
		cout << "Error(CalcMat): Input Mat is empty." << endl;
		exit(0);
	}
}

void CalcMat::SetMat(const char* fileName, const char* fileNode)
{
	if (fileName == NULL)
	{
		cout << "Error(CalcMat): Filename should not be null." << endl;
		return;
	}
	Mat src;

	FileStorage cvfs(fileName, CV_STORAGE_READ);
	FileNode node(cvfs.fs, NULL);
	FileNode fn = node[string(fileNode)];
	read(fn[0], src);
	//cout << src << endl;

	SetMat(src);
}

cv::Mat CalcMat::MultiMat(char* fileName)
{
	if (vecMat.empty())
	{
		cout << "Error(CalcMat): No input mat." << endl;
		return Mat();
	}

	Mat dst = vecMat[0];
	for (size_t i = 0; i < vecMat.size(); ++i)
	{
		dst = vecMat[i] * dst;
	}

	if (fileName != NULL)
	{
		cv::FileStorage   cvfs(fileName, CV_STORAGE_WRITE);
		cv::WriteStructContext ws(cvfs, "mat_array", CV_NODE_SEQ);    // create node
		cv::write(cvfs, "", dst);

		cout << "Matrix was saved." << endl;
	}

	return dst;
}