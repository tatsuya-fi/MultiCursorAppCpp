#pragma once

class CalcMat
{
public:
	CalcMat();
	~CalcMat();

	void SetMat(const cv::Mat& src);
	void SetMat(const char* fileName, const char* fileNode);

	cv::Mat MultiMat(char* fileName);

private:
	std::vector<cv::Mat> vecMat;
};

