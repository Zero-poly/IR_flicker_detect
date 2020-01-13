#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include "GeometryTypes.hpp"
#include "CameraCalibration.hpp"

using namespace std;

class IR_detect
{
public:
	vector<cv::Point2f> sort_points;
	cv::Mat diff;
	cv::Mat cut_diff;
	cv::Mat open_diff;
	cv::Mat final_diff;
	bool isShow = true;
	void getDiffImage( );
	void cutInnerRing();
	void processCutDiff();
	
private:
	cv::Mat _camMatrix;
	cv::Mat _distCoeff;
	//还未进行干扰点剔除以及排序的圆斑质心点
	vector<cv::Point2f> raw_points;
	void calcPosAtti(vector<cv::Point2f>);
	double calSimilarity(vector<cv::Point3f> vec_1, vector<cv::Point3f> vec_2);
	//bool compare_1(cv::Point2f a, cv::Point2f b);
	//bool compare_2(cv::Point3f a, cv::Point3f b);
	
	vector<cv::Rect> getCandiAreas(cv::Mat raw_image);
	vector<cv::Point3f> getFeatureVec(vector<cv::Point2f> centers, cv::Point2f image_center);
	void initCalibration();
	vector<cv::Point2f> innerSort(vector<cv::Point2f> inner, cv::Point2f center);
	vector<cv::Point2f> outerSort(cv::Point2f first_pt, cv::Point2f second_pt, vector<cv::Point2f> outer);
	
	vector<cv::Point2f> sortPoints(vector<cv::Point2f> points);
};




