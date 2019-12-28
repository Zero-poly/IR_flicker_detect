#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include "GeometryTypes.hpp"
#include "CameraCalibration.hpp"

using namespace std;

//图像匹配
void NTSS(cv::Mat big, cv::Mat small)
{
	assert(big.cols > (small.cols + 16));
	assert(big.rows > (small.rows + 16));

	if (big.channels() == 3)
		cv::cvtColor(big, big, CV_BGR2GRAY);
	if (small.channels() == 3)
		cv::cvtColor(small, small, CV_BGR2GRAY);

	int x_c = int(big.cols / 2.0);
	int y_c = int(big.rows / 2.0);
	int count = 0;
	int w = small.cols, h = small.rows;

	vector<cv::Point> res = { cv::Point(0,0),cv::Point(-1,-1), cv::Point(0,-1), cv::Point(1,-1), cv::Point(-1,0),
						cv::Point(1,0), cv::Point(-1,1), cv::Point(0,1), cv::Point(1,1) };


	while (count < 3)
	{
		vector<int> scores;
		for (int i = 0; i < res.size(); i++)
		{
			cv::Mat roi, diff;
			roi = big(cv::Rect(x_c + res[i].x - w / 2.0, y_c + res[i].y - h / 2.0, w, h));
			cv::absdiff(roi, small, diff);
			//threshold(diff, diff, 100, 255, THRESH_BINARY);
			scores.push_back(countNonZero(diff));
			//cv::imwrite("C:\\Users\\LIUU\\Pictures\\correct_diff_" + to_string(count + 1) + "_" + to_string(i) + ".jpg", diff);
		}
		vector<int>::iterator smallest = min_element(begin(scores), end(scores));
		int pos = distance(begin(scores), smallest);
		cout << pos << endl;
		if (pos == 0)
			break;
		x_c += res[pos].x;
		y_c += res[pos].y;
		count++;
	}
	cv::Mat roi, diff;
	roi = big(cv::Rect(x_c - w / 2.0, y_c - h / 2.0, w, h));
	cv::absdiff(roi, small, diff);
	cv::imshow("correct_diff", diff);
}


//点pt到点pt_1和pt_2所在直线的距离
float getDistance(cv::Point2f pt_1, cv::Point2f pt_2, cv::Point2f pt)
{
	float A, B, C, dis;
	A = pt_2.y - pt_1.y;
	B = pt_1.x - pt_2.x;
	C = pt_2.x * pt_1.y - pt_1.x * pt_2.y;

	dis = abs(A * pt.x + B * pt.y + C) / sqrt(A * A + B * B);
	return dis;
}

//计算图像坐标系下点pt与基点pt_0与x轴正向夹角
float calcAngel(cv::Point2f pt_0, cv::Point2f pt)
{
	float delta_x = pt.x - pt_0.x;
	float delta_y = pt.y - pt_0.y;

	assert(delta_x != 0 || delta_y != 0);

	if (delta_x == 0)
	{
		if (delta_y > 0)
			return 90;
		else
			return -90;
	}

	return cvFastArctan(delta_y, delta_x);
}


//内圈三个点排序
vector<cv::Point2f> innerSort(vector<cv::Point2f> inner, cv::Point2f center)
{
	assert(inner.size() == 3);

	vector<float> angels;
	for (int i = 0; i < 3; i++)
	{
		float ang = cvFastArctan(inner[i].y - center.y, inner[i].x - center.x);
		angels.push_back(ang);
	}

	vector<cv::Point2f> new_inner(3);
	vector<float>::iterator max = max_element(begin(angels), end(angels));
	vector<float>::iterator min = min_element(begin(angels), end(angels));
	
	for (int i = 0; i < 3; i++)
	{
		if (angels[i] == *max)
			new_inner[1] = inner[i];
		else if (angels[i] == *min)
			new_inner[2] = inner[i];
		else
			new_inner[0] = inner[i];
	}

	return new_inner;
}

bool compare_1(cv::Point2f a, cv::Point2f b)
{
	return a.x < b.x;
}

//外圈四个点排序
vector<cv::Point2f> outerSort(cv::Point2f first_pt, cv::Point2f second_pt, vector<cv::Point2f> outer)
{
	assert(outer.size() == 4);

	cv::Point2f distance[4];
	for (int i = 0; i < 4; i++)
	{
		float x = sqrt( pow(outer[i].x - first_pt.x,2)+ pow(outer[i].y - first_pt.y, 2));
		float y = i;
		distance[i] = cv::Point2f(x, y);
	}
	sort(distance, distance + 4, compare_1);

	float dis_1= sqrt(pow(outer[distance[0].y].x - second_pt.x, 2) + pow(outer[distance[0].y].y - second_pt.y, 2));
	float dis_2 = sqrt(pow(outer[distance[1].y].x - second_pt.x, 2) + pow(outer[distance[1].y].y - second_pt.y, 2));

	int first_pos;
	if (dis_1 < dis_2)
		first_pos = int(distance[0].y);
	else
		first_pos = int(distance[1].y);

	vector<cv::Point2f> new_outer(4);
	new_outer[0] = outer[first_pos % 4];
	new_outer[1] = outer[(first_pos+1) % 4];
	new_outer[2] = outer[(first_pos+2) % 4];
	new_outer[3] = outer[(first_pos + 3) % 4];

	return new_outer;
}

//对七个斑点进行排序
vector<cv::Point2f> sortPoints(vector<cv::Point2f> points)
{
	assert(points.size() == 7);

	//先找出最外圈四个点，按顺时针排序
	vector<float> x_vec, y_vec;
	for (int i = 0; i < points.size(); i++)
	{
		//cout << "(" << points[i].x << "," << points[i].y << ")" << "  ";
		x_vec.push_back(points[i].x);
		y_vec.push_back(points[i].y);
	}
	//cout << endl;

	vector<float>::iterator max_x = max_element(begin(x_vec), end(x_vec));
	vector<float>::iterator min_x = min_element(begin(x_vec), end(x_vec));
	vector<float>::iterator max_y = max_element(begin(y_vec), end(y_vec));
	vector<float>::iterator min_y = min_element(begin(y_vec), end(y_vec));
	//std::cout << "Max x: " << *max_x << " at position " << distance(begin(x_vec), max_x) << endl;
	//std::cout << "Min x: " << *min_x << " at position " << distance(begin(x_vec), min_x) << endl;
	//std::cout << "Max y: " << *max_y << " at position " << distance(begin(y_vec), max_y) << endl;
	//std::cout << "Min y: " << *min_y << " at position " << distance(begin(y_vec), min_y) << endl;

	vector<cv::Point2f> outer;
	outer.push_back(points[distance(begin(y_vec), min_y)]);
	outer.push_back(points[distance(begin(x_vec), max_x)]);
	outer.push_back(points[distance(begin(y_vec), max_y)]);
	outer.push_back(points[distance(begin(x_vec), min_x)]);

	//排除外圈四个点后，剩下的就是内圈三个点
	vector<cv::Point2f> inner;
	for (int i = 0; i < points.size(); i++)
	{
		if (i == distance(begin(y_vec), min_y))
			continue;
		if (i == distance(begin(x_vec), max_x))
			continue;
		if (i == distance(begin(y_vec), max_y))
			continue;
		if (i == distance(begin(x_vec), min_x))
			continue;
		inner.push_back(points[i]);
	}

	float x_c = (outer[0].x + outer[1].x + outer[2].x + outer[3].x) / 4.0;
	float y_c = (outer[0].y + outer[1].y + outer[2].y + outer[3].y) / 4.0;

	vector<cv::Point2f> new_inner = innerSort(inner, cv::Point2f(x_c, y_c));
	vector<cv::Point2f> new_outer = outerSort(new_inner[0], new_inner[1], outer);

	vector<cv::Point2f> new_sort;
	new_sort.insert(new_sort.end(), new_inner.begin(), new_inner.end());
	new_sort.insert(new_sort.end(), new_outer.begin(), new_outer.end());

	return new_sort;
}

//对帧差法得到的图预处理，提取LED亮斑，确定各个斑点序号
void processDiff(cv::Mat image)
{
	assert(image.channels() == 1);

	cv::threshold(image, image, 100, 255, THRESH_BINARY);
	cv::imshow("binary", image);

	//cv::Canny(image, image, 50, 100);
	//cv::imshow("canny", image);

	vector<vector<cv::Point>> contours;
	vector<Vec4i> hierarchy;
	cv::findContours(image, contours, hierarchy, RETR_EXTERNAL,CHAIN_APPROX_NONE);

	vector<cv::Moments> mu(contours.size());
	for (unsigned int i = 0; i < contours.size(); i++)
	{
		mu[i] = cv::moments(contours[i], true);
	}

	vector<cv::Point2f> mc(contours.size());
	for (unsigned int i = 0; i < contours.size(); i++)
	{
		mc[i] = cv::Point2f(static_cast<float>(mu[i].m10 / mu[i].m00), static_cast<float>(mu[i].m01 / mu[i
		].m00));
	}
	cv::imshow("center", image);

	vector<cv::Point2f> sort_points=sortPoints(mc);

	for (int i = 0; i < sort_points.size(); i++)
	{
		cv::circle(image, sort_points[i], 5, Scalar(255));
		cv::imshow("sort", image);
		cv::waitKey(0);
	}

	cv::waitKey(0);
}

cv::Mat _camMatrix;
cv::Mat _distCoeff;
void initCalibration( )
{
	float fc1, fc2, cc1, cc2, kc1, kc2, kc3, kc4, distorsionCoeff[4];
	fc1 = 730.166036369399 / 1.6;
	fc2 = 729.992389107947 / 1.6;
	cc1 = 489.104940065542 / 1.6;
	cc2 = 383.685983738317 / 1.6;
	kc1 = -0.0116300151234399;
	kc2 = 0.0407467972044594;
	kc3 = 0;
	kc4 = 0;

	distorsionCoeff[0] = kc1;
	distorsionCoeff[1] = kc2;
	distorsionCoeff[2] = kc3;
	distorsionCoeff[3] = kc4;

	CameraCalibration calibration = CameraCalibration(fc1, fc2, cc1, cc2, distorsionCoeff);

	cv::Mat(3, 3, CV_32F,
		const_cast<float*>(&calibration.getIntrinsic().data[0]))
		.copyTo(_camMatrix);
	cv::Mat(4, 1, CV_32F,
		const_cast<float*>(&calibration.getDistorsion().data[0]))
		.copyTo(_distCoeff);
}

//计算相对位姿
void calcPosAtti(vector<cv::Point2f> points)
{
	assert(points.size() == 7);

	vector<cv::Point3f> _Dock3d;
	Transformation _trans;

	_Dock3d.clear();
	_Dock3d.push_back(cv::Point3f(0, 0, 0));
	_Dock3d.push_back(cv::Point3f(0, 0, 0));
	_Dock3d.push_back(cv::Point3f(0, 0, 0));
	_Dock3d.push_back(cv::Point3f(0, 0, 0));
	_Dock3d.push_back(cv::Point3f(0, 0, 0));
	_Dock3d.push_back(cv::Point3f(0, 0, 0));
	_Dock3d.push_back(cv::Point3f(0, 0, 0));

	Mat Rvec;
	Mat_<float> Tvec;
	Mat raux, taux;
	cv::solvePnP(_Dock3d, points, _camMatrix, _distCoeff, raux, taux);
	raux.convertTo(Rvec, CV_32F);
	taux.convertTo(Tvec, CV_32F);
	cv::Mat_<float> rotMat(3, 3);
	cv::Rodrigues(Rvec, rotMat);

	Transformation transformation;
	// Copy to transformation matrix
	for (int col = 0; col < 3; col++) {
		for (int row = 0; row < 3; row++) {
			transformation.r().mat[row][col] =
				rotMat(row, col); // Copy rotation component
		}
		transformation.t().data[col] = Tvec(col); // Copy translation component
	}

	// Since solvePnP finds camera location, w.r.t to marker pose, to get marker
	// pose w.r.t to the camera we invert it.
	transformation = transformation.getInverted();
	transformation.id() = 0;
	_trans = transformation;

	cout<<" x : "<< transformation.t().data[0];
	cout<<" y : "<< transformation.t().data[1];
	cout<<" z : "<< transformation.t().data[2]<<endl;
	cout<<endl;
}