#include "myAlgorithm.h"

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


//内圈三个点排序
vector<cv::Point2f> IR_detect::innerSort(vector<cv::Point2f> inner, cv::Point2f center)
{
	assert(inner.size() == 3);
	vector<cv::Point2f> new_inner(3);

	int i = 0;
	for (; i < 3; i++)
	{
		float delta_1 = abs(inner[i].x + inner[(i + 1) % 3].x - 2 * center.x) + abs(inner[i].y + inner[(i + 1) % 3].y - 2 * center.y);
		float delta_2 = abs(inner[i].x + inner[(i + 2) % 3].x - 2 * center.x) + abs(inner[i].y + inner[(i + 2) % 3].y - 2 * center.y);
		if (delta_1 > 10 && delta_2 > 10)
		{
			new_inner[0] = inner[i];
			break;
		}
	}

	float angle = cvFastArctan(new_inner[0].y - center.y, new_inner[0].x - center.x);
	float delta_x = inner[(i + 1) % 3].x - inner[(i + 2) % 3].x;
	float delta_y = inner[(i + 1) % 3].y - inner[(i + 2) % 3].y;

	if (angle >= 45 && angle < 135)
	{
		if (delta_x < 0)
		{
			new_inner[1] = inner[(i + 1) % 3];
			new_inner[2] = inner[(i + 2) % 3];
		}
		else
		{
			new_inner[1] = inner[(i + 2) % 3];
			new_inner[2] = inner[(i + 1) % 3];
		}
	}

	if (angle >= 135 && angle < 225)
	{
		if (delta_y < 0)
		{
			new_inner[1] = inner[(i + 1) % 3];
			new_inner[2] = inner[(i + 2) % 3];
		}
		else
		{
			new_inner[1] = inner[(i + 2) % 3];
			new_inner[2] = inner[(i + 1) % 3];
		}
	}

	if (angle >= 225 && angle < 315)
	{
		if (delta_x > 0)
		{
			new_inner[1] = inner[(i + 1) % 3];
			new_inner[2] = inner[(i + 2) % 3];
		}
		else
		{
			new_inner[1] = inner[(i + 2) % 3];
			new_inner[2] = inner[(i + 1) % 3];
		}
	}

	if (angle >= 315 || angle < 45)
	{
		if (delta_y > 0)
		{
			new_inner[1] = inner[(i + 1) % 3];
			new_inner[2] = inner[(i + 2) % 3];
		}
		else
		{
			new_inner[1] = inner[(i + 2) % 3];
			new_inner[2] = inner[(i + 1) % 3];
		}
	}
	/*vector<float> angels;
	for (int i = 0; i < 3; i++)
	{
		float ang = cvFastArctan(inner[i].y - center.y, inner[i].x - center.x);
		angels.push_back(ang);
	}


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
	}*/

	return new_inner;
}

bool compare_1(cv::Point2f a, cv::Point2f b)
{
	return a.x < b.x;
}

//外圈四个点排序
vector<cv::Point2f> IR_detect::outerSort(cv::Point2f first_pt, cv::Point2f second_pt, vector<cv::Point2f> outer)
{
	assert(outer.size() == 4);

	cv::Point2f distance[4];
	for (int i = 0; i < 4; i++)
	{
		float x = sqrt(pow(outer[i].x - first_pt.x, 2) + pow(outer[i].y - first_pt.y, 2));
		float y = i;
		distance[i] = cv::Point2f(x, y);
	}
	sort(distance, distance + 4, compare_1);

	float dis_1 = sqrt(pow(outer[distance[0].y].x - second_pt.x, 2) + pow(outer[distance[0].y].y - second_pt.y, 2));
	float dis_2 = sqrt(pow(outer[distance[1].y].x - second_pt.x, 2) + pow(outer[distance[1].y].y - second_pt.y, 2));

	int first_pos;
	if (dis_1 < dis_2)
		first_pos = int(distance[0].y);
	else
		first_pos = int(distance[1].y);

	vector<cv::Point2f> new_outer(4);
	new_outer[0] = outer[first_pos % 4];
	new_outer[1] = outer[(first_pos + 1) % 4];
	new_outer[2] = outer[(first_pos + 2) % 4];
	new_outer[3] = outer[(first_pos + 3) % 4];

	return new_outer;
}

//对七个斑点进行排序
vector<cv::Point2f> IR_detect::sortPoints(vector<cv::Point2f> points)
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

//对裁剪后的帧差图提取圆斑，后排除干扰点以及排序
void IR_detect::processCutDiff( )
{
	assert(open_diff.channels() == 1);

	//计算各斑点质心，用于排序和特征向量计算
	vector<vector<cv::Point>> contours;
	cv::findContours(open_diff, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	vector<cv::Point2f> centers;
	for (int j = 0; j < contours.size(); j++)
	{
		cv::Moments mom = cv::moments(contours[j], true);
		cv::Point2f center = cv::Point2f(float(mom.m10 / mom.m00), float(mom.m01 / mom.m00));
		centers.push_back(center);
	}

	//排除干扰点
	//---TODO--

	//对排除干扰点后的7个特征点排序
	sort_points = sortPoints(centers);

	if(isShow)
		for (int i = 0; i < sort_points.size(); i++)
		{
			cv::circle(open_diff, sort_points[i], 5, cv::Scalar(255));
			cv::imshow("sort", open_diff);
			cv::waitKey(0);
		}
}


void IR_detect::initCalibration()
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
void IR_detect::calcPosAtti(vector<cv::Point2f> points)
{
	assert(points.size() == 7);

	vector<cv::Point3f> _Dock3d;
	Transformation _trans;

	_Dock3d.clear();
	_Dock3d.push_back(cv::Point3f(0.74, -8.93, -16.5));
	_Dock3d.push_back(cv::Point3f(9.67, -0.37, -16.5));
	_Dock3d.push_back(cv::Point3f(-9.67, 0.37, -16.5));
	_Dock3d.push_back(cv::Point3f(11.91, -9.67, 0));
	_Dock3d.push_back(cv::Point3f(10.79, 11.91, 0));
	_Dock3d.push_back(cv::Point3f(-11.16, 11.91, 0));
	_Dock3d.push_back(cv::Point3f(-10.79, -10.79, 0));

	cv::Mat Rvec;
	cv::Mat_<float> Tvec;
	cv::Mat raux, taux;
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

	cout << " x : " << transformation.t().data[0];
	cout << " y : " << transformation.t().data[1];
	cout << " z : " << transformation.t().data[2] << endl;
	cout << endl;
}


//锥套模型候选区域的检测
vector<cv::Rect> IR_detect::getCandiAreas(cv::Mat raw_image)
{
	//cv::Mat image = cv::imread("C:\\Users\\LIUU\\Pictures\\IR_detect_6\\1577763332.03.jpg", 1);
	//cv::Mat image = cv::imread("C:\\Users\\LIUU\\Pictures\\IR_detect_6\\1577763332.28.jpg", 1);
	cv::Mat image = raw_image.clone();
	if (image.channels() == 3)
		cv::cvtColor(image, image, CV_BGR2GRAY);
	//cv::imshow("image", image);

	cv::threshold(image, image, 50, 255, cv::THRESH_BINARY_INV);
	//cv::imshow("threshold", image);

	//cv::Mat element = cv::getStructuringElement(MORPH_RECT,Size(3, 3));
	//cv::morphologyEx(image, image, MORPH_CLOSE, element);
	//cv::imshow("close", image);

	//cv::waitKey(0);

	//cv::Mat canny;
	//cv::Canny(image, canny, 30, 100);
	//cv::imshow("canny", canny);

	vector<vector<cv::Point>> contours;
	cv::findContours(image, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);

	cv::Mat draw_paper = cv::Mat::zeros(image.size(), CV_8UC1);
	vector<cv::Rect> rects;
	for (int i = 0; i < contours.size(); i++)
	{
		//轮廓长度判定
		double len = cv::arcLength(contours[i], 1);

		if (len < 60)
			continue;

		//轮廓形状判定
		cv::RotatedRect rot_rect = cv::minAreaRect(contours[i]);
		double r = double(rot_rect.size.height + rot_rect.size.width) / 4;

		if (double(rot_rect.size.height) / rot_rect.size.width < 0.85 || double(rot_rect.size.height) / rot_rect.size.width >1.15)
			continue;
		if (2 * CV_PI * r / len < 0.65 || 2 * CV_PI * r / len >1.1)
			continue;

		//轮廓中心点是否在轮廓内部判定
		cv::Moments mom = cv::moments(contours[i], true);
		cv::Point2f center = cv::Point2f(float(mom.m10 / mom.m00), float(mom.m01 / mom.m00));

		double inContour = cv::pointPolygonTest(contours[i], center, false);
		if (inContour != 1)
			continue;


		//cout << double(rect.height) / rect.width << endl;
		//cout << 2 * CV_PI * r / len << endl;
		cv::Rect rect = cv::boundingRect(contours[i]);
		rects.push_back(rect);
		cv::drawContours(draw_paper, contours, i, cv::Scalar(255));
		cv::circle(draw_paper, center, 3, cv::Scalar(255), -1);
		//imshow("contours", draw_paper);
		//cv::waitKey(0);
	}
	//imshow("contours", draw_paper);
	//cv::waitKey(0);
	return rects;
}


void IR_detect::getDiffImage()
{
	cv::Mat image_1 = cv::imread("C:\\Users\\LIUU\\Pictures\\IR_detect_6\\1577763339.53.jpg", 0);
	cv::Mat image_2 = cv::imread("C:\\Users\\LIUU\\Pictures\\IR_detect_6\\1577763339.78.jpg", 0);

	if (isShow)
	{
		cv::Mat Diff;
		cv::absdiff(image_1, image_2, Diff);
		imshow("src_Diff", Diff);
	}

	cv::Mat img_1 = image_1.clone();
	cv::Mat img_2 = image_2.clone();

	vector<cv::Rect> rects_1, rects_2;
	rects_1 = getCandiAreas(image_1);
	rects_2 = getCandiAreas(image_2);

	assert(rects_1.size() > 0 && rects_2.size() > 0);

	for (int i = 0; i < rects_1.size(); i++)
	{
		for (int j = 0; j < rects_2.size(); j++)
		{
			//vector<int> size = { rects_1[i].height,rects_1[i].width,rects_2[j].height,rects_2[j].width };
			//vector<int>::iterator iter= max_element(size.begin(), size.end());
			//int R = (*iter) / 2+1;

			cv::Mat area_1 = img_1(rects_1[i]);
			cv::Mat area_2 = img_2(rects_2[j]);

			if (double(rects_1[i].area()) / rects_2[j].area() > 0.9 && double(rects_1[i].area()) / rects_2[j].area() < 1.1)
			{
				cv::Mat diff;
				cv::resize(area_1, area_1, area_2.size());
				cv::absdiff(area_1, area_2, diff);

				if(isShow)
					imshow("diff", diff);
			}
		}
	}
}


//LOG核函数生成
cv::Mat getLOGKernel(cv::Size& ksize, double sigma)
{
	cv::Mat kernel(ksize, CV_64F);
	cv::Point centPoint = cv::Point((ksize.width - 1) / 2, ((ksize.height - 1) / 2));
	// first calculate Gaussian
	for (int i = 0; i < kernel.rows; i++)
	{
		double* pData = kernel.ptr<double>(i);
		for (int j = 0; j < kernel.cols; j++)
		{
			double param = -((i - centPoint.y) * (i - centPoint.y) + (j - centPoint.x) * (j - centPoint.x)) / (2 * sigma * sigma);
			pData[j] = exp(param);
		}
	}
	double maxValue;
	minMaxLoc(kernel, NULL, &maxValue);
	double EPS = 0.1;
	for (int i = 0; i < kernel.rows; i++)
	{
		double* pData = kernel.ptr<double>(i);
		for (int j = 0; j < kernel.cols; j++)
		{
			if (pData[j] < EPS * maxValue)
			{
				pData[j] = 0;
			}
		}
	}
	double sumKernel = sum(kernel)[0];
	if (sumKernel != 0)
	{
		kernel = kernel / sumKernel;
	}
	// now calculate Laplacian
	for (int i = 0; i < kernel.rows; i++)
	{
		double* pData = kernel.ptr<double>(i);
		for (int j = 0; j < kernel.cols; j++)
		{
			double addition = ((i - centPoint.y) * (i - centPoint.y) + (j - centPoint.x) * (j - centPoint.x) - 2 * sigma * sigma) / (sigma * sigma * sigma * sigma);
			pData[j] *= addition;
		}
	}
	// make the filter sum to zero
	sumKernel = sum(kernel)[0];
	kernel -= (sumKernel / (ksize.width * ksize.height));
	return kernel;
}


bool compare_2(cv::Point3f a, cv::Point3f b)
{
	return a.y < b.y;
}
//阵列数组相似度检测
vector<cv::Point3f> IR_detect::getFeatureVec(vector<cv::Point2f> centers, cv::Point2f image_center)
{
	vector<cv::Point3f> feature_vec;
	float min_len = 1000, min_theta = 1000;
	for (int i = 0; i < centers.size(); i++)
	{
		float len = sqrt(pow(centers[i].x - image_center.x, 2) + pow(centers[i].y - image_center.y, 2));
		float theta = cvFastArctan(centers[i].y - image_center.y, centers[i].x - image_center.x);
		feature_vec.push_back(cv::Point3f(len, theta, i));
		if (len < min_len)
			min_len = len;
		if (theta < min_theta)
			min_theta = theta;
	}
	sort(feature_vec.begin(), feature_vec.end(), compare_2);

	for (int i = 0; i < feature_vec.size(); i++)
	{
		feature_vec[i].y -= min_theta;
		feature_vec[i].x /= min_len;
		cout << feature_vec[i].x << "  " << feature_vec[i].y << "  " << feature_vec[i].z << endl;
	}

	return feature_vec;
}

//计算两个二维数组相关性
double IR_detect::calSimilarity(vector<cv::Point3f> vec_1, vector<cv::Point3f> vec_2)
{
	assert(vec_1.size() == vec_2.size());

	int rows = vec_1.size();
	cv::Mat mat_1(rows, 2, CV_32F, cv::Scalar(0)), mat_2(rows, 2, CV_32F, cv::Scalar(0));
	for (int i = 0; i < vec_1.size(); i++)
	{
		mat_1.at<float>(i, 0) = vec_1[i].x;
		mat_1.at<float>(i, 1) = vec_1[i].y;

		mat_2.at<float>(i, 0) = vec_2[i].x;
		mat_2.at<float>(i, 1) = vec_2[i].y;
	}

	return cv::compareHist(mat_1, mat_2, CV_COMP_CHISQR);
}


//帧差法图像裁剪，去除锥套内环的环境干扰
void IR_detect::cutInnerRing( )
{
	int i = 0;
	vector<cv::Point3f> feature_vec_0 ;
	vector<cv::Point3f> feature_vec_1;

	assert(diff.channels() == 1);

	if (isShow)
		cv::imshow("src_diff", diff);

	//图像掩膜排除环境干扰
	cv::Mat mask = cv::Mat::zeros(diff.size(), CV_8UC1);
	double R = min(diff.cols, diff.rows) / 2;
	cv::circle(mask, cv::Point((diff.cols - 1) / 2, (diff.rows - 1) / 2), R, cv::Scalar(255), -1);
	double r = max(diff.cols, diff.rows) * 0.4267 / 2;
	cv::circle(mask, cv::Point(diff.cols / 2 - 1, diff.rows / 2 - 1), r, cv::Scalar(0), -1);
	cv::bitwise_and(diff, mask, diff);

	if (isShow)
		cv::imshow("cut", diff);

	cut_diff = diff.clone();

	//利用椭圆核进行开运算提取圆形斑点
	cv::resize(diff, diff, cv::Size(60, 60));
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
	cv::morphologyEx(diff, diff, cv::MORPH_OPEN, element);
	cv::threshold(diff, diff, 100, 255, cv::THRESH_BINARY);

	open_diff = diff.clone();
	if (isShow)
		cv::imshow("open", diff);


	//cv::threshold(diff, diff, 100, 255, CV_THRESH_OTSU);
	//cv::imshow("threshold", diff);

	//cv::Mat kernel = getLOGKernel(cv::Size(3, 3), 3);
	//cv::filter2D(diff, diff, CV_64F, kernel);
	//cv::imshow("LOG", diff);
}