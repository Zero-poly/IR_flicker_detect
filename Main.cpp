/*
This code is intended for academic use only.
You are free to use and modify the code, at your own risk.

If you use this code, or find it useful, please refer to the paper:

Michele Fornaciari, Andrea Prati, Rita Cucchiara,
A fast and effective ellipse detector for embedded vision applications
Pattern Recognition, Volume 47, Issue 11, November 2014, Pages 3693-3708, ISSN 0031-3203,
http://dx.doi.org/10.1016/j.patcog.2014.05.012.
(http://www.sciencedirect.com/science/article/pii/S0031320314001976)


The comments in the code refer to the abovementioned paper.
If you need further details about the code or the algorithm, please contact me at:

michele.fornaciari@unimore.it

last update: 23/12/2014
*/

//#include <cv.hpp>
#include <limits.h> /* PATH_MAX */
#include <stdlib.h>
#include <stdio.h>
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "EllipseDetectorYaed.h"
#include <fstream>


using namespace std;
using namespace cv;



// Should be checked
void SaveEllipses(const string& workingDir, const string& imgName, const vector<Ellipse>& ellipses /*, const vector<double>& times*/)
{
	string path(workingDir + "/" + imgName + ".txt");
	ofstream out(path, ofstream::out | ofstream::trunc);
	if (!out.good())
	{
		cout << "Error saving: " << path << endl;
		return;
	}

	// Save execution time
	//out << times[0] << "\t" << times[1] << "\t" << times[2] << "\t" << times[3] << "\t" << times[4] << "\t" << times[5] << "\t" << "\n";

	unsigned n = ellipses.size();
	// Save number of ellipses
	out << n << "\n";

	// Save ellipses
	for (unsigned i = 0; i < n; ++i)
	{
		const Ellipse& e = ellipses[i];
		out << e._xc << "\t" << e._yc << "\t" << e._a << "\t" << e._b << "\t" << e._rad << "\t" << e._score << "\n";
	}
	out.close();
}

// Should be checked
bool LoadTest(vector<Ellipse>& ellipses, const string& sTestFileName, vector<double>& times, bool bIsAngleInRadians = true)
{
	ifstream in(sTestFileName);
	if (!in.good())
	{
		cout << "Error opening: " << sTestFileName << endl;
		return false;
	}

	times.resize(6);
	in >> times[0] >> times[1] >> times[2] >> times[3] >> times[4] >> times[5];

	unsigned n;
	in >> n;

	ellipses.clear();

	if (n == 0) return true;

	ellipses.reserve(n);

	while (in.good() && n--)
	{
		Ellipse e;
		in >> e._xc >> e._yc >> e._a >> e._b >> e._rad >> e._score;

		if (!bIsAngleInRadians)
		{
			e._rad = e._rad * float(CV_PI / 180.0);
		}

		e._rad = fmod(float(e._rad + 2.0*CV_PI), float(CV_PI));

		if ((e._a > 0) && (e._b > 0) && (e._rad >= 0))
		{
			ellipses.push_back(e);
		}
	}
	in.close();

	// Sort ellipses by decreasing score
	sort(ellipses.begin(), ellipses.end());

	return true;
}


void LoadGT(vector<Ellipse>& gt, const string& sGtFileName, bool bIsAngleInRadians = true)
{
	ifstream in(sGtFileName);
	if (!in.good())
	{
		cout << "Error opening: " << sGtFileName << endl;
		return;
	}

	unsigned n;
	in >> n;

	gt.clear();
	gt.reserve(n);

	while (in.good() && n--)
	{
		Ellipse e;
		in >> e._xc >> e._yc >> e._a >> e._b >> e._rad;

		if (!bIsAngleInRadians)
		{
			// convert to radians
			e._rad = float(e._rad * CV_PI / 180.0);
		}

		if (e._a < e._b)
		{
			float temp = e._a;
			e._a = e._b;
			e._b = temp;

			e._rad = e._rad + float(0.5*CV_PI);
		}

		e._rad = fmod(float(e._rad + 2.f*CV_PI), float(CV_PI));
		e._score = 1.f;
		gt.push_back(e);
	}
	in.close();
}

bool TestOverlap(const Mat1b& gt, const Mat1b& test, float th)
{
	float fAND = float(countNonZero(gt & test));
	float fOR = float(countNonZero(gt | test));
	float fsim = fAND / fOR;

	return (fsim >= th);
}

int Count(const vector<bool> v)
{
	int counter = 0;
	for (unsigned i = 0; i < v.size(); ++i)
	{
		if (v[i]) { ++counter; }
	}
	return counter;
}

// Should be checked !!!!!
std::tuple<float, float, float> Evaluate(const vector<Ellipse>& ellGT, const vector<Ellipse>& ellTest, const float th_score, const Mat3b& img)
{
	float threshold_overlap = 0.8f;
	//float threshold = 0.95f;

	unsigned sz_gt = ellGT.size();
	unsigned size_test = ellTest.size();

	unsigned sz_test = unsigned(min(1000, int(size_test)));

	vector<Mat1b> gts(sz_gt);
	vector<Mat1b> tests(sz_test);

	for (unsigned i = 0; i < sz_gt; ++i)
	{
		const Ellipse& e = ellGT[i];

		Mat1b tmp(img.rows, img.cols, uchar(0));
		ellipse(tmp, Point(e._xc, e._yc), Size(e._a, e._b), e._rad * 180.0 / CV_PI, 0.0, 360.0, Scalar(255), -1);
		gts[i] = tmp;
	}

	for (unsigned i = 0; i < sz_test; ++i)
	{
		const Ellipse& e = ellTest[i];

		Mat1b tmp(img.rows, img.cols, uchar(0));
		ellipse(tmp, Point(e._xc, e._yc), Size(e._a, e._b), e._rad * 180.0 / CV_PI, 0.0, 360.0, Scalar(255), -1);
		tests[i] = tmp;
	}

	Mat1b overlap(sz_gt, sz_test, uchar(0));
	for (int r = 0; r < overlap.rows; ++r)
	{
		for (int c = 0; c < overlap.cols; ++c)
		{
			overlap(r, c) = TestOverlap(gts[r], tests[c], threshold_overlap) ? uchar(255) : uchar(0);
		}
	}

	int counter = 0;

	vector<bool> vec_gt(sz_gt, false);

	for (int i = 0; i < sz_test; ++i)
	{
		const Ellipse& e = ellTest[i];
		for (int j = 0; j < sz_gt; ++j)
		{
			if (vec_gt[j]) { continue; }

			bool bTest = overlap(j, i) != 0;

			if (bTest)
			{
				vec_gt[j] = true;
				break;
			}
		}
	}

	int tp = Count(vec_gt);
	int fn = int(sz_gt) - tp;
	int fp = size_test - tp; // !!!!

	float pr(0.f);
	float re(0.f);
	float fmeasure(0.f);

	if (tp == 0)
	{
		if (fp == 0)
		{
			pr = 1.f;
			re = 0.f;
			fmeasure = (2.f * pr * re) / (pr + re);
		}
		else
		{
			pr = 0.f;
			re = 0.f;
			fmeasure = 0.f;
		}
	}
	else
	{
		pr = float(tp) / float(tp + fp);
		re = float(tp) / float(tp + fn);
		fmeasure = (2.f * pr * re) / (pr + re);
	}

	return make_tuple(pr, re, fmeasure);
}

vector<Ellipse> OnImage(Mat image)
{
    //Mat image=imread("/home/liuu/Pictures/IR_detect_5/1576640196.04.jpg",1);
    //Mat image=imread("/home/liuu/Pictures/IR_detect_5/1576640196.29.jpg",1);
    Mat1b gray;
    cvtColor(image, gray, CV_BGR2GRAY);
    Size sz=image.size();
    threshold(gray,gray,30,255,THRESH_BINARY_INV);
    Mat element=getStructuringElement(MORPH_RECT,Size(3,3));
    morphologyEx(gray,gray,MORPH_CLOSE,element);
//    imshow("close",gray);

    // Parameters Settings (Sect. 4.2)
    int		iThLength = 16;
    float	fThObb = 3.0f;
    float	fThPos = 1.0f;
    float	fTaoCenters = 0.05f;
    int 	iNs = 16;
    float	fMaxCenterDistance = sqrt(float(sz.width*sz.width + sz.height*sz.height)) * fTaoCenters;

    float	fThScoreScore = 0.6f;

    // Other constant parameters settings.

    // Gaussian filter parameters, in pre-processing
    Size	szPreProcessingGaussKernelSize = Size(5, 5);
    double	dPreProcessingGaussSigma = 1.0;

    float	fDistanceToEllipseContour = 0.1f;	// (Sect. 3.3.1 - Validation)
    float	fMinReliability = 0.5;	// Const parameters to discard bad ellipses


    CEllipseDetectorYaed* yaed = new CEllipseDetectorYaed();
    yaed->SetParameters(szPreProcessingGaussKernelSize,
        dPreProcessingGaussSigma,
        fThPos,
        fMaxCenterDistance,
        iThLength,
        fThObb,
        fDistanceToEllipseContour,
        fThScoreScore,
        fMinReliability,
        iNs
        );


    // Detect
    vector<Ellipse> ellsYaed;
    Mat1b gray2 = gray.clone();
    yaed->Detect(gray2, ellsYaed);

    vector<double> times = yaed->GetTimes();
    cout << "--------------------------------" << endl;
    cout << "Execution Time: " << endl;
    cout << "Edge Detection: \t" << times[0] << endl;
    cout << "Pre processing: \t" << times[1] << endl;
    cout << "Grouping:       \t" << times[2] << endl;
    cout << "Estimation:     \t" << times[3] << endl;
    cout << "Validation:     \t" << times[4] << endl;
    cout << "Clustering:     \t" << times[5] << endl;
    cout << "--------------------------------" << endl;
    cout << "Total:	         \t" << yaed->GetExecTime() << endl;
    cout << "--------------------------------" << endl;



    Mat3b resultImage = image.clone();
    yaed->DrawDetectedEllipses(resultImage, ellsYaed);

    imshow("Yaed", resultImage);
    return ellsYaed;
}

#include <algorithm>
void NTSS(Mat big,Mat small)
{
    assert(big.cols>(small.cols+16));
    assert(big.rows>(small.rows+16));

    int x_c=int(float(big.cols)/2.0);
    int y_c=int(float(big.rows)/2.0);
    int count=0;
    int w=small.cols, h=small.rows;

    while(count<3)
    {
        Mat roi,diff;
        roi=big(Rect(x_c-w/2.0,y_c-h/2.0,w,h));
        absdiff(roi,small,diff);
        threshold(diff,diff,100,255,THRESH_BINARY);

        vector<int> scores;
        scores.push_back(countNonZero(diff));

        vector<int>::iterator smallest = min_element(begin(scores), end(scores));
        int pos=distance(begin(scores), smallest);

        switch (pos) {
            case 0:
            break;
            case 1:
                x_c+=-1,y_c+=-1;
            break;
            case 2:
                x_c+=0,y_c+=-1;
            break;
            case 3:
                x_c+=1,y_c+=-1;
            break;
            case 4:
                x_c+=-1,y_c+=0;
            break;
            case 5:
                x_c+=1,y_c+=0;
            break;
            case 6:
                x_c+=-1,y_c+=1;
            break;
        }
    }
}


int main()
{
    Mat frame_1=imread("C:\\Users\\LIUU\\Pictures\\IR_detect_5\\1576640196.04.jpg",1);
    Mat frame_2=imread("C:\\Users\\LIUU\\Pictures\\IR_detect_5\\1576640196.29.jpg",1);

//    Mat frame_1=imread("/home/liuu/Pictures/IR_detect_5/1576640165.54.jpg",1);
//    Mat frame_2=imread("/home/liuu/Pictures/IR_detect_5/1576640165.79.jpg",1);

//    Mat frame_1=imread("/home/liuu/Pictures/IR_detect_5/1576640151.54.jpg",1);
//    Mat frame_2=imread("/home/liuu/Pictures/IR_detect_5/1576640151.79.jpg",1);

    imshow("frame_1",frame_1);
    imshow("frame_2",frame_2);
    waitKey(0);
    Mat Diff;
    absdiff(frame_1,frame_2,Diff);
    imshow("Diff",Diff);
    waitKey(0);

    vector<Ellipse> ells_1=OnImage(frame_1);
    vector<Ellipse> ells_2=OnImage(frame_2);
    if(ells_1.size()==0 || ells_2.size()==0)
    {
        cout<<"at least one image no ellipse..."<<endl;
        return 0;
    }

    for(int i=0;i<ells_1.size();i++)
    {
        for(int j=0;j<ells_2.size();j++)
        {
            Ellipse ell_1=ells_1[i],ell_2=ells_2[j];
            int r_1=max(ell_1._a,ell_2._b);
            int r_2=max(ell_2._a,ell_2._b);
            int r=max(r_1,r_2);
            Rect rect_1=Rect(ell_1._xc-r,ell_1._yc-r,2*r,2*r);
            Rect rect_2=Rect(ell_2._xc-r,ell_2._yc-r,2*r,2*r);
            Mat roi_1=frame_1(rect_1),roi_2=frame_2(rect_2);
            imshow("roi_1",roi_1);
            imshow("roi_2",roi_2);
            waitKey(0);
            Mat diff;
            absdiff(roi_1,roi_2,diff);
            imshow("diff",diff);
            waitKey(0);
        }
    }
}