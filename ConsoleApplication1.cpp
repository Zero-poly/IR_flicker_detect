// ConsoleApplication1.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

//拍摄图片并保存
void shootImage()
{
    VideoCapture capture;
    capture.open(0);  //笔记本自带摄像头编号是0，usb摄像头编号是1
    //默认是自动曝光，要修改成手动曝光,0.25 means "manual exposure, manual iris"
    capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
    //
    capture.set(CV_CAP_PROP_EXPOSURE, 0.01);
    if (!capture.isOpened())
        cout << "Failed to open camera" << endl;
    namedWindow("Window");

    Mat frame;
    while (capture.isOpened())
    {
        capture >> frame;
        time_t timep = time(NULL);
        string str = to_string(timep);
        imshow("Window", frame);
        imwrite("C:\\Users\\LIUU\\Pictures\\IR_detect_4\\" + str + ".png", frame);
        waitKey(1);
    }
}

int main_1()
{
    shootImage();
    waitKey(0);
    return 0;
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门使用技巧: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
