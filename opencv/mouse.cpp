#include <iostream>
#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
using namespace cv;
using namespace std;

/*定义回调函数
  int x,int y，代表鼠标位于窗口的（x，y）坐标位置，窗口左上角默认为原点，向右为x轴，向下为y轴
  int event，鼠标操作时间的整数代号，在opencv中，event鼠标事件总共有10中，从0-9依次代表如下:
	#define CV_EVENT_MOUSEMOVE 0             滑动
	#define CV_EVENT_LBUTTONDOWN 1           左键点击
	#define CV_EVENT_RBUTTONDOWN 2           右键点击
	#define CV_EVENT_MBUTTONDOWN 3           中间点击
	#define CV_EVENT_LBUTTONUP 4             左键释放
	#define CV_EVENT_RBUTTONUP 5             右键释放
	#define CV_EVENT_MBUTTONUP 6             中间释放
	#define CV_EVENT_LBUTTONDBLCLK 7         左键双击
	#define CV_EVENT_RBUTTONDBLCLK 8         右键双击
	#define CV_EVENT_MBUTTONDBLCLK 9         中间释放
  int flags，代表鼠标的拖拽事件，以及键盘鼠标联合事件，依次如下：（多种组合）
	#define CV_EVENT_FLAG_LBUTTON 1           左键不放
	#define CV_EVENT_FLAG_RBUTTON 2           右键不放
	#define CV_EVENT_FLAG_MBUTTON 4           中间不放
	#define CV_EVENT_FLAG_CTRLKEY 8           按Ctrl不放
	#define CV_EVENT_FLAG_SHIFTKEY 16         按Shift不放
	#define CV_EVENT_FLAG_ALTKEY   32         按Alt不放

*/
void onMouse(int event, int x, int y, int flags, void *param)
{
	Mat *im = reinterpret_cast<Mat *>(param);
	switch (event)
	{

	case CV_EVENT_LBUTTONDOWN: //左键点击
	{
		Vec3b pixel = im->at<Vec3b>(y, x); // 向右为x轴，向下为y轴，因此y是行，x是列； 按B,G,R输出
		cout << "at(" << x << "," << y << ")->(B,G,R)=" << pixel << endl;
		break;
	}
	case CV_EVENT_RBUTTONDOWN: //右键点击
	{
		cout << "向右为x轴,x =" << endl;
		cin >> x;
		cout << "向下为y轴,y =" << endl;
		cin >> y;
		Vec3b pixel = im->at<Vec3b>(y, x); // 向右为x轴，向下为y轴，因此y是行，x是列； 按B,G,R输出
		cout << "at(" << x << "," << y << ")->(B,G,R)=" << pixel << endl;
		break;
	}
	case CV_EVENT_MOUSEMOVE: // 移动鼠标
	{
		if (!(flags & CV_EVENT_FLAG_LBUTTON)) //左键没有按下时鼠标移动
		{
		}
		else if (flags & CV_EVENT_FLAG_LBUTTON) //左键按下时鼠标移动
		{
		}
		break;
	}
	default:
		break;
	}
}

int main()
{
	Mat image;
	image = imread("../data/caigou.jpg", CV_LOAD_IMAGE_COLOR); // 按ＢＧＲ读入，不是ＲＧＢ
	namedWindow("mywindow", WINDOW_AUTOSIZE);
	cv::setMouseCallback("mywindow", onMouse, reinterpret_cast<void *>(&image)); // 注册鼠标处理函数放在窗口定义之后
	imshow("mywindow", image);
	waitKey(0);
}