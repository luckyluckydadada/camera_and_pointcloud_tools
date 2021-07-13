#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <opencv2/highgui.hpp>
using namespace cv;
using namespace std;

//RGB colour visualisation example
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	//创建3D窗口并添加点云
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

void get_calib(char *calib_string, float *calib_arr, int len)
{
	const char *delim = " ";
	char *p = strtok(calib_string, delim); // fist element delete
	for (int i = 0; i < len; i++)
	{
		p = strtok(NULL, delim);
		calib_arr[i] = atof(p);
	}
}

// -----Main-----
int main(int argc, char **argv)
{
	// 解析命令行参数
	if (argc != 5)
	{
		cout << "Usage : xx yy.pcd zz.png calib.txt" << endl;
		//return 0;
	}

	clock_t start, end;
	start = clock();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile(argv[1], *point_cloud_ptr);

	Mat img = imread(argv[2]);
	if (img.channels() != 3)
	{
		cout << "RGB pics needed." << endl;
		return 0;
	}

	int rows = img.rows;
	int cols = img.cols;
	unsigned char red, green, blue;
	float p_u, p_v, p_w;	  //pics_uv1;(u for cols, v for lines!!!)
	float c_x, c_y, c_z, c_i; //clouds_xyz、intensity;

	ifstream cal_file;
	cal_file.open(argv[3]);
	int i;
	char buf[512];
	float calib[16];

	// P2
	for (i = 0; i < 3; i++)
	{
		cal_file.getline(buf, sizeof(buf));
	}
	get_calib(buf, calib, 12);
	Mat P2 = (Mat_<float>(3, 4) << calib[0], calib[1], calib[2], calib[3], calib[4], calib[5], calib[6], calib[7], calib[8], calib[9], calib[10], calib[11]);

	// R0_rect
	for (; i < 5; i++)
	{
		cal_file.getline(buf, sizeof(buf));
	}
	// cout<<buf<<endl;
	get_calib(buf, calib, 9);
	Mat R0_rect = (Mat_<float>(4, 4) << calib[0], calib[1], calib[2], 0, calib[3], calib[4], calib[5], 0, calib[6], calib[7], calib[8], 0, 0, 0, 0, 1);
	// cout<<buf<<endl;
	// cout<<calib[8]<<endl;

	// Tr_velo_to_cam
	for (; i < 6; i++)
	{
		cal_file.getline(buf, sizeof(buf));
	}
	// cout<<buf<<endl;
	get_calib(buf, calib, 12);
	Mat Tr_velo_to_cam = (Mat_<float>(4, 4) << calib[0], calib[1], calib[2], calib[3], calib[4], calib[5], calib[6], calib[7], calib[8], calib[9], calib[10], calib[11], 0, 0, 0, 1);
	// cout<<buf<<endl;
	// cout<<calib[8]<<endl;

	Mat trans = Mat(3, 4, CV_32FC1);
	trans = P2 * R0_rect * Tr_velo_to_cam;
	Mat c_tmp = Mat(4, 1, CV_32FC1);
	Mat p_result = Mat(1, 3, CV_32FC1);
	// cout << "trans = " << trans << endl;

	//   ofstream outfile;
	//   outfile.open("sout.txt");
	for (int nIndex = 0; nIndex < point_cloud_ptr->points.size(); nIndex++)
	{
		c_x = point_cloud_ptr->points[nIndex].x;
		c_y = point_cloud_ptr->points[nIndex].y;
		c_z = point_cloud_ptr->points[nIndex].z;

		c_tmp = (Mat_<float>(4, 1) << c_x, c_y, c_z, 1);
		p_result = trans * c_tmp;

		p_w = p_result.at<float>(0, 2);
		p_u = (int)((p_result.at<float>(0, 0)) / p_w);
		p_v = (int)((p_result.at<float>(0, 1)) / p_w);
		if ((p_u < 0) || (p_u > cols) || (p_v < 0) || (p_v > rows) || (p_w < 0))
		{
			point_cloud_ptr->points[nIndex].r = 128;
			point_cloud_ptr->points[nIndex].g = 2;
			point_cloud_ptr->points[nIndex].b = 64;
			continue;
		}
		point_cloud_ptr->points[nIndex].r = img.at<Vec3b>(p_v, p_u)[2]; //not (p_u,p_v)!
		point_cloud_ptr->points[nIndex].g = img.at<Vec3b>(p_v, p_u)[1];
		point_cloud_ptr->points[nIndex].b = img.at<Vec3b>(p_v, p_u)[0];

		// ouput colour point cloud
		int r = point_cloud_ptr->points[nIndex].r, g = point_cloud_ptr->points[nIndex].g, b = point_cloud_ptr->points[nIndex].b;
		float x = point_cloud_ptr->points[nIndex].x, y = point_cloud_ptr->points[nIndex].y, z = point_cloud_ptr->points[nIndex].z;
		// cout<<x<<" "<<y<<" "<<z<<" "<<r<<" "<<g<<" "<<b<<endl;
		// outfile<<x<<" "<<y<<" "<<z<<" "<<r<<" "<<g<<" "<<b<<endl;
	}
	end = clock();
	cout << (double)(end - start)/CLOCKS_PER_SEC <<"s";
	// outfile.close();
	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	// viewer = rgbVis(point_cloud_ptr);
	// // 主循环
	// while (!viewer->wasStopped())
	// {
	//     viewer->spinOnce(100);
	//     boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	// }
}
