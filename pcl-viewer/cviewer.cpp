#include <iostream>
#include <pcl/io/pcd_io.h>                  //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h>                //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h> //可视化头文件//读取点云文件，及其错误处理
#include <stdio.h>
#include <unistd.h>

int main(int argc, char **argv)
{
  // 创建 viewer 指针对象
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

  // 设置背景颜色
  viewer->setBackgroundColor(0, 0, 0);

  //加载pcd点云
  std::string filename;
  if (argv[1] == NULL)
  {
    filename = getcwd(NULL, 0);
    filename += "/../../example_data/bin.pcd";
    cout << "文件路径" << filename << endl;
  }
  else
  {
    filename = argv[1];
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloud) == -1)
  {
    PCL_ERROR("Couldn't read file !\n");
    return (-1);
  }
  // //打印出总的点云数
  // std::cout << "Loaded "
  //           << cloud->width * cloud->height // 115384*1
  //           << " data points from test_file.pcd with the following fields: "
  //           << std::endl;
  // //打印出每一个点的值
  // for (size_t i = 0; i < cloud->points.size(); ++i) // size = 115384
  //   std::cout << " " << cloud->points[i].x
  //             << " " << cloud->points[i].y
  //             << " " << cloud->points[i].z << std::endl;

  // 随机颜色
  // pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> random_color(cloud);
  // 定制颜色
  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> custom_color(cloud, 0, 0, 255);

  // 添加点云数据到viewer，并有颜色
  // viewer->addPointCloud<pcl::PointXYZ>(cloud, random_color, "sample cloud");
  // 添加点云数据到viewer
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");

  // 设置点云的显示（渲染）大小
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sample cloud");

  // 显示坐标轴，并设置比例
  viewer->addCoordinateSystem(1.0);

  //展示读取的点云对象
  while (!viewer->wasStopped())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}
