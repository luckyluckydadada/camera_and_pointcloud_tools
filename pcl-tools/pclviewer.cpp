#include <iostream> //标准输入输出流
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h> //可视化头文件
int main(int argc, char** argv){
 pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);   // 创建点云对象（指针）
 if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/lucky/d/kitti/training/velodyne_pcd/000000.pcd", *cloud) == -1) 
 {
  PCL_ERROR("Couldn't read file test_pcd.pcd \n"); 
  return (-1);
 } //读取点云文件，及其错误处理 
 std::cout << "Loaded "
  << cloud->width * cloud->height  // 115384*1
  << " data points from test_file.pcd with the following fields: "
  << std::endl;  //打印出总的点云数
 for (size_t i = 0; i < cloud->points.size (); ++i)  // size = 115384 
  std::cout << " " << cloud->points[i].x
  << " " << cloud->points[i].y
  << " " << cloud->points[i].z << std::endl; //打印出每一个点的值
 pcl::visualization::CloudViewer viewer("pcd viewer");  //创建可视化窗口
 viewer.showCloud(cloud); //展示读取的点云对象
 while (!viewer.wasStopped())
    {
    } //窗口不闪退
  return (0);
}
