#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <liblas/liblas.hpp> //sudo apt install liblas-dev liblas-c-dev
#include <iomanip>
#include <iostream>
#include <sstream>
#include <cmath>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <string>

//  实现Unshort转为Unchar类型
//int Unshort2Unchar(uint16_t &green, uint8_t &g)
//{
//    unsigned short * word;
//    word = &green;
//    int size = WideCharToMultiByte(CP_ACP, 0, LPCWSTR(word), -1, NULL, 0, NULL, FALSE);
//    char *AsciiBuff=new char[size];
//    WideCharToMultiByte(CP_ACP, 0, LPCWSTR(word), -1, AsciiBuff, size, NULL, FALSE);
//    g = *AsciiBuff;
//
//    delete[] AsciiBuff;
//    AsciiBuff = NULL;
//    return 0;
//}

void writePointCloudFromLas(const char *strInputLasName, const char *strOutPutPointCloudName)
{
  //打开las文件
  std::ifstream ifs;
  ifs.open(strInputLasName, std::ios::in | std::ios::binary);
  if (!ifs.is_open())
  {
    std::cout << "无法打开.las" << std::endl;
    return;
  }
  liblas::ReaderFactory readerFactory;
  liblas::Reader reader = readerFactory.CreateWithStream(ifs);
  //写点云
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudOutput(new pcl::PointCloud<pcl::PointXYZRGBA>());
  cloudOutput->clear();
  while (reader.ReadNextPoint())
  {
    double x = reader.GetPoint().GetX();
    double y = reader.GetPoint().GetY();
    double z = reader.GetPoint().GetZ();
    uint16_t red = reader.GetPoint().GetColor()[0];
    uint16_t green = reader.GetPoint().GetColor()[1];
    uint16_t blue = reader.GetPoint().GetColor()[2];

    /*****颜色说明
        *   uint16_t  范围为0-256*256 ；
        *   pcl 中 R  G  B利用的unsigned char  0-256；
        *   因此这里将uint16_t 除以256  得到  三位数的0-256的值
        *   从而进行rgba 值32位 的位运算。
        *
        ******/

    pcl::PointXYZRGBA thePt; //int rgba = 255<<24 | ((int)r) << 16 | ((int)g) << 8 | ((int)b);
    thePt.x = x;
    thePt.y = y;
    thePt.z = z;
    thePt.rgba = (uint32_t)255 << 24 | (uint32_t)(red / 256) << 16 | (uint32_t)(green / 256) << 8 | (uint32_t)(blue / 256);
    //uint32_t rgb = *reinterpret_cast<int*>(&thePt.rgb);  //reinterpret_cast 强制转换
    cloudOutput->push_back(thePt);
  }

  //boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  //viewer->setBackgroundColor(0, 0, 0); //设置背景
  //viewer->addPointCloud(cloudOutput,"cloudssd");
  //while (!viewer->wasStopped()){
  //    viewer->spinOnce();
  //}
  pcl::io::savePCDFileASCII(strOutPutPointCloudName, *cloudOutput);
  cloudOutput->clear();
}

//读入点云，写.las

void writeLasFromPointCloud3(const std::string strInputPointCloudName, std::string strOutLasName)
{
  std::ofstream ofs(strOutLasName.c_str(), ios::out | ios::binary);
  if (!ofs.is_open())
  {
    std::cout << "err  to  open  file  las....." << std::endl;
    return;
  }
  liblas::Header header;
  header.SetVersionMajor(1);
  header.SetVersionMinor(2);
  header.SetDataFormatId(liblas::ePointFormat3);
  header.SetScale(0.001, 0.001, 0.001);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(strInputPointCloudName, *cloud);
  std::cout <<strInputPointCloudName<< "总数:" << cloud->points.size() << std::endl;
  //写liblas,
  liblas::Writer writer(ofs, header);
  liblas::Point point(&header);

  for (size_t i = 0; i < cloud->points.size(); i++)
  {
    double x = cloud->points[i].x;
    double y = cloud->points[i].y;
    double z = cloud->points[i].z;
    point.SetCoordinates(x, y, z);

    //        uint32_t red = (uint32_t)cloud->points[i].r;
    //        uint32_t green = (uint32_t)cloud->points[i].g;
    //        uint32_t blue = (uint32_t)cloud->points[i].b;
    //        liblas::Color pointColor(red, green, blue);
    //        point.SetColor(pointColor);
    writer.WritePoint(point);
    //std::cout << x << "," << y << "," << z << std::endl;
  }
  double minPt[3] = {9999999, 9999999, 9999999};
  double maxPt[3] = {0, 0, 0};
  header.SetPointRecordsCount(cloud->points.size());
  header.SetPointRecordsByReturnCount(0, cloud->points.size());
  header.SetMax(maxPt[0], maxPt[1], maxPt[2]);
  header.SetMin(minPt[0], minPt[1], minPt[2]);
  writer.SetHeader(header);
}

int main(int argc, char **argv)
{
  //char strInputLasName[] = "qq.las";//"Scan_az001.las";
  //char strOutPutPointCloudName[]="qqqq.pcd";
  //writePointCloudFromLas(strInputLasName, strOutPutPointCloudName);

  //std::string strInputPointCloudName = "eewge";
  //std::string strOutLasName = "eewge";
  //writeLasFromPointCloud(strInputPointCloudName.c_str(), strOutLasName.c_str());
  if (argc < 2)
  {
    std::cout << "参数个数不能少于2个！" << std::endl;
    return -1;
  }

  std::string strInputPointCloudName = argv[1];
  std::string strOutLasName = argv[1];
  strOutLasName.replace(strOutLasName.find(".pcd"), 4, ".las");
  writeLasFromPointCloud3(strInputPointCloudName, strOutLasName);

  return 0;
}