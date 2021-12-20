//#include<iostream>
//las2pcd

#include <iostream>
#include <iterator>
#include <cstdlib>
#include <liblas/liblas.hpp> // sudo apt install liblas-dev liblas-c-dev
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <liblas/reader.hpp>
#include <liblas/factory.hpp> 

#include <liblas/detail/fwd.hpp>
#include <liblas/point.hpp>

#include <string.h>
#include <stdio.h>

using namespace std;

string split(const string &str, const string &pattern); //分割字符串的函数声明

int main(int argc, char **argv)
{
  std::ifstream ifs(argv[1], std::ios::in | std::ios::binary);
  liblas::ReaderFactory f;
  liblas::Reader reader = f.CreateWithStream(ifs);

  unsigned long int nbPoints = reader.GetHeader().GetPointRecordsCount();

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.width = nbPoints;
  cloud.height = 1;
  cloud.is_dense = false;
  cloud.points.resize(cloud.width * cloud.height);

  int i = 0;
  uint16_t r1, g1, b1;
  int r2, g2, b2;
  uint32_t rgb;

  while (reader.ReadNextPoint())
  {

    cloud.points[i].x = (reader.GetPoint().GetX());
    cloud.points[i].y = (reader.GetPoint().GetY());
    cloud.points[i].z = (reader.GetPoint().GetZ());

    r1 = (reader.GetPoint().GetColor().GetRed());
    g1 = (reader.GetPoint().GetColor().GetGreen());
    b1 = (reader.GetPoint().GetColor().GetBlue());
    r2 = ceil(((float)r1 / 65536) * (float)256);
    g2 = ceil(((float)g1 / 65536) * (float)256);
    b2 = ceil(((float)b1 / 65536) * (float)256);
    rgb = ((int)r2) << 16 | ((int)g2) << 8 | ((int)b2);
    cloud.points[i].rgb = *reinterpret_cast<float *>(&rgb);

    i++;
  }

  //分割重组字符串

  string oss = split(argv[1], ".las");

  std::cout << oss << " Generated! " << std::endl;

  pcl::io::savePCDFileASCII(oss, cloud); 
  return (0);
}

string split(const string &str, const string &pattern)
{
  string backString;
  vector<string> res;
  if (str == "")
  {
    std::cout << "\n输入的字符串为空！\n"
              << std::endl;
  }

  //在字符串末尾也加入分隔符，方便截取最后一段
  string strs = str + pattern;
  size_t pos = strs.find(pattern);

  while (pos != strs.npos)
  {
    string temp = strs.substr(0, pos);
    res.push_back(temp);
    //去掉已分割的字符串,在剩下的字符串中进行分割
    strs = strs.substr(pos + 1, strs.size());
    pos = strs.find(pattern);
  }

  //便利vector把每一个保存到string中
  //    for (std::vector<string>::const_iterator iter = res.begin(); iter != res.end();iter++)
  //		{
  //    		cout << *iter << " ";
  //			backString = backString + *iter;
  //		}
  backString = res[0] + ".pcd";
  return backString;
}