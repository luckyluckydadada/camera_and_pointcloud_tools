//#include<iostream>
//las2txt

#include <iostream>
#include <iterator>
#include <cstdlib>
#include <liblas/liblas.hpp> // sudo apt install liblas-dev liblas-c-dev
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
  double x, y, z;
  uint16_t r1, g1, b1;
  int r2, g2, b2;
  uint32_t rgb;
  string oss = split(argv[1], ".las");
  ofstream output_filename(oss);

  while (reader.ReadNextPoint())
  {

    x = (reader.GetPoint().GetX());
    y = (reader.GetPoint().GetY());
    z = (reader.GetPoint().GetZ());

    r1 = (reader.GetPoint().GetColor().GetRed());
    g1 = (reader.GetPoint().GetColor().GetGreen());
    b1 = (reader.GetPoint().GetColor().GetBlue());
    r2 = ceil(((float)r1 / 65536) * (float)256);
    g2 = ceil(((float)g1 / 65536) * (float)256);
    b2 = ceil(((float)b1 / 65536) * (float)256);
    rgb = ((int)r2) << 16 | ((int)g2) << 8 | ((int)b2);

    output_filename << to_string(x) << "," << to_string(y) << "," << to_string(z) << "," << to_string(rgb) << endl;
  }

  //分割重组字符串

  std::cout << oss << " Generated! " << std::endl;
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
  backString = res[0] + ".txt";
  return backString;
}