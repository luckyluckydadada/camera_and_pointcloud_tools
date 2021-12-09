#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile(argv[1], *cloud);

    cloud->width = (int)cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    char txtfile[512];
    sprintf(txtfile, "%s.txt", argv[1]);
    std::ofstream fout;
    fout.open(txtfile);

    for (int i = 0; i < cloud->points.size(); ++i)
    {
        fout << to_string(cloud->points[i].x) << " " << to_string(cloud->points[i].y) << " " << to_string(cloud->points[i].z) << " " << to_string(cloud->points[i].intensity) << std::endl;
    }
    fout.close();
    return 0;
}