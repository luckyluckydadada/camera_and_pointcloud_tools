#include <iostream> //标准输入输出流
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文
#include <eigen3/Eigen/Core>
void pcdBinaryOrPcdASCII(std::string fileIn, std::string fileOut, std::string type)
{
    pcl::PCLPointCloud2 cloud;
    Eigen::Vector4f origin; 
    Eigen::Quaternionf orientation;
 
    //将文件中的数据读到cloud, origin, orientation中
    pcl::io::loadPCDFile(fileIn, cloud, origin, orientation);
 
    std::cerr << "Loaded a point cloud with " << cloud.width * cloud.height <<
        " points (total size is " << cloud.data.size() <<
        ") and the following channels: " << pcl::getFieldsList(cloud) << std::endl;
 
    pcl::PCDWriter w;
    if (type == "bin2ascii")
    {
        std::cerr << "Saving file " << fileOut << " as ascii." << std::endl;
        w.writeASCII(fileOut, cloud, origin, orientation);
    }
    else if (type == "ascii2bin")
    {
        std::cerr << "Saving file " << fileOut << " as binary." << std::endl;
        w.writeBinary(fileOut, cloud, origin, orientation);
    }
    else if (type == "compress")
    {
        std::cerr << "Saving file " << fileOut << " as binary_compressed." << std::endl;
        w.writeBinaryCompressed(fileOut, cloud, origin, orientation);
    } 
    else
    {
        std::cout<<"Usage:"<< std::endl<<
        "./bin2ascii_ascii2bin in.pcd out.pcd bin2ascii"<<std::endl<<
        "./bin2ascii_ascii2bin in.pcd out.pcd ascii2bin"<<std::endl<<
        "./bin2ascii_ascii2bin in.pcd out.pcd compress"<<std::endl;
    }     
 
}
int main(int argc,char *argv[ ]){
    if (argc!=4){
        std::cout<<"Usage:"<< std::endl<<
        "./bin2ascii_ascii2bin in.pcd out.pcd bin2ascii"<<std::endl<<
        "./bin2ascii_ascii2bin in.pcd out.pcd ascii2bin"<<std::endl<<
        "./bin2ascii_ascii2bin in.pcd out.pcd compress"<<std::endl;
    }
	else
		pcdBinaryOrPcdASCII(argv[1],argv[2],argv[3]);
}
