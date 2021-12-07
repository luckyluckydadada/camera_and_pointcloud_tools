# velodyne_grabber
1 直接从velodyne激光设备读取点云，用vtk可视化。
2 读取pcap文件（录制的velodyne激光点云文件），用vtk可视化。
## linux build
```
cd velodyne_grabber
mkdir build
cd build
cmake ..
make
```
## run
```
默认从velodyne设备（ip192.168.1.201，port2368）读取点云。
./velodyne_grabber  
从指定velodyne设备读取点云。
./velodyne_grabber  
从velodyne录制的pcap文件读取点云。
./velodyne_grabber  
```./
