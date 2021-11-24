## kitti_fusion_pointcloud_and_image
给kitti中的点云上色，点云+rgb+calib=彩色点云
### build
```
cd kitti_fusion_pointcloud_and_image
mkdir build
cd build
cmake ..
make
```
### run
```
./fusion yy.pcd zz.png calib.txt
```