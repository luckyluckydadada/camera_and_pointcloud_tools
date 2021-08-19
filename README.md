# camera_and_pointcloud_tools

## pcl-viewer
### build
```
cd pcl-viewer
mkdir build
cd build
cmake ..
make
```
### run
viewer
```
./viewer
```

## calibrate
### build
```
cd calibrate
mkdir build
cd build
cmake ..
make
```
### run
```
./demo
```

## kitti_fusion_pointcloud_and_image
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

## pcl trans
### build
```
cd pcl-trans
mkdir build
cd build
cmake ..
make
```
### run
kitti2pcd
```
./kittibin2pcd 000000.bin 000000.pcd
python kitti2pcd.py
```
kitti_to_rosbag
```
https://github.com/ethz-asl/kitti_to_rosbag
```
pcdbin2pcdascii_pcdascii2pcdbin Usage:
```
./pcdbin2pcdascii_pcdascii2pcdbin in.pcd out.pcd bin2ascii
./pcdbin2pcdascii_pcdascii2pcdbin in.pcd out.pcd ascii2bin
./pcdbin2pcdascii_pcdascii2pcdbin in.pcd out.pcd compress
```
## kitti_native_evaluation
https://github.com/asharakeh/kitti_native_evaluation

