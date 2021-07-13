# camera_and_pointcloud_tools

## pcl-pcd-viewer
### build
```
cd pcl-pcd-viewer
mkdir build
cd build
cmake ..
make
```
### run
```
./pcldemo
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

## kitti2pcd
### build
```
cd kitti2pcd
g++ bin2pcd.cpp -o bin2pcd
```
### run
```
./bin2pcd 000000.bin 000000.pcd
# or 
python kitti2pcd.py
```

## kitti_native_evaluation
https://github.com/asharakeh/kitti_native_evaluation

## kitti_to_rosbag
https://github.com/ethz-asl/kitti_to_rosbag