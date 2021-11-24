# kitti2pcd
kitti转pcd。
## build
```
cd pcl-trans
mkdir build
cd build
cmake ..
make
```
## run
kitti2pcd
```
./kittibin2pcd 000000.bin 000000.pcd
python kitti2pcd.py
```