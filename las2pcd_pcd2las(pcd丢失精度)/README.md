# las2pcd_pcd2las
las转pcd。pcd转las。  las数据整数和小数部分都够大时，丢失小数部分精度。
## build
```
sudo apt install liblas-dev liblas-c-dev
cd las2pcd
mkdir build
cd build
cmake ..
make
```
## run
```
./las2pcd in.las  
```
## run
```
./pcd2las in.pcd 
```