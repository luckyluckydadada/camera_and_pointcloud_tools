# 方案1：直接使用官方的
sudo apt install liblas-bin
## run
```
las2txt in.las out.txt
```


# 方案2：las2txt
las数据整数和小数部分都够大时，转txt时，pcl库（float）会丢失小数部分精度。转为txt更为稳妥。
## build
```
sudo apt install liblas-dev liblas-c-dev
cd las2txt
mkdir build
cd build
cmake ..
make
```
## run
```
./las2txt in.las  
```
