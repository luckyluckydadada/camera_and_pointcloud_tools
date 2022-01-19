# OPENCV

## apt 安装
默认版本：3.2.0
```
sudo apt-get install libopencv-dev python-opencv
```

## example
### mouse
鼠标点击图片触发事件，如获取ＢＧＲ信息等
### a


## 源码编译安装
依赖：
```
[编译器] 
sudo apt-get install build-essential
[必须的] 
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
[可选的] 
sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper
编译&安装：
```
```
git clone https://github.com/opencv/opencv.git
cd ~/opencv # 进入源码的路径
mkdir release # 创建编译目录
cd release
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ..
# 第一个变量是指定编译类型的，这里我们选择release版本，第二个变量是指定安装目录的，第三个参数时源代码的路径。
make
sudo make install
```

## 基础概念
### 函数库
两个函数库：core module和highgui module。
前者包括了OpenCV中的基础模块库，比如图片容器Mat；
后者包含了负责输入输出操作的函数，如imread和imwrite。

### 加载图片
首先，我们需要创建一个Mat对象来存储被加载的图片数据，然后就可以使用imread函数来加载图片了。
```
Mat image;
image = imread(argv[1], CV_LOAD_IMAGE_COLOR);
```
第一个参数是所要加载的图片的名字（包括路径），这里我们从命令行参数来获取。
OpenCV支持大多数常见的图片格式，如bmp、png、jpeg等等。

第二个参数指定图片的格式：
```
CV_LOAD_IMAGE_UNCHANGED(<0) 保持原加载图片的格式不变
CV_LOAD_IMAGE_GRAYSCALE(0)  加载原图像的灰度（单通道）图
CV_LOAD_IMAGE_COLOR(>0)     以RGB（三通道）格式加载原图像，默认格式
```

### 显示图像

在加载完图像之后，我们便可以查看图像了。首先需要创建一个窗口以显示图片：
```
namedWindow( "Display window", WINDOW_AUTOSIZE );
```
第一个参数为窗口的名字，第二个参数为图片的处理方式，有以下两种方式：

```
CV_WINDOW_AUTOSIZE：窗口适应原图片大小
CV_WINDOW_NORMAL：图片适应窗口大小
```
在使用第二个参数时，你还需要使用|操作符指定图片是否保持纵横比：
```
CV_WINDOW_KEEPRATIO为保持纵横比
CV_WINDOW_FREERATIO为不保持纵横比
```
最后使用imshow函数更新窗口内容为刚刚加载的图片即可：
```
imshow("Display window", image);
```
当然，仅仅是这样还是不够的，你会发现运行程序之后，图片一闪而过，根本无法长时间显示图片。因此，我们需要通过waitKey函数设置窗口显示的时间（参数为等待时间，单位为ms）。这里我们设置时间为无限长（参数为0），直到用户按下任意键。
```
waitKey(0);
```

### 保存图片
保存图片的函数用法很简单，一般情况下只会用到前两个参数。
```
imwrite("rst_mage.jpg", rst_image);
```
第一个参数为图片的名字，记住一定要包含正确的图片格式（bmp、gif、png等）;
第二个参数为要保存的图片对象;
第三个参数主要针对特定的格式，而且是以编码对(vector)
```
vector<int> compression_params;
compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
compression_params.push_back(9);
imwrite("alpha.png", mat, compression_params);
```
