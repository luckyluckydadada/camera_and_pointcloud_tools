/*
 *  功能描述: 定义了一些点云工程通用的类型
 */
namespace cloud_icp_reg {
#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

typedef pcl::PointXYZRGBA CloudItem;//XYZRGBA格式的点云单元
typedef pcl::PointCloud< CloudItem> Cloud;//由XYZRGBA格式的点云单元构成的点云数据
typedef Cloud::ConstPtr CloudConstPtr;//指向带有颜色信息的点云的常量指针
typedef Cloud::Ptr CloudPtr;//指向带有颜色信息的点云的指针,点云pcl中Ptr为shared_ptr类型

typedef pcl::PointXYZI CloudIItem;//XYZI格式的点云单元,其中I为强度
typedef pcl::PointCloud< CloudIItem> CloudI;//由XYZI格式的点云单元构成的点云数据
typedef CloudI::ConstPtr CloudIConstPtr;//指向带有强度信息的点云的常量指针
typedef CloudI::Ptr CloudIPtr;//指向带有强度信息的点云的指针

//禁用默认的拷贝和复制构造函数
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
            TypeName(const TypeName&); \
            TypeName& operator=(const TypeName&)

//单例模式
#define SINGLETON_CLASS(class_name)\
    private:\
    class_name();\
    public:\
    static class_name & instance()\
        {\
        static class_name ins;\
        return ins;\
        }
} // cloud_icp_reg