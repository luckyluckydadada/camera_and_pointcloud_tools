#pragma once
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/grabber.h>

#include "hdl_grabber.h"
#include "cmm_types.h"

namespace cloud_icp_reg {
using namespace pcl;
typedef void(*CloudCallBack)(const CloudIConstPtr& cloud);

class HDLViewerGrabberHelper {
public:

    HDLViewerGrabberHelper(HDLGrabber& grabber, CloudCallBack callback)
            : grabber_(grabber), callback_(callback), isDataReady(false)
    {
    }

    void run();
    void cloud_callback(const CloudIConstPtr& cloud);

    pcl::HDLGrabber& grabber_;
    boost::mutex cloud_mutex_;
    CloudCallBack callback_;

    CloudIConstPtr cloud_;
    bool isDataReady;
};
} // cloud_icp_reg