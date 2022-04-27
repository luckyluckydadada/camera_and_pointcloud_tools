#include "stdafx.h"
#include "hdl_viewer_grabber_helper.h"

namespace cloud_icp_reg {
using std::cout;
using std::endl;
using std::string;
void HDLViewerGrabberHelper::run() {
    const boost::function< void(const CloudIConstPtr&)> cloud_cb = boost::bind(
                &HDLViewerGrabberHelper::cloud_callback, this, _1);
    boost::signals2::connection cloud_connection = grabber_.registerCallback(cloud_cb);

    grabber_.start();

    while (grabber_.isRunning()) {
        CloudIConstPtr cloud;

        if (!isDataReady) {
            boost::this_thread::sleep(boost::posix_time::microseconds(100));
            continue;
        }

        // See if we can get a cloud
        if (cloud_mutex_.try_lock()) {
            cloud_.swap(cloud);

            if (cloud != nullptr && !cloud->empty()) {
                callback_(cloud);
                isDataReady = false;
            } else {
                if (cloud != nullptr) {
                    cout << "error ................" << "stamp " << cloud->header.stamp / 1000
                         << "...cloud size 0 :" << cloud->size() << endl;
                }
            }

            grabber_._is_data_away = true;
            cloud_mutex_.unlock();
        }

        boost::this_thread::sleep(boost::posix_time::microseconds(100));
    }

    grabber_.stop();

    cloud_connection.disconnect();
}


void HDLViewerGrabberHelper::cloud_callback(const CloudIConstPtr& cloud) {
    while (isDataReady) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }

    cloud_mutex_.lock();
    cloud_ = cloud;
    cloud_mutex_.unlock();
    isDataReady = true;
}
} // cloud_icp_reg