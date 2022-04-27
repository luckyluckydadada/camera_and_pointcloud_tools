// HDPcapChecker.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "output_config.h"
#include "hdl_grabber.h"
#include "hdl_viewer_grabber_helper.h"

namespace hd_pcap_checker {
using std::string;
using std::cout;
using std::endl;
using std::vector;
using std::ofstream;

using cloud_icp_reg::CloudIConstPtr;
using cloud_icp_reg::CloudIPtr;
using cloud_icp_reg::CloudI;
using cloud_icp_reg::CloudIItem;
using cloud_icp_reg::HDLViewerGrabberHelper;
using pcl::EnumPtDev;

struct CheckOptions {
    bool check_brokenframe;
    bool check_intensity;
    bool check_timestamp;
    bool check_zeroblock;
};

static CheckOptions check_options;
static boost::mutex file_mutex;
static std::string out_filename;
static long pcap_idx = 0;

float dis(const CloudIItem& p) {
    const auto x = p.x;
    const auto y = p.y;
    const auto z = p.z;
    return std::sqrt(x * x + y * y + z * z);
}

template <class T>
std::tuple<T, T> calc_mean_std(const vector<T>& v) {
    T sum = std::accumulate(v.begin(), v.end(), T(0));
    T mean = sum / v.size();

    std::vector<T> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(),
                   std::bind2nd(std::minus<T>(), mean));
    T sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), T(0));
    T stdev = std::sqrt(sq_sum / v.size());
    return std::make_tuple(mean, stdev);
}

void check_pcap_reg(const CloudIConstPtr& cloud) {
	pcap_idx++;
    if (!check_options.check_intensity) {
        return;
    }
    const auto rows = 32U;
    const auto size = cloud->size();
    const auto cols = size / rows;

    const auto timestamp = cloud->header.stamp;

    //cout << size << endl;
    //cout << cols << endl;

    vector<CloudIPtr> each_laser;

    for (auto i = 0U; i < rows; ++i) {
        each_laser.push_back(boost::make_shared<CloudI>());
    }

	//检测满天星
	long pt_stars=0;
    // 将32个激光头的点云分开
    for (auto j = 0U; j < cols; ++j) {
        for (auto i = 0U; i < rows; ++i) {
            const auto index = i + j * rows;
            const auto& p = cloud->points[index];

            if (!boost::math::isnormal(p.x) || !boost::math::isnormal(p.y) ||
                    !boost::math::isnormal(p.z)) {
                continue;
            }

            if (boost::math::isinf(p.x) || boost::math::isinf(p.y) ||
                    boost::math::isinf(p.z)) {
                continue;
            }

            if (boost::math::isnan(p.x) || boost::math::isnan(p.y) ||
                    boost::math::isnan(p.z)) {
                continue;
            }

            each_laser[i]->push_back(p);
			if (p.x > 0 && p.z < -10)
			{
				pt_stars++;
			}
        }
    }
	if (pt_stars > 5)
	{
		boost::lock_guard<boost::mutex> lock(file_mutex);
		ofstream ofile(out_filename, std::ios::app);
		std::cout << "err... noisepointcloud " << pt_stars <<std::endl;
		ofile << "noisepointcloud " << timestamp << std::endl;
		ofile.close();
	}
	if (pcap_idx % 1000 == 0)
	{
		boost::lock_guard<boost::mutex> lock(file_mutex);
		ofstream ofile(out_filename, std::ios::app);
		ofile << "pcap idx " << pcap_idx << std::endl;
		ofile.close();
	}

    for (auto i = 0U; i < rows; ++i) {
        const auto ptr = each_laser[i];
        const auto laser_size = ptr->size();
        vector<float> dis_vec;
        vector<float> intensity_vec;
        dis_vec.reserve(laser_size);

        for (auto j = 0U; j < laser_size; ++j) {
            const auto& p = ptr->at(j);
            const auto distance = dis(p);

            if (distance < 0.00001) {
                continue;
            }

            dis_vec.push_back(distance);
            intensity_vec.push_back(p.intensity);
        }

        if (dis_vec.empty()) {
            continue;
        }

        float dis_mean = 0;
        float dis_stdev = 0;
        std::tie(dis_mean, dis_stdev) = calc_mean_std(dis_vec);
#if OUTPUT_DEBUG_DISTANCE
        cout << "dis avg " << dis_mean << " stdev " << dis_stdev << endl;
#endif

        // 如果是满天星的故障点云，那么每个点的距离会是一个均匀分布，即为U(0,100)
        // 该均匀分布的期望为(a+b)/2=50
        // 该均匀分布的标准差为sqrt((a-b)^2/12)=50/sqrt(3)
        // 所以这个阈值可以设置为40 sqrt(600)
        if (dis_mean >= 100 && dis_stdev >= 40) {
            cout << "#" << i << "激光头杂点过多(avg:" << dis_mean << ",stdev:" << dis_stdev << ")" <<
                 endl;
            boost::lock_guard<boost::mutex> lock(file_mutex);
            ofstream ofile(out_filename, std::ios::app);
            ofile << "noisepointcloud " << timestamp << std::endl;
            ofile.close();
        }

        float int_mean = 0;
        float int_stdev = 0;
        std::tie(int_mean, int_stdev) = calc_mean_std(intensity_vec);
#if OUTPUT_DEBUG_INTENSITY
        cout << "intensity avg " << int_mean << " stdev " << int_stdev << endl;
#endif

        if (int_mean <= 1 && int_stdev <= 1) {
            cout << "#" << i << "激光头反射率过低(avg:" << int_mean << ",stdev:" << int_stdev << ")" <<
                 endl;
            boost::lock_guard<boost::mutex> lock(file_mutex);
            ofstream ofile(out_filename, std::ios::app);
            ofile << "intensity " << timestamp << std::endl;
            ofile.close();
        } else if (int_mean >= 150 && int_stdev <= 1) {
            cout << "#" << i << "激光头反射率过高(avg:" << int_mean << ",stdev:" << int_stdev << ")" <<
                 endl;
            boost::lock_guard<boost::mutex> lock(file_mutex);
            ofstream ofile(out_filename, std::ios::app);
            ofile << "intensity " << timestamp << std::endl;
            ofile.close();
        }
    }
}

void check_pcap(const string& input_pcap, const string& out_file) {
    out_filename = out_file;
    std::ofstream ofile(out_file, std::ios::trunc);
    ofile.close();
    const boost::shared_ptr<pcl::HDLGrabber> grabber(new pcl::HDLGrabber(string(""),
            input_pcap,
            EnumPtDev::ENUM_PTDEV_HDL_64E,
            out_file,
            check_options.check_brokenframe,
            check_options.check_timestamp,
            check_options.check_zeroblock,
            file_mutex));
    HDLViewerGrabberHelper hdl_helper(*grabber, check_pcap_reg);
    hdl_helper.run();
	boost::lock_guard<boost::mutex> lock(file_mutex);
	ofstream ofile_end(out_filename, std::ios::app);
	ofile_end << "pcap count " << pcap_idx << std::endl;
	ofile_end.close();
}

}

int main(int argc, char* argv[]) {
    namespace po = boost::program_options;

    const std::string output_subfix = "_check_info.txt";

    po::options_description desc("V2.1\nHD Point Cloud Checker By BeeC@Baidu");
    desc.add_options()
        ("help,h", "show this help message")
        ("pcap,p", po::value<std::string>(), "input pcap file path")
        ("out,o", po::value<std::string>(),
        (std::string("output check info file, if this option is not provided, [pcap]") + output_subfix +
        " will be used").c_str())
        ("check-brokenframe,b", po::bool_switch()->default_value(false), "enable brokenframe check")
        ("check-intensity,i", po::bool_switch()->default_value(false), "enable intensity check")
        ("check-timestamp,t", po::bool_switch()->default_value(false), "enable timestamp check")
        ("check-zeroblock,z", po::bool_switch()->default_value(false), "enable zeroblock check");

    po::variables_map vm;

    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (po::validation_error& e) {
        std::cout << "parameter [" << e.get_option_name() << "] error" << std::endl;
        std::cout << desc << std::endl;
        return -1;
    } catch (std::exception& e) {
        std::cout << e.what() << std::endl;
        std::cout << desc << std::endl;
        return -1;
    } catch (...) {
        std::cout << "parameter error" << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    if (vm.count("pcap") != 1) {
        std::cout << "parameter [pcap] error" << std::endl;
        std::cout << desc << std::endl;
        return -1;
    }

    const auto pcap_file(vm["pcap"].as<std::string>());
    std::string out_file;

    if (vm.count("out") != 0) {
        out_file = vm["out"].as<std::string>();
    } else {
        out_file = pcap_file + "_check_info.txt";
    }

    auto& op = hd_pcap_checker::check_options;
    op.check_brokenframe = vm["check-brokenframe"].as<bool>();
    op.check_intensity = vm["check-intensity"].as<bool>();
    op.check_timestamp = vm["check-timestamp"].as<bool>();
    op.check_zeroblock = vm["check-zeroblock"].as<bool>();

    std::cout << "current options:" << std::endl;
    std::cout << "pcap file:\t" << pcap_file << std::endl;
    std::cout << "output file:\t" << out_file << std::endl;
    std::cout << std::boolalpha << "check brokenframe:\t" << op.check_brokenframe << std::endl;
    std::cout << std::boolalpha << "check intensity:\t" << op.check_intensity << std::endl;
    std::cout << std::boolalpha << "check timestamp:\t" << op.check_timestamp << std::endl;
    std::cout << std::boolalpha << "check zeroblock:\t" << op.check_zeroblock << std::endl;
    std::cout << std::endl;
   
    if (!op.check_brokenframe
        && !op.check_intensity
        && !op.check_timestamp
        && !op.check_zeroblock) {
        std::cout << "nothing to check, exit" << std::endl;
        return -2;
    }

    hd_pcap_checker::check_pcap(pcap_file, out_file);
    return 0;
}

