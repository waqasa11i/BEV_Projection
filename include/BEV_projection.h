#ifndef IMAGE
#define IMAGE

#include "read.h"

namespace fs = boost::filesystem;

using namespace std;
using namespace cv;
using namespace std::chrono;

class imProject{

private:
    std::string _name, _path;
    std::vector<std::string> sequence_label_paths;

public:
    imProject(std::string path);  
    void projection(pcl::PointCloud<pcl::PointXYZI>::Ptr cld, int frame);
};

#endif
