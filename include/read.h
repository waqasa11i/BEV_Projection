#ifndef READ_DATA
#define READ_DATA

#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <unordered_map>
#include <iostream>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <boost/filesystem.hpp>

#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgcodecs.hpp"

// #include <Eigen/QR>
// #include <Eigen/Eigenvalues>

extern std::unordered_map<int, std::string> label_name;
extern std::unordered_map<int, std::vector<int> > color_map;

void initColorMap();

void readLabels(const std::string &labelPath, std::vector<uint16_t> &labels);

void readBin(const std::string &binPath, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr);


#endif
