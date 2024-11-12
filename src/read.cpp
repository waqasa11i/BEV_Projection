#include "read.h"

using namespace std;

// ##################### label ########################
std::unordered_map<int, std::string> label_name;
std::unordered_map<int, std::vector<int> > color_map;

void initColorMap() {
    label_name[0] = "unlabeled";
    label_name[1] = "outlier";
    label_name[10] = "car";
    label_name[11] = "bicycle";
    label_name[13] = "bus";
    label_name[15] = "motorcycle";
    label_name[16] = "on-rails";
    label_name[18] = "truck";
    label_name[20] = "other-vehicle";
    label_name[30] = "person";
    label_name[31] = "bicyclist";
    label_name[32] = "motorcyclist";
    label_name[40] = "road";
    label_name[44] = "parking";
    label_name[48] = "sidewalk";
    label_name[49] = "other-ground";
    label_name[50] = "building";
    label_name[51] = "fence";
    label_name[52] = "other-structure";
    label_name[60] = "lane-marking";
    label_name[70] = "vegetation";
    label_name[71] = "trunk";
    label_name[72] = "terrain";
    label_name[80] = "pole";
    label_name[81] = "traffic-sign";
    label_name[99] = "other-object";
    label_name[252] = "moving-car";
    label_name[253] = "moving-bicyclist";
    label_name[254] = "moving-person";
    label_name[255] = "moving-motorcyclist";
    label_name[256] = "moving-on-rails";
    label_name[257] = "moving-bus";
    label_name[258] = "moving-truck";
    label_name[259] = "moving-other-vehicle";


    color_map[0] = {0, 0, 0};
    color_map[1] = {0, 0, 255};
    color_map[10] = {245, 150, 100};
    color_map[11] = {245, 230, 100};
    color_map[13] = {250, 80, 100};
    color_map[15] = {150, 60, 30};
    color_map[16] = {255, 0, 0};
    color_map[18] = {180, 30, 80};
    color_map[20] = {255, 0, 0};
    color_map[30] = {30, 30, 255};
    color_map[31] = {200, 40, 255};
    color_map[32] = {90, 30, 150};
    color_map[40] = {255, 0, 255};
    color_map[44] = {255, 150, 255};
    color_map[48] = {75, 0, 75};
    color_map[49] = {75, 0, 175};
    color_map[50] = {0, 200, 255};
    color_map[51] = {50, 120, 255};
    color_map[52] = {0, 150, 255};
    color_map[60] = {170, 255, 150};
    color_map[70] = {0, 175, 0};
    color_map[71] = {0, 60, 135};
    color_map[72] = {80, 240, 150};
    color_map[80] = {150, 240, 255};
    color_map[81] = {0, 0, 255};
    color_map[99] = {255, 255, 50};
    color_map[252] = {245, 150, 100};
    color_map[256] = {255, 0, 0};
    color_map[253] = {200, 40, 255};
    color_map[254] = {30, 30, 255};
    color_map[255] = {90, 30, 150};
    color_map[257] = {250, 80, 100};
    color_map[258] = {180, 30, 80};
    color_map[259] = {255, 0, 0};
}

void readLabels(const std::string &labelPath, std::vector<uint16_t> &labels) {
	  cout << "readLabels:  \t" << labelPath << endl;
    std::fstream input(labelPath.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::cerr << "Could not read file: " << labelPath << '\n';
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);
    for (int i = 0; input.good() && !input.eof(); i++) {
        uint32_t data;
        input.read(reinterpret_cast<char *>(&data), sizeof(data));
        uint16_t label = data & 0xFFFF;
        uint16_t instance = data >> 16;
        labels.push_back(label);
        // cout << "Label: \t " << label << endl;
    }
    input.close();
}


void readBin(const std::string &binPath, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr) {
    std::fstream input(binPath.c_str(), std::ios::in | std::ios::binary);
    if (!input.good()) {
        std::cerr << "Could not read file: " << binPath << '\n';
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    for (int ii = 0; input.good() && !input.eof(); ii++) {
        pcl::PointXYZI point;

        input.read((char *) &point.x, sizeof(float));
        input.read((char *) &point.y, sizeof(float));
        input.read((char *) &point.z, sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));

        cloudPtr->push_back(point);
    }
    input.close();
}

