#include "BEV_projection.h"


std::string padZeros(int val, int num_digits = 6) 
{
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

imProject::imProject(std::string path): _path(path){
    
    //Get all the labels filenames
    fs::path p2(_path + "labels/");
    fs::directory_iterator end_itr;

    for(fs::directory_iterator _entry(p2); _entry!=end_itr; ++_entry) {
        sequence_label_paths.emplace_back(_entry->path().string());
    }
    
    std::sort(sequence_label_paths.begin(), sequence_label_paths.end());

    initColorMap();
}

Mat project_points(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud){

    Mat points(3, cloud->points.size(), CV_64F);
    Mat imPoints(2, cloud->points.size(), CV_64F);
        
    //Transform the p[oints using matrix T
    for(int i=0; i<cloud->points.size(); i++){

        points.at<double>(0,i) = cloud->points.at(i).x - 30;
        points.at<double>(1,i) = cloud->points.at(i).y - 35;
        points.at<double>(2,i) = cloud->points.at(i).z + 80;
    }

    int fo = 500;

    //Project points to 2D plane using camera matrix K
    for(int i=0; i<cloud->points.size(); i++){
        imPoints.at<int>(0,i) = ceil((fo*points.at<double>(0,i) + fo*points.at<double>(2,i))/points.at<double>(2,i));
        imPoints.at<int>(1,i) = (fo*points.at<double>(1,i) + fo*points.at<double>(2,i))/points.at<double>(2,i);
    }

    return imPoints;
}

void imProject::projection(pcl::PointCloud<pcl::PointXYZI>::Ptr cld, int frame){
    
    std::vector<uint16_t> labels;

    readLabels(sequence_label_paths[frame], labels);

    Mat proPoints(2, cld->points.size(), CV_64F);

    proPoints = project_points(cld);

    Mat Image_sm = Mat(500, 500, CV_8UC3, cv::Scalar::all(0));

    //Semantic Image formation
    for(int i=0; i<cld->points.size(); i++){
        if(color_map[labels[i]][0] == 0 && color_map[labels[i]][0] == 0 && color_map[labels[i]][2] == 0){
            continue;
        } 
        int a = abs(proPoints.at<int>(0,i));
        int b = abs(proPoints.at<int>(1,i));

        if(a >= 500 || b >= 500){continue;}

        Image_sm.at<Vec3b>(a, b)[0] = color_map[labels[i]][0];
        Image_sm.at<Vec3b>(a, b)[1] = color_map[labels[i]][1];
        Image_sm.at<Vec3b>(a, b)[2] = color_map[labels[i]][2];
    }

    std::string curr_node_idx_str = padZeros(frame);
    std::string im_file_sm = _path + "BEV_images/" + curr_node_idx_str + ".png";
    cv::imwrite(im_file_sm, Image_sm);

}

int main(int argc,char** argv){

    if(argc != 2) {
        cout << "Usage: ./BEV_projection /path_to_data";
        return -1;
    }

    std::string path = argv[1];

    imProject projector(path);

    // Read data from KITTI bins

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZI>);

    fs::path p1(path + "velodyne/");
    fs::directory_iterator end_itr;

    std::vector<std::string> sequence_scan_names;
    std::vector<std::string> sequence_scan_paths;

    for(fs::directory_iterator _entry(p1); _entry!=end_itr; ++_entry) {
        sequence_scan_names.emplace_back(_entry->path().filename().string());
        sequence_scan_paths.emplace_back(_entry->path().string());
    }

    std::sort(sequence_scan_names.begin(), sequence_scan_names.end());
    std::sort(sequence_scan_paths.begin(), sequence_scan_paths.end());

    cout << "Initialize data stream" << endl;

    for (int i = 0; i < sequence_scan_paths.size(); ++i) { 

        cout << "Point cloud file: \t " << sequence_scan_paths[i] << endl;
        readBin(sequence_scan_paths[i], cloudPtr);

        projector.projection(cloudPtr, i);  

        cloudPtr->points.clear();
    }

    return 0;
}
  