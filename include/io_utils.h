#ifndef SCROLL_WHEEL_PICKING_IO_UTILS_H
#define SCROLL_WHEEL_PICKING_IO_UTILS_H

#include <iostream>
#include <regex>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"

using namespace pcl;

//struct DtInfo{
//    float dt_x_min;
//    float dt_y_min;
//    float dt_z_min;
//    float dt_scale;
//    int dt_size;
//};

class IO{
public:
    static void read_calibration_mat(std::string filename, Eigen::Matrix4f &calibration_mat);
    static void read_calibration_mat_double(std::string filename, Eigen::Matrix4d &calibration_mat);

    static void read_json_file(std::string json_filename, rapidjson::Document &config_file);

    static void read_cloud(std::string filename, PointCloud<PointXYZ>::Ptr &point_cloud);

    static void read_float_array(const std::string &file_name, float* data, int data_length);
};

#endif //SCROLL_WHEEL_PICKING_IO_UTILS_H
