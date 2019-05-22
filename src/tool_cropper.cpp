#include "tool_utils.h"
#include "io_utils.h"

using namespace pcl;
int main(int argc, char** argv)
{
    if (argc != 4)
    {
        std::cerr << " format: this-tool-name config_file input.pcd output.pcd" << std::endl;
        return -1;
    }
    rapidjson::Document config;
    IO::read_json_file(argv[1], config);
    PointCloud<PointXYZ>::Ptr input_cloud (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr output_cloud (new PointCloud<PointXYZ>);
    IO::read_cloud(argv[2], input_cloud);

//    Eigen::Matrix4d inv_calibration_matrix_double;
//    IO::read_calibration_mat_double(config["calib_mat"].GetString(), inv_calibration_matrix_double);
//    Eigen::Matrix4d calibration_matrix_double = inv_calibration_matrix_double.inverse();
//    Eigen::Matrix4f calibration_matrix = calibration_matrix_double.cast<float>();
//    pcl::transformPointCloud(*input_cloud, *output_cloud, calibration_matrix);
//    io::savePCDFileBinary(argv[3], *output_cloud);

    float crop_x_min = (float) config["crop_x_min"].GetDouble();
    float crop_y_min = (float) config["crop_y_min"].GetDouble();
    float crop_z_min = (float) config["crop_z_min"].GetDouble();
    float crop_x_max = (float) config["crop_x_max"].GetDouble();
    float crop_y_max = (float) config["crop_y_max"].GetDouble();
    float crop_z_max = (float) config["crop_z_max"].GetDouble();

    Eigen::Vector4f crop_min, crop_max;
    crop_min << crop_x_min, crop_y_min, crop_z_min, 1.0;
    crop_max << crop_x_max, crop_y_max, crop_z_max, 1.0;
    crop_box(input_cloud, output_cloud, crop_min, crop_max);

    io::savePCDFileBinary(argv[3], *output_cloud);

    return 0;

}
