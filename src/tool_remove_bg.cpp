#include "io_utils.h"
#include "tool_utils.h"
#include "visualize_utilis.h"

int main(int argc, char** argv)
{
    rapidjson::Document config;
    IO::read_json_file(argv[1], config);

    std::string calib_mat_filename = config["calib_mat"].GetString();
    Eigen::Matrix4d calib_mat_double;
    IO::read_calibration_mat_double(calib_mat_filename, calib_mat_double);

    if (argc != 4)
    {
        std::cerr << " format: this-tool-name config_file input.pcd input_bg.pcd" << std::endl;
        std::cout << " output file will be auto named ad input_filename_remove_bg.pcd" << std::endl;
        return -1;
    }
    Eigen::Matrix4f calib_mat;

//    rapidjson::Document config;
//    IO::read_json_file(argv[1], config);

    PointCloud<PointXYZ>::Ptr input_cloud (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr input_bg (new PointCloud<PointXYZ>);

    PointCloud<PointXYZ>::Ptr output_cloud (new PointCloud<PointXYZ>);

    IO::read_cloud(argv[2], input_cloud);
    IO::read_cloud(argv[3], input_bg);

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

    double radius = config["remove_bg_radius"].GetDouble();
    remove_background(output_cloud, input_bg, output_cloud, radius);



//    Eigen::Vector4f mean;
//    Eigen::Matrix3f rot;
//    PCA<PointXYZ> pca;
//    pca.setInputCloud(output_cloud);
//    mean = pca.getMean();
//    rot = pca.getEigenVectors();
//
//    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();
//    trans.block(0, 0, 3, 3) = rot;
//    trans.block(0, 3, 4, 1) = mean;
//    Eigen::Affine3f frame;
//    frame.matrix() = trans;
//
//    std::cout << "rot : \n" << rot <<std::endl;
//    std::cout << "mean : \n" << mean <<std::endl;
//
////    Vis::visualize_cloud_with_reference_frame(output_cloud, frame);
//
//    PointCloud<PointXYZ>::Ptr trans_cloud (new PointCloud<PointXYZ>);
//    Eigen::Matrix4f inv_trans = trans.inverse();
//    transformPointCloud(*output_cloud, *trans_cloud, inv_trans);
//
//    Eigen::Affine3f i_frame;
//    i_frame.matrix() = Eigen::Matrix4f::Identity();
////    Vis::visualize_cloud_with_reference_frame(trans_cloud, i_frame);

//    std::string calib_mat_filename = config["calib_mat"].GetString();
//    Eigen::Matrix4f calib_mat;
//    IO::read_calibration_mat(calib_mat_filename, calib_mat);

    PointCloud<PointXYZ>::Ptr output_in_world (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr output_in_world_centered (new PointCloud<PointXYZ>);
    transformPointCloud(*output_cloud, *output_in_world, calib_mat);

    PCA<PointXYZ> pca;
    pca.setInputCloud(output_in_world);
    Eigen::Vector4f mean;
    mean = pca.getMean();
    std::cout << "mean: \n" << mean << std::endl;

    Eigen::Matrix4f center_align_trans = Eigen::Matrix4f::Identity();
    center_align_trans.block(0, 3, 3, 1) = - mean.block(0, 0, 3, 1);
    std::cout << "center align mat: \n" << center_align_trans << std::endl;\

    transformPointCloud(*output_in_world, *output_in_world_centered, center_align_trans);



    Eigen::Affine3f i_frame;
    i_frame.matrix() = Eigen::Matrix4f::Identity();
    Vis::visualize_cloud_with_reference_frame(output_in_world_centered, i_frame);

    std::string output_filename(argv[2]);
    io::savePCDFile(output_filename.substr(0, output_filename.length() - 4)+"_remove_bg_centered.pcd", *output_in_world_centered);

    return 0;


}