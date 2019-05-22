#include "io_utils.h"


void IO::read_calibration_mat(std::string filename,
                          Eigen::Matrix4f &calibration_mat)
{
    std::cout << ">> read calibration matrix from " << filename << std::endl;
    std::ifstream file_pt(filename.c_str(), std::ios::in|std::ios::binary);
    float a = 0;
    for(int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            file_pt.read(reinterpret_cast<char *>(&a), sizeof(float));
            calibration_mat(i, j) = a;
        }
    }

#ifdef DEBUG
    std::cout << "--> calibration matrix is: " << std::endl;
    std::cout << calibration_mat << std::endl;
#endif
}

void IO::read_calibration_mat_double(std::string filename,
                                     Eigen::Matrix4d &calibration_mat)
{
    std::cout << ">> read calibration matrix from " << filename << std::endl;
    std::ifstream file_pt(filename.c_str(), std::ios::in|std::ios::binary);
    double a = 0;
    for(int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            file_pt.read(reinterpret_cast<char *>(&a), sizeof(double));
            calibration_mat(i, j) = a;
        }
    }

#ifdef DEBUG
    std::cout << "--> calibration matrix is: " << std::endl;
    std::cout << calibration_mat << std::endl;
#endif
}

void IO::read_cloud(std::string filename, PointCloud<PointXYZ>::Ptr &point_cloud)
{
    std::regex pcd_regex(".*pcd");
    std::regex ply_regex(".*ply");
    if (std::regex_match(filename, pcd_regex)) {
        std::cout << ">> reading pcd file from " << filename << std::endl;
        io::loadPCDFile(filename, *point_cloud);
    } else if (std::regex_match(filename, ply_regex)) {
        std::cout << "reading ply file from " << filename<< std::endl;
        io::loadPLYFile(filename, *point_cloud);
    } else {
        std::cerr << ">> file type must be ply or pcd" << std::endl;
        return;
    }
#ifdef DEBUG
    std::cout << "--> point cloud size: " << point_cloud->size()
              << " width: " << point_cloud->width
              << " height: " << point_cloud->height
              << std::endl;
#endif
}

void IO::read_json_file(std::string json_filename, rapidjson::Document &config_file)
{
    FILE *fp = fopen(json_filename.c_str(), "rb");
    char read_buffer[65536];
    rapidjson::FileReadStream is(fp, read_buffer, sizeof(read_buffer));
    config_file.ParseStream(is);
    fclose(fp);
}

void IO::read_float_array(const std::string &file_name, float* data, int data_length) {
    FILE* file_pt = fopen((char*)file_name.c_str(), "rb");
    for(int i = 0; i < data_length; i++){
        std::fread(reinterpret_cast<char*>(&data[i]), sizeof(float), 1, file_pt);
    }
    fclose(file_pt);
}


//int main(int argc, char** argv)
//{
////    std::string mat_filename = "/data/Projects/random-bin-picking/src/scroll_wheel_picking/resource/calibration_matrix/camera_in_world_4.bin";
////    Eigen::Matrix4f mat;
////    read_calibration_mat(mat_filename, mat);
////    std::cout << mat << std::endl;
////
////    std::string model_filename = "/data/Projects/random-bin-picking/src/scroll_wheel_picking/resource/gray_scroll_matlab_mm_centered_reverse.pcd";
////    PointCloud<PointXYZ>::Ptr model (new PointCloud<PointXYZ>);
////    read_data(model_filename, model);
//
////    std::string config_filename =
////            "/data/Projects/random-bin-picking/src/scroll_wheel_picking/resource/config.json";
////    rapidjson::Document config;
////    read_json_file(config_filename, config);
////    std::cout << config["scene_cloud_path"].GetString() << std::endl;
//
//    std::string dt_filename =
//            "/data/Projects/random-bin-picking/src/scroll_wheel_picking/resource/gray_scroll_matlab_mm_centered_reverse_256_1.25.txt";
//    DtInfo dt_info;
//    float *dt;
//    load_distance_transform_model(dt_filename, dt_info, dt);
//    std::cout << dt_info.dt_size << std::endl;
//    for (int i = 0; i < 10; ++i) {
//        std::cout << dt[i] << std::endl;
//    }
//    free(dt);
//}

