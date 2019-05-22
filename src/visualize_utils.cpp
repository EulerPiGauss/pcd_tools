//
// Created by senseadmin on 18-10-12.
//

#include "visualize_utilis.h"

void add_trans_text(
        boost::shared_ptr<pcl::visualization::PCLVisualizer> &viewer,
        Eigen::Matrix4f mat)
{
    std::stringstream ss;
    ss << "o: " << mat(0, 3)
       << ", " << mat(1, 3)
       << ", " << mat(2, 3);
    viewer->addText(ss.str(), 5, 40, 10, 1.0, 1.0, 1.0, "o");
    ss.str("");
    ss << "x: " << mat(0, 0)
       << ", " << mat(1, 0)
       << ", " << mat(2, 0);
    viewer->addText(ss.str(), 5, 30, 10, 1.0, 0, 0, "x");
    ss.str("");
    ss << "y: " << mat(0, 1)
       << ", " << mat(1, 1)
       << ", " << mat(2, 1);
    viewer->addText(ss.str(), 5, 20, 10, 0, 1.0, 0, "y");
    ss.str("");
    ss << "z: " << mat(0, 0)
       << ", " << mat(1, 0)
       << ", " << mat(2, 0);
    viewer->addText(ss.str(), 5, 10, 10, 0, 0, 1.0, "z");
}

float put_in_range(float a, float b, float val) {
    if (a < b) {
        if (val < a)
            return a;
        else if (val > b)
            return b;
    } else {
        if (val < b)
            return b;
        else if (val > a)
            return a;
    }
    return val;
}

std::vector<uint8_t> colormap_jet(float value, float min_val, float max_val) {
    const float gray = 8 * put_in_range(0.0, 1.0, (value - min_val) / (max_val - min_val));
    std::vector<uint8_t> pixel(3);
    const float s = (float) 1.0 / (float) 2.0;
    if (gray <= 1) {
        pixel[0] = 0;
        pixel[1] = 0;
        pixel[2] = static_cast<unsigned char>((gray + 1) * s * 255 + 0.5);
    } else if (gray <= 3) {
        pixel[0] = 0;
        pixel[1] = static_cast<unsigned char>((gray - 1) * s * 255 + 0.5);
        pixel[2] = 255;
    } else if (gray <= 5) {
        pixel[0] = static_cast<unsigned char>((gray - 3) * s * 255 + 0.5);
        pixel[1] = 255;
        pixel[2] = static_cast<unsigned char>((5 - gray) * s * 255 + 0.5);
    } else if (gray <= 7) {
        pixel[0] = 255;
        pixel[1] = static_cast<unsigned char>((7 - gray) * s * 255 + 0.5);
        pixel[2] = 0;
    } else {
        pixel[0] = static_cast<unsigned char>((9 - gray) * s * 255 + 0.5);
        pixel[1] = 0;
        pixel[2] = 0;
    }

    return pixel;
}

void Vis::visualize_single_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new
                         pcl::visualization::PCLVisualizer("single_cloud_vis"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud, 0, 255, 0);
    viewer->addPointCloud(cloud, green, "cloud");
    viewer->spin();
}

void Vis::visualize_point_clouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_list) {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new
                pcl::visualization::PCLVisualizer("multiple clouds vis"));
    viewer->setBackgroundColor(0, 0, 0);

    int n_cloud = (int) cloud_list.size();
    float color_max_val = n_cloud + 1;
    float color_min_val = -2;
    for (int cloud_idx = 0; cloud_idx < n_cloud; cloud_idx++) {
        std::vector<uint8_t> rgb = colormap_jet((float) cloud_idx, color_min_val, color_max_val);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> curr_color(cloud_list[cloud_idx], rgb[0],
                                                                                   rgb[1], rgb[2]);
        char cloud_name[10];
        sprintf(cloud_name, "cloud_%d", cloud_idx);
        viewer->addPointCloud(cloud_list[cloud_idx], curr_color, cloud_name);
        viewer->setPointCloudRenderingProperties
                (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloud_name);
    }
    viewer->spin();
}

void Vis::visualize_cloud_with_local_world_frame(PointCloud<PointXYZ>::Ptr &cloud)
{
    Eigen::Vector4f centroid_point;
    compute3DCentroid(*cloud, centroid_point);
    Eigen::Vector4f min, max;
    getMinMax3D(*cloud, min, max);
    Eigen::Vector4f scale_v = max-min;
    double scale = sqrt(scale_v.dot(scale_v));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new
                     pcl::visualization::PCLVisualizer("local world frame"));
    viewer->addPointCloud(cloud, "cloud");
    Eigen::Matrix3f world_frame = Eigen::Matrix3f::Identity();
    Eigen::Affine3f reference_frame;
    reference_frame.matrix().block(0, 0, 3, 3) = world_frame;
    reference_frame.matrix().block(0, 3, 4, 1) = centroid_point;
    viewer->addCoordinateSystem(scale/6.0, reference_frame);
    viewer->spin();
}

void Vis::visualize_cloud_with_reference_frame(PointCloud<PointXYZ>::Ptr &cloud,
                                          Eigen::Affine3f &reference_mat)
{
    Eigen::Vector4f min, max;
    getMinMax3D(*cloud, min, max);
    Eigen::Vector4f scale_v = max-min;
    double scale = sqrt(scale_v.dot(scale_v));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new
             pcl::visualization::PCLVisualizer("local world frame"));
    viewer->addPointCloud(cloud, "cloud");
    viewer->addCoordinateSystem(scale/6.0, reference_mat);
    add_trans_text(viewer, reference_mat.matrix());
    viewer->spin();
}

//void Vis::visualize_cloud_with_normal(PointCloud<PointXYZ>::Ptr &cloud,
//                                 PointCloud<Normal>::Ptr &cloud_normal)
//{
//    assert(cloud->size() == cloud_normal->size());
//    int normal_display_level = (cloud->size()/MAX_NORMAL_SIZE > 1)
//            ?cloud->size()/MAX_NORMAL_SIZE:1;
//    double reso = compute_approximate_resolution(cloud);
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new
//                     pcl::visualization::PCLVisualizer("cloud normal vis"));
//    visualization::PointCloudColorHandlerCustom<PointXYZ>
//            cloud_green (cloud,0,255, 0);
//    viewer->addPointCloud(cloud, cloud_green, "cloud");
//    viewer->addPointCloudNormals<PointXYZ, Normal>(cloud, cloud_normal,
//                                      normal_display_level, 8*reso, "normal");
//    viewer->spin();
//}


//#include "io_utils.h"
//int main(int argc, char** argv)
//{
//    std::string model_filename = "/data/Projects/random-bin-picking/src/scroll_wheel_picking/resource/gray_scroll_matlab_mm_centered_reverse.pcd";
//    PointCloud<PointXYZ>::Ptr model (new PointCloud<PointXYZ>);
//    read_cloud(model_filename, model);
//
//    std::string scene_filename = "/data/Projects/random-bin-picking/src/scroll_wheel_picking/resource/col_1_whole0.pcd";
//    PointCloud<PointXYZ>::Ptr scene (new PointCloud<PointXYZ>);
//    read_cloud(scene_filename, scene);
//
//    std::vector<PointCloud<PointXYZ>::Ptr> cloud_list{model, scene};
//    visualize_point_clouds(cloud_list);
//
//
//    std::cout << "hello" << std::endl;
//}