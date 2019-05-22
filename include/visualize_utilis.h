//
// Created by senseadmin on 18-10-12.
//

#ifndef SCROLL_WHEEL_PICKING_VISUALIZE_UTILIS_H
#define SCROLL_WHEEL_PICKING_VISUALIZE_UTILIS_H

#include <sstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/registration_visualizer.h>
#include <pcl/common/common.h>

#define MAX_NORMAL_SIZE 500

using namespace pcl;

class Vis{
public:
    static void visualize_single_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    static void visualize_point_clouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_list);

    static void visualize_cloud_with_local_world_frame(PointCloud<PointXYZ>::Ptr &cloud);

    static void visualize_cloud_with_reference_frame(PointCloud<PointXYZ>::Ptr &cloud, Eigen::Affine3f &reference_mat);

    static void visualize_cloud_with_normal(PointCloud<PointXYZ>::Ptr &cloud, PointCloud<Normal>::Ptr &cloud_normal);

};
#endif //SCROLL_WHEEL_PICKING_VISUALIZE_UTILIS_H
