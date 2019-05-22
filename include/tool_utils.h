#ifndef PCD_TOOLS_TOOL_UTILS_H
#define PCD_TOOLS_TOOL_UTILS_H

#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/pca.h>

using namespace pcl;

double compute_approximate_resolution (pcl::PointCloud<pcl::PointXYZ>::Ptr
                                       &input_cloud);

void crop_box(PointCloud<PointXYZ>::Ptr &input_cloud,
              PointCloud<PointXYZ>::Ptr &output_cloud,
              Eigen::Vector4f &crop_min,
              Eigen::Vector4f &crop_max,
              bool is_negative=false);

void extract_points(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                    pcl::PointIndices::Ptr &indices,
                    bool is_negative = false);

void remove_background(pcl::PointCloud<pcl::PointXYZ>::Ptr &scene,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &bg,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &output,
                       double radius);

#endif //PCD_TOOLS_TOOL_UTILS_H
