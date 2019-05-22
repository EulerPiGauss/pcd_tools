#include "tool_utils.h"

double compute_approximate_resolution (pcl::PointCloud<pcl::PointXYZ>::Ptr
                                       &input_cloud)
{
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    cloud->points.resize(1000);
    if (input_cloud->size() > 1000)
    {
        std::copy(input_cloud->points.begin(),
                  input_cloud->points.begin()+1000,
                  cloud->points.begin());
    } else
    {
        std::copy(input_cloud->points.begin(),
                  input_cloud->points.end(),
                  cloud->points.begin());
    }
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud (cloud);

    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
        if (! pcl_isfinite ((*cloud)[i].x))
        {
            continue;
        }
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2)
        {
            res += sqrt (sqr_distances[1]);
            ++n_points;
        }
    }
    if (n_points != 0)
    {
        res /= n_points;
    }
    return res;
}

void crop_box(PointCloud<PointXYZ>::Ptr &input_cloud,
              PointCloud<PointXYZ>::Ptr &output_cloud,
              Eigen::Vector4f &crop_min,
              Eigen::Vector4f &crop_max,
              bool is_negative)
{
    CropBox<PointXYZ> box_cropper;
    box_cropper.setNegative(is_negative);
    box_cropper.setMin(crop_min);
    box_cropper.setMax(crop_max);
    box_cropper.setInputCloud(input_cloud);
    box_cropper.filter(*output_cloud);
#ifdef DEBUG
    std::cout << "--> crop cloud with " << input_cloud->size()
              << " points into cloud with " << output_cloud->size()
              << " points" << std::endl;
#endif
}

void extract_points(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud,
                    pcl::PointIndices::Ptr &indices,
                    bool is_negative)
{
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    extractor.setInputCloud(input_cloud);
    extractor.setIndices(indices);
    extractor.setNegative(is_negative);
    extractor.filter(*output_cloud);
}

void remove_background(pcl::PointCloud<pcl::PointXYZ>::Ptr &scene,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &bg,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr &output,
                       double radius)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(bg);

    std::vector<int> closest_point_index;
    std::vector<int> close_list;
    std::vector<float> square_distance;

    for (int i = 0; i < scene->size(); ++i) {
        pcl::PointXYZ tmp_point = scene->points[i];
        tree.radiusSearch(tmp_point, radius, closest_point_index, square_distance);
        if (!closest_point_index.empty())
        {
            close_list.push_back(i);
        }
    }
    std::cout << "the background point number :  " << close_list.size() << std::endl;
    pcl::PointIndices::Ptr bg_indices (new pcl::PointIndices);
    bg_indices->indices = close_list;

    extract_points(scene, output, bg_indices, true);
}

