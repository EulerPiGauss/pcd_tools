#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <fstream>
#include <sstream>

int main(int argc, char** argv)
{
    std::ifstream infile(argv[1]);
    float x, y, z;
    int r, g, b;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB pt;
    while(infile >> x >> y >> z >> r >> g >> b)
    {
        pt.x = x;
        pt.y = y;
        pt.z = z;
        pt.r = (uint8_t)r;
        pt.g = (uint8_t)g;
        pt.b = (uint8_t)b;

        cloud->points.push_back(pt);
    }
    cloud->width = 1;
    cloud->height = cloud->points.size();

    std::string output_filename(argv[1]);
    pcl::io::savePCDFileBinary(output_filename.substr(0, output_filename.length()-3)+"pcd", *cloud);


//    for (int i = 0; i < 10; ++i) {
//        infile >> x >> y >> z >> r >> g >> b;
//        std::cout << std::setprecision(9) << x << '\t' << y << '\t' << z << '\t' << r << '\t' << g << '\t' << b << std::endl;
//    }


}