#include <iostream> 
#include <pcl/io/pcd_io.h> //PCD I/O Operation
#include <pcl/point_types.h> //PCD I/O Operation

int
    main()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;

    // Load cloud data ....
    cloud.width=5;
    cloud.height=1;
    cloud.is_dense=false;
    cloud.resize (cloud.width*cloud.height);

    for (auto& point: cloud)
    {
        point.x = 1024 * rand () / (RAND_MAX+1.0f);
        point.y = 1024 * rand () / (RAND_MAX+1.0f);
        point.z = 1024 * rand () / (RAND_MAX+1.0f);
    }
    pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);

    std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;

    for (const auto& point: cloud)
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    return(0);
}
