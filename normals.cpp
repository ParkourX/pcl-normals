#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <limits>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

int main ()
{
        // load point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::io::loadPCDFile ("01.pcd", *cloud);

// estimate normals

          // Create the normal estimation class, and pass the input dataset to it
          pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
          ne.setInputCloud (cloud);

          // Create an empty kdtree representation, and pass it to the normal estimation object.
          // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
          pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
          ne.setSearchMethod (tree);

          // Output datasets
          pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

          // ne.setViewPoint (0, 0, 0);
          ne.useSensorOriginAsViewPoint();

          // Use all neighbors in a sphere of radius [mm]
          // !!!! IN OUR DATASETS 1000 eq 1[m], PCL uses 1 eq 1[m], hence radius parameter is not in [mm], but in [mm]
          ne.setRadiusSearch (1000);

          // Compute the features
          ne.compute (*cloud_normals);

//flipNormalTowardsViewpoint (const PointT &point, float vp_x, float vp_y, float vp_z, Eigen::Vector4f &normal);

        std::cout << "Point cloud size:" << cloud->points.size() << std::endl;
        std::cout << "Normals size:" << cloud_normals->points.size() << std::endl;


                std::cout << "This is list of first 30 normals:" << std::endl;
                //

                for (size_t i = 0; i < 30; ++i) {
                  std::cout << "    " << cloud_normals->points[i].normal_x
                          << " "    << cloud_normals->points[i].normal_y
                            << " "    << cloud_normals->points[i].normal_z << std::endl;
               }

                std::cout << "This is list of first 30 points:" << std::endl;
                // cloud->points.size ()

                for (size_t i = 0; i < 30; ++i) {
                  std::cout << "    " << cloud->points[i].x
                            << " "    << cloud->points[i].y
                            << " "    << cloud->points[i].z << std::endl;
                }


        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);


        pcl::copyPointCloud(*cloud_normals, *cloud_with_normals);
        pcl::copyPointCloud(*cloud, *cloud_with_normals);

        std::vector<int> indices;
        pcl::removeNaNNormalsFromPointCloud(*cloud_with_normals, *filtered_cloud, indices);

        pcl::io::savePLYFileASCII("01-result.ply", *filtered_cloud);


        // visualize point cloud and normals
        pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        viewer.setBackgroundColor (0.0, 0.0, 0.0);

        //viewer.addPointCloud<pcl::PointXYZRGBNormal>(filtered_cloud, "cloud");

        viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
        viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, cloud_normals, 1, 100, "normals");
        viewer.addCoordinateSystem(1.0);
        viewer.initCameraParameters();



        while (!viewer.wasStopped ())
        {
          viewer.spinOnce ();
        }
        return 0;
}
