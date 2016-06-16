#include <iostream>
#include <limits>
#include <time.h>       /* clock_t, clock, CLOCKS_PER_SEC */

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/gp3.h>
#include <pcl/registration/icp.h>


int main(int argc, char** argv)
{


        if (argc < 2) {
          std::cout << "Not enough arguments!" << "ARGC: " << argc << std::endl;
          return -1;
        }

        clock_t initial_time, timer;
        timer = clock();
        initial_time = clock();

// load point cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr second_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

       	// Read two PCD files from disk.
       	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *cloud) != 0)
       	{
            // pcl::io::savePLYFileASCII("01-first-source.ply", *cloud);
       		return -1;
       	}

        if (argc == 3) {
         	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[2], *second_cloud) != 0)
         	{
              // pcl::io::savePLYFileASCII("01-second-source.ply", *second_cloud);
         		return -1;
         	}
        }


        timer = clock() - timer;
        std::cout << "File loaded in: " << ((float)timer)/CLOCKS_PER_SEC << " seconds" << std::endl;

        int initial_size = cloud->points.size();
        std::cout << "Point cloud size before filtering is: " << initial_size << std::endl;


// // voxel grid downsampling
//         // Create the filtering object
//         pcl::VoxelGrid<pcl::PointXYZRGB> vgd;
//         vgd.setInputCloud (cloud);
//         vgd.setLeafSize (100.0f, 100.0f, 100.0f); // milimeters
//         vgd.filter (*filtered_cloud);
//         pcl::copyPointCloud(*filtered_cloud,*cloud);
//
//         int downsampled_size = cloud->points.size();
//         std::cout << "Point cloud size after downsampling via voxel grid: " << downsampled_size << std::endl;
//
//         timer = clock() - timer;
//         std::cout << "Point cloud downsampled in: " << ((float)timer)/CLOCKS_PER_SEC << " seconds" << std::endl;


// filter out outliers
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud (cloud);
        sor.setMeanK (30); // number of nearlie4st neighbors to consider
        sor.setStddevMulThresh (1.0);
        sor.filter (*filtered_cloud);
        pcl::copyPointCloud(*filtered_cloud,*cloud);

        int filtered_size = cloud->points.size();
        std::cout << "Point cloud size after filtering: " << filtered_size << std::endl;

        timer = clock() - timer;
        std::cout << "Point cloud filtered in: " << ((float)timer)/CLOCKS_PER_SEC << " seconds" << std::endl;


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

          //ne.setViewPoint (0, 0, 0);
          ne.useSensorOriginAsViewPoint();

          // Use all neighbors in a sphere of radius [mm]
          // !!!! IN OUR DATASETS 1000 eq 1[m], PCL uses 1 eq 1[m], hence radius parameter is not in [mm], but in [mm]
          ne.setRadiusSearch (1500);

          // Compute the features
          ne.compute (*cloud_normals);

          int normals_size = cloud_normals->points.size();
          std::cout << "Normals size: " << normals_size << std::endl;

          timer = clock() - timer;
          std::cout << "Normals computed in: " << ((float)timer)/CLOCKS_PER_SEC << " seconds" << std::endl;

          //flipNormalTowardsViewpoint (const PointT &point, float vp_x, float vp_y, float vp_z, Eigen::Vector4f &normal);

          // std::cout << "Point cloud size:" << cloud->points.size() << std::endl;

          // std::cout << "This is list of first 30 normals:" << std::endl;
          // for (size_t i = 0; i < 30; ++i) {
          //   std::cout << "    " << cloud_normals->points[i].normal_x
          //           << " "    << cloud_normals->points[i].normal_y
          //             << " "    << cloud_normals->points[i].normal_z << std::endl;
          // }
          // std::cout << "This is list of first 30 points:" << std::endl;
          // for (size_t i = 0; i < 30; ++i) {
          //   std::cout << "    " << cloud->points[i].x
          //             << " "    << cloud->points[i].y
          //             << " "    << cloud->points[i].z << std::endl;
          // }


// merge point cloud with normals
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        // pcl::copyPointCloud(*cloud_normals, *cloud_with_normals);
        // pcl::copyPointCloud(*cloud, *cloud_with_normals);
        pcl::concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);
        timer = clock() - timer;
        std::cout << "Point cloud and computer normals merged in: " << ((float)timer)/CLOCKS_PER_SEC << " seconds" << std::endl;


// remove NaN Normals From point cloud and outliers
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp_filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        std::vector<int> indices;
        pcl::removeNaNNormalsFromPointCloud(*cloud_with_normals, *temp_filtered_cloud, indices);
        pcl::copyPointCloud(*temp_filtered_cloud,*cloud_with_normals);

        int filtered_normals_size = cloud_with_normals->points.size();
        std::cout << "Filtered normals size: " << filtered_normals_size << std::endl;

        timer = clock() - timer;
        std::cout << "Normals filtered in: " << ((float)timer)/CLOCKS_PER_SEC << " seconds" << std::endl;


// // surface reconstruction
//         // Create search tree*
//         pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
//         tree2->setInputCloud (cloud_with_normals);
//
//         // Initialize objects
//         pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
//         pcl::PolygonMesh triangles;
//
//         // Set the maximum distance between connected points (maximum edge length)
//         gp3.setSearchRadius (250);
//
//         // Set typical values for the parameters
//         gp3.setMu (2.5);
//         gp3.setMaximumNearestNeighbors (100);
//         gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
//         gp3.setMinimumAngle(M_PI/18); // 10 degrees
//         gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
//         gp3.setNormalConsistency(false);
//
//         // Get result
//         gp3.setInputCloud (cloud_with_normals);
//         gp3.setSearchMethod (tree2);
//         gp3.reconstruct (triangles);
//
//         // Additional vertex information
//         // std::vector<int> parts = gp3.getPartIDs();
//         // std::vector<int> states = gp3.getPointStates();
//
// // save reconstructed surface
//         pcl::io::savePLYFile("01-surface.ply", triangles);



  // ICP object.
if (argc == 3) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_icp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> registration;
        registration.setInputSource(cloud);
        registration.setInputTarget(second_cloud);

        registration.align(*final_icp_cloud);
        if (registration.hasConverged())
        {
        	std::cout << "ICP converged." << std::endl
        			  << "The score is " << registration.getFitnessScore() << std::endl;
        	std::cout << "Transformation matrix:" << std::endl;
        	std::cout << registration.getFinalTransformation() << std::endl;
        }
        else std::cout << "ICP did not converge." << std::endl;

        pcl::io::savePLYFileASCII("icp-result.ply", *final_icp_cloud);
        pcl::io::savePLYFileASCII("icp-source.ply", *cloud);
        pcl::io::savePLYFileASCII("icp-target.ply", *second_cloud);
}





// save PLY file for inspection with meshlab
        pcl::io::savePLYFileASCII("normals-result.ply", *cloud_with_normals);

        timer = clock() - timer;
        std::cout << "File saved in: " << ((float)timer)/CLOCKS_PER_SEC << " seconds" << std::endl;

        std::cout << std::endl << "Total time passed with processing: " << ((float)(clock() - initial_time))/CLOCKS_PER_SEC << " seconds" << std::endl << std::endl;

// visualize point cloud and normals
        pcl::visualization::PCLVisualizer viewer("PCL Viewer");
        viewer.setBackgroundColor (0.0, 0.0, 0.0);
        viewer.addPointCloud<pcl::PointXYZRGBNormal>(cloud_with_normals, "cloud");
        viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(cloud_with_normals,1,1000,"normals");
        // viewer.addPointCloud<pcl::PointXYZRGB>(cloud, "cloud");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "normals");
        // viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, cloud_normals, 1, 10, "normals");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.4, 0.0, "normals");
        viewer.addCoordinateSystem(1.0);
        viewer.initCameraParameters();

        timer = clock() - timer;
        std::cout << "Viewer initialized in: " << ((float)timer)/CLOCKS_PER_SEC << " seconds" << std::endl;

        while (!viewer.wasStopped ())
        {
          viewer.spinOnce ();
        }
        return 0;
}
