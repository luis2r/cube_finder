#include "cube_finder/cube_finder.h"

#include <pcl/filters/voxel_grid.h>

#include <pcl/search/kdtree.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/impl/angles.hpp>

using namespace cube_finder;

CubeFinder::CubeFinder(ros::NodeHandle n) : n_(n)
{
    debug_pc_ = n_.advertise<PointCloud2>("/cube_finder/points", 1);
    debug_table_ = n_.advertise<PointCloud2>("/cube_finder/table", 1);
    debug_not_table_ = n_.advertise<PointCloud2>("/cube_finder/not_table", 1);
    debug_clusters_ = n_.advertise<PointCloud2>("/cube_finder/clusters", 1);
    pc_sub_ = n_.subscribe("/head_camera/depth_registered/points", 1, &CubeFinder::pcCallback, this);
}

CubeFinder::~CubeFinder() {}

inline void
downsample(PCLPointCloud &pc) {
    double resolution = 0.01f;
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(pc.makeShared());
    vg.setLeafSize(resolution, resolution, resolution);
    vg.filter(pc);
}

inline void
segmentWithRANSAC(PCLPointCloud &pc, PCLPointCloud &table, PCLPointCloud &non_table)
{
    // Set the headers
    table.header = pc.header;
    non_table.header = pc.header;
    // Create the coefficients and inliers storage
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create a SACSeg. object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(400); // 400
    seg.setDistanceThreshold(0.01); // 0.1
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(pcl::deg2rad(1.5)); // 0.15
    // Filter until the cloud is too small or I "find" the table plane
    pcl::PointCloud<pcl::PointXYZ> filter_pc(pc);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    bool found_ground = false;
    while (filter_pc.size() > 10 && !found_ground) {
      seg.setInputCloud(filter_pc.makeShared());
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0) {
        ROS_WARN("No plane found in the pointcloud!");
        break;
      }
      // if (std::abs(coefficients->values.at(3)) < 0.07) {
      //   ROS_INFO("True");
      // } else {
      //   ROS_INFO("False");
      // }
      // Get the indices using the extractor
      extract.setInputCloud(filter_pc.makeShared());
      extract.setIndices(inliers);
      extract.setNegative(false);
      pcl::PointCloud<pcl::PointXYZ> cloud_out;
      extract.filter(cloud_out);
      table += cloud_out;
      if(inliers->indices.size() != filter_pc.size()){
        extract.setNegative(true);
        pcl::PointCloud<pcl::PointXYZ> cloud_out2;
        extract.filter(cloud_out2);
        non_table += cloud_out2;
        filter_pc = cloud_out2;
      }
      found_ground = true;
    }
}

inline void
extractClusters(const PCLPointCloud::ConstPtr &pc_in, pcl::PointCloud<pcl::PointXYZRGB> &pc_out)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pc_in);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.01); // 2cm
    ec.setMinClusterSize(200);
    ec.setMaxClusterSize(300);
    ec.setSearchMethod(tree);
    ec.setInputCloud(pc_in);
    ec.extract(cluster_indices);

    int i = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it, ++i)
    {
        uint8_t r = 0, g = 0, b = 0;
        switch(i % 3)
        {
            case 0:
                r = 255;
                break;
            case 1:
                g = 255;
                break;
            case 2:
                b = 255;
                break;
        };
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            pcl::PointXYZRGB rgb_point(r, g, b);
            rgb_point.x = pc_in->points[*pit].x;
            rgb_point.y = pc_in->points[*pit].y;
            rgb_point.z = pc_in->points[*pit].z;
            pc_out.points.push_back(rgb_point);
        }
    }
}

void
CubeFinder::pcCallback(const PointCloud2::ConstPtr& pc_in)
{
    ROS_INFO_STREAM("Got it!\n" << pc_in->header);
    PointCloud2 pc_tf;
    try {
      listener_.waitForTransform("base_link", pc_in->header.frame_id,
                                 pc_in->header.stamp, ros::Duration(0.2));
      pcl_ros::transformPointCloud("base_link", *(pc_in), pc_tf, listener_);
    }
    catch (tf::TransformException& e) {
        ROS_WARN("Failed to transform PointCloud: %s", e.what());
        return;
    }
    PCLPointCloud pc;
    pcl::fromROSMsg(pc_tf, pc);
    downsample(pc);
    PCLPointCloud table, not_table;
    segmentWithRANSAC(pc, table, not_table);
    pcl::PointCloud<pcl::PointXYZRGB> clusters;
    extractClusters(not_table.makeShared(), clusters);
    PointCloud2 pc_out, table_out, not_table_out, clusters_out;
    pcl::toROSMsg(pc, pc_out);
    pcl::toROSMsg(table, table_out);
    pcl::toROSMsg(not_table, not_table_out);
    pcl::toROSMsg(clusters, clusters_out);
    clusters_out.header = pc_out.header;
    debug_pc_.publish(pc_out);
    debug_table_.publish(table_out);
    debug_not_table_.publish(not_table_out);
    debug_clusters_.publish(clusters_out);
}
