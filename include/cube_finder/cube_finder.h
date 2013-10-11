#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl_ros/transforms.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include "tf/transform_listener.h"

using sensor_msgs::PointCloud;
using sensor_msgs::PointCloud2;

namespace cube_finder {
typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::ConstPtr PCLPointCloudPtr;

class CubeFinder
{
private:
    ros::NodeHandle n_;
    ros::Subscriber pc_sub_;
    ros::Publisher debug_pc_;
    ros::Publisher debug_table_;
    ros::Publisher debug_not_table_;
    ros::Publisher debug_clusters_;

    tf::TransformListener listener_;

public:
    CubeFinder(ros::NodeHandle n);
    ~CubeFinder();

    void pcCallback(const PointCloud2::ConstPtr& pc_in);
};

}
