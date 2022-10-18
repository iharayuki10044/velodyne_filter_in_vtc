#ifndef __VELODYNE_FILTER_IN_VTC_H__
#define __VELODYNE_FILTER_IN_VTC_H__

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/tf.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

class VelodyneFilterInVtc{
    public:
        VelodyneFilterInVtc();
        void process();

        void velodyne_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);

        double caluclate_distance(double x, double y, double z);



    private:
        double HZ;
        double THRESHOLD;
        bool is_velodyne_subscribed;

        pcl::PointCloud<pcl::PointXYZI>::Ptr velodyne_cloud;

        ros::NodeHandle private_nh;

        ros::Subscriber velodyne_sub;

        ros::Publisher velodyne_filtered_pub;

};

#endif // __VELODYNE_FILTER_IN_VTC_H__
