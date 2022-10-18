#include "velodyne_filter_in_vtc/velodyne_filter_in_vtc.h"

VelodyneFilterInVtc::VelodyneFilterInVtc()
    : private_nh("~")
{
    private_nh.param("HZ", HZ, 10.0);
    private_nh.param("THRESHOLD", THRESHOLD, 0.5);
    
    velodyne_sub = private_nh.subscribe("/velodyne_points", 1, &VelodyneFilterInVtc::velodyne_callback, this);

    velodyne_filtered_pub = private_nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points_filtered", 1);

    bool is_velodyne_subscribed = false;

}

void VelodyneFilterInVtc::velodyne_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, *velodyne_cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    for(auto &point : velodyne_cloud->points){
        double distance = caluclate_distance(point.x, point.y, point.z);
        if(distance < THRESHOLD){
            filtered_cloud->push_back(point);
        }
    }

    sensor_msgs::PointCloud2 filtered_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header.frame_id = "velodyne";
    filtered_cloud_msg.header.stamp = ros::Time::now();

    velodyne_filtered_pub.publish(filtered_cloud_msg);

    is_velodyne_subscribed = true;
}

double VelodyneFilterInVtc::caluclate_distance(double x, double y, double z)
{
    return sqrt(x*x + y*y + z*z);
}

int main(int argc, char** argv){
    // ros::init(argc, argv, "velodyne_filter_in_vtc");
    // ros::NodeHandle nh;
    // ros::NodeHandle pnh("~");
    // VelodyneFilterInVtc velodyne_filter_in_vtc(nh, pnh);
    // velodyne_filter_in_vtc.process();
    return 0;
}
