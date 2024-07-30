#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Header.h>
#include <vector>
#include <random>

std::vector<float> points;
sensor_msgs::PointCloud2 cloud;

void initializePointCloud()
{
    cloud.header.frame_id = "base_link";
    cloud.height = 1;
    cloud.width = 1000000;
    cloud.is_dense = false;
    cloud.is_bigendian = false;
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step * cloud.height);

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");

    std::mt19937 gen;
    std::uniform_real_distribution<float> dist(-10.0, 10.0);

    for (size_t i = 0; i < cloud.width; ++i) {
        points.push_back(dist(gen));
        points.push_back(dist(gen));
        points.push_back(dist(gen));
    }

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (size_t i = 0; i < cloud.width; ++i, ++iter_x, ++iter_y, ++iter_z) {
        *iter_x = points[3 * i];
        *iter_y = points[3 * i + 1];
        *iter_z = points[3 * i + 2];
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud", 1);
    ros::Rate rate(10);

    initializePointCloud();

    while (ros::ok()) {
        cloud.header.stamp = ros::Time::now();
        ROS_INFO("Publishing PointCloud2 message");
        pub.publish(cloud);
        rate.sleep();
    }

    return 0;
}
