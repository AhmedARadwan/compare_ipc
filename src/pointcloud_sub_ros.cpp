#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

int counter = 0;
ros::Duration total_latency = ros::Duration(0);

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    ros::Time receive_time = ros::Time::now();
    ros::Time sent_time = msg->header.stamp;
    ros::Duration latency = receive_time - sent_time;
    // ROS_INFO("Latency: %f ms", latency.toSec()*1000);
    total_latency += latency;
    counter++;
    if (counter == 1000) {
        ROS_INFO("Average latency: %f ms", (total_latency.toSec()*1000)/1000);
        counter = 0;
        total_latency = ros::Duration(0);
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "pointcloud_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/point_cloud", 1, callback);
    ros::spin();

    return 0;
}
