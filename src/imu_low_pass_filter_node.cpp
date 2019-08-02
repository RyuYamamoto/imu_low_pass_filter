#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

const double alpha = 0.8;
ros::Subscriber _imu_in;
ros::Publisher _imu_out;
static double gravity[3] = {0,0,0};

void imu_low_pass_filter(const sensor_msgs::ImuConstPtr &msg)
{
    sensor_msgs::Imu out_msg;

    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = msg->header.frame_id;

    gravity[0] = alpha * gravity[0] + (1-alpha)*msg->linear_acceleration.x;
    gravity[1] = alpha * gravity[1] + (1-alpha)*msg->linear_acceleration.y;
    gravity[2] = alpha * gravity[2] + (1-alpha)*msg->linear_acceleration.z;

    out_msg.linear_acceleration.x = msg->linear_acceleration.x - gravity[0];
    out_msg.linear_acceleration.y = msg->linear_acceleration.y - gravity[1];
    out_msg.linear_acceleration.z = msg->linear_acceleration.z - gravity[2];

    _imu_out.publish(msg);
}

int main(int argc, char *argv[])
{
    ros::NodeHandle nh;
    ros::init(argc, argv, "imu_low_pass_filter_node");

    _imu_out = nh.advertise<sensor_msgs::Imu>("/imu/low_pass/data_raw", 100);
    _imu_in = nh.subscribe("/imu/imu_raw", 100, imu_low_pass_filter);
    
    return 0;
}