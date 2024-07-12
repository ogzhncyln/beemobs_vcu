#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ackermann_msgs/AckermannDrive.h>

ros::Subscriber ack_sub;
ros::Publisher test_pub;
 
void ack_callback(const ackermann_msgs::AckermannDrive::ConstPtr &msg)
{
    std_msgs::Float64 r;
    r.data = msg->speed;
    test_pub.publish(r);
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_topic");
    ros::NodeHandle nh;

    ack_sub = nh.subscribe<ackermann_msgs::AckermannDrive>("/ackermann_cmd",1,ack_callback);
    test_pub = nh.advertise<std_msgs::Float64>("/test_topic",1);

    ros::spin();

    return 0;
}
