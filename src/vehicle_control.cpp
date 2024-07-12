#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <smart_can_msgs/rc_unittoOmux.h>
#include <smart_can_msgs/AUTONOMOUS_BrakePedalControl.h>
#include <smart_can_msgs/FB_VehicleSpeed.h>

ros::Subscriber ack_sub;
ros::Subscriber spd_sub;
ros::Publisher steering_pub;
ros::Publisher throttle_pub;
ros::Publisher brake_pub;
ros::Publisher vehicle_control;

uint8_t ignition = 0;
uint8_t gear=0;
uint8_t spd=0;
bool brake = false;

void spd_callback(const smart_can_msgs::FB_VehicleSpeed::ConstPtr &msg)
{
    spd = msg->FB_ReelVehicleSpeed_Ms;
}
 
void ack_callback(const ackermann_msgs::AckermannDrive::ConstPtr &msg)
{
    std_msgs::Float64 steering_msg;
    std_msgs::Float64 throttle_msg;
    smart_can_msgs::rc_unittoOmux control_msg;
    smart_can_msgs::AUTONOMOUS_BrakePedalControl brake_msg;

    ignition = 1;
    brake = abs(msg->speed) < 0.001;

    std::cout << brake << std::endl;

    if (msg->speed > 0)
    {
        gear = 1;
    }else if (msg->speed < 0)
    {
        gear = 2;
    }else{
        gear = 0;
    }
    
    if (brake)
    {
        brake_msg.AUTONOMOUS_BrakeMotor_Voltage = 1;
        brake_msg.AUTONOMOUS_BrakePedalMotor_ACC = 10000;
        brake_msg.AUTONOMOUS_BrakePedalMotor_EN = 1;
        brake_msg.AUTONOMOUS_BrakePedalMotor_PER = 95;
    }
    else
    {
        brake_msg.AUTONOMOUS_BrakeMotor_Voltage = 1;
        brake_msg.AUTONOMOUS_BrakePedalMotor_ACC = 10000;
        brake_msg.AUTONOMOUS_BrakePedalMotor_EN = 1;
        brake_msg.AUTONOMOUS_BrakePedalMotor_PER = 0;
    }

    steering_msg.data = -msg->steering_angle / 3.14 * 180.0 / 2.0;
    throttle_msg.data = abs(msg->speed * 20.0);
    control_msg.RC_Ignition = ignition;
    control_msg.RC_SelectionGear = gear;

    vehicle_control.publish(control_msg);
    steering_pub.publish(steering_msg);
    throttle_pub.publish(throttle_msg);
    brake_pub.publish(brake_msg);
    
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_topic");
    ros::NodeHandle nh;

    ack_sub = nh.subscribe<ackermann_msgs::AckermannDrive>("/ackermann_cmd",1,ack_callback);
    spd_sub = nh.subscribe<smart_can_msgs::FB_VehicleSpeed>("/beemobs/FB_VehicleSpeed",1,spd_callback);
    steering_pub = nh.advertise<std_msgs::Float64>("/beemobs/steering_target_value",1);
    throttle_pub = nh.advertise<std_msgs::Float64>("/beemobs/speed_target_value",1);
    vehicle_control = nh.advertise<smart_can_msgs::rc_unittoOmux>("/beemobs/rc_unittoOmux",1);
    brake_pub = nh.advertise<smart_can_msgs::AUTONOMOUS_BrakePedalControl>("/beemobs/AUTONOMOUS_BrakePedalControl",1);
    //hand_brake_pub = nh.advertise<smart_can_msgs::AUTONOMOUS_BrakePedalControl>("/beemobs/AUTONOMOUS_BrakePedalControl",1);
    ros::spin();

    return 0;
}
