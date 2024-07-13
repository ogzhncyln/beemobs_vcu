#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <smart_can_msgs/rc_unittoOmux.h>
#include <smart_can_msgs/AUTONOMOUS_BrakePedalControl.h>
#include <smart_can_msgs/FB_VehicleSpeed.h>
#include <smart_can_msgs/snd_RCUnit_HandBrakeData.h>
#include <smart_can_msgs/AUTONOMOUS_HB_MotorControl.h>
#include <std_msgs/Int32.h>

ros::Subscriber ack_sub;
ros::Subscriber spd_sub;
ros::Subscriber signal_sub;
ros::Subscriber hand_brake_sub;
ros::Publisher steering_pub;
ros::Publisher throttle_pub;
ros::Publisher brake_pub;
ros::Publisher vehicle_control;
ros::Publisher signal_pub;
ros::Publisher hand_brake_pub;

uint8_t ignition = 0;
uint8_t gear=0;
uint8_t spd=0;
uint8_t spd_kmh=0;
uint8_t signal;

bool brake = false;
bool hand_brake = false;

void signal_callback(const std_msgs::Int32::ConstPtr &msg)
{
    signal = msg->data;
}

void spd_callback(const smart_can_msgs::FB_VehicleSpeed::ConstPtr &msg)
{
    spd = msg->FB_ReelVehicleSpeed_Ms;
    spd_kmh = msg->FB_ReelVehicleSpeed_KMh;
}
 
void ack_callback(const ackermann_msgs::AckermannDrive::ConstPtr &msg)
{
    std_msgs::Float64 steering_msg;
    std_msgs::Float64 throttle_msg;
    smart_can_msgs::rc_unittoOmux control_msg;
    smart_can_msgs::AUTONOMOUS_BrakePedalControl brake_msg;

    ignition = 1;
    hand_brake = abs(msg->speed) < 0.001;

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
        //brake_msg.AUTONOMOUS_BrakeMotor_Voltage = 1;
        //brake_msg.AUTONOMOUS_BrakePedalMotor_ACC = 10000;
        //brake_msg.AUTONOMOUS_BrakePedalMotor_EN = 1;
        //brake_msg.AUTONOMOUS_BrakePedalMotor_PER = 60;
    }
    else
    {
        //brake_msg.AUTONOMOUS_BrakeMotor_Voltage = 1;
        //brake_msg.AUTONOMOUS_BrakePedalMotor_ACC = 10000;
        //brake_msg.AUTONOMOUS_BrakePedalMotor_EN = 1;
        //brake_msg.AUTONOMOUS_BrakePedalMotor_PER = 0;
    }

    steering_msg.data = -msg->steering_angle / 3.14 * 180.0;
    throttle_msg.data = abs(msg->speed * 20.0);
    control_msg.RC_Ignition = ignition;
    control_msg.RC_SelectionGear = gear;
    control_msg.RC_SignalStatus = signal;

    vehicle_control.publish(control_msg);
    steering_pub.publish(steering_msg);
    throttle_pub.publish(throttle_msg);
    brake_pub.publish(brake_msg);
    
}

void hb_callback(const smart_can_msgs::snd_RCUnit_HandBrakeData::ConstPtr &msg)
{
    smart_can_msgs::AUTONOMOUS_HB_MotorControl r;

    if(hand_brake)
    {
        if(msg->RC_HandBrake_PRESS == 1)
        {
            r.AUTONOMOUS_HB_MotEN = 0;
            r.AUTONOMOUS_HB_Motor_PWM = 0;
            r.AUTONOMOUS_HB_MotState = 0;
        }
        else if(msg->RC_HandBrake_FREE == 1)
        {
            r.AUTONOMOUS_HB_MotEN=1;
            r.AUTONOMOUS_HB_Motor_PWM = 200;
            r.AUTONOMOUS_HB_MotState = 0;
        }
    }
    else
    {
        if(msg->RC_HandBrake_PRESS == 1)
        {
            r.AUTONOMOUS_HB_MotEN =1;
            r.AUTONOMOUS_HB_Motor_PWM = 200;
            r.AUTONOMOUS_HB_MotState = 1;
        }
        else if(msg->RC_HandBrake_FREE == 1)
        {
            r.AUTONOMOUS_HB_MotEN=0;
            r.AUTONOMOUS_HB_Motor_PWM = 0;
            r.AUTONOMOUS_HB_MotState = 0;
        }
    }

    hand_brake_pub.publish(r);
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"test_topic");
    ros::NodeHandle nh;

    ack_sub = nh.subscribe<ackermann_msgs::AckermannDrive>("/ackermann_cmd",1,ack_callback);
    spd_sub = nh.subscribe<smart_can_msgs::FB_VehicleSpeed>("/beemobs/FB_VehicleSpeed",1,spd_callback);
    signal_sub = nh.subscribe<std_msgs::Int32>("/signal_status",1,signal_callback);
    hand_brake_sub = nh.subscribe<smart_can_msgs::snd_RCUnit_HandBrakeData>("/beemobs/snd_RCUnit_HandBrakeData",1,hb_callback);
    steering_pub = nh.advertise<std_msgs::Float64>("/beemobs/steering_target_value",1);
    throttle_pub = nh.advertise<std_msgs::Float64>("/beemobs/speed_target_value",1);
    vehicle_control = nh.advertise<smart_can_msgs::rc_unittoOmux>("/beemobs/rc_unittoOmux",1);
    brake_pub = nh.advertise<smart_can_msgs::AUTONOMOUS_BrakePedalControl>("/beemobs/AUTONOMOUS_BrakePedalControl",1);
    hand_brake_pub = nh.advertise<smart_can_msgs::AUTONOMOUS_HB_MotorControl>("/beemobs/AUTONOMOUS_HB_MotorControl",1);

    smart_can_msgs::rc_unittoOmux control_msg;
    control_msg.RC_Ignition = 1;
    vehicle_control.publish(control_msg);

    ros::spin();

    return 0;
}
