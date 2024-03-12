#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "common/can.h"
#include "common/servo_cmd.h"
#include "common/servo_encoder.h"
float speed ;
float angle;

void chatterCallback(const common::servo_encoder::ConstPtr& msg)
{
// ROS_INFO("%d,%d,%d,%d,%d,%d,%d,%d",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6],msg->data[7]);
    speed =  msg->se_vel;
    angle =  msg->se_theta;
    ROS_INFO("speed = %f,angle = %f",speed,angle);
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"listener");
    ros::NodeHandle n,m;
    ros::Publisher chatter_pub =m.advertise< common::servo_cmd >("servo_cmd",1000);
    ros::Rate looprate(100);
    common::servo_cmd msg;
    ros::Subscriber sbu = n.subscribe("servo_encoder",1000,chatterCallback);
    while(ros::ok)
    {
        
        
        msg.sc_vel = -10.6;
        msg.sc_theta = -96.8;
        ROS_INFO("%f,%f",msg.sc_vel,msg.sc_theta);
        chatter_pub.publish(msg);
        ros::spinOnce();
        looprate.sleep();
        
    }
    return 0;
}