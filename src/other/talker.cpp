#include <sstream>
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "common/can.h"
#include "common/servo_cmd.h"
#include "common/servo_encoder.h"

uint8_t dat[10]={1,0xff,0xfe,0xff,0xfe,6,7,8,9,10};
int16_t spe,ang;
void chatterCallback(const common::can::ConstPtr& msg)
{
    //ROS_INFO("%d,%d,%d,%d,%d,%d,%d,%d",msg.datas[0],msg.datas[1],msg.datas[2],msg.datas[3],msg.datas[4],msg.datas[5],msg.datas[6],msg.datas[7]);
    spe = (int16_t)(msg->datas[1]&0x00ff);
    ang= (int16_t)((msg->datas[2]<<8&0xff00)|(msg->datas[3]&0x00ff));
    
    ROS_INFO("speed = %d,angle = %d",spe,ang);
}
int main(int argc,char **argv)
{
    common::can msg;
    ros::init(argc,argv,"talker");
    ros::NodeHandle n,m;
    ros::Publisher chatter_pub =n.advertise< common::can >("can_rx",1000);
    ros::Subscriber sbu = n.subscribe("can_tx",1000,chatterCallback);
    ros::Rate looprate(100);
    int count = 0;
    int16_t se_speed =-10;
    int16_t se_angle =-16;
    while(ros::ok)
    {
        //std_msgs::UInt8MultiArray msg;
        std::stringstream ss;
        msg.datas.resize(8);
        for (uint8_t i = 0; i < 8; i++)
        {
            msg.datas[i] = dat[i];
          
        }
        msg.datas[2] =se_speed&0xff;
        msg.datas[1] =(se_speed>>8)&0xff; 
        msg.datas[4] =se_angle&0xff;
        msg.datas[3] =(se_angle>>8)&0xff; 
        msg.datas[6] = 2;
        ROS_INFO("%d,%d,%d,%d,%d,%d,%d,%d",msg.datas[0],msg.datas[1],msg.datas[2],msg.datas[3],msg.datas[4],msg.datas[5],msg.datas[6],msg.datas[7]);
        
        
        chatter_pub.publish(msg);
        ros::spinOnce();
        looprate.sleep();
        
        ++count;
    }

    return 0;
}