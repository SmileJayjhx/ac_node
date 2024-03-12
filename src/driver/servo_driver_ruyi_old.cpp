
#include "servo_driver_floor.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
servo_driver_banyunche::servo_driver_banyunche(ros::NodeHandle &nh,
                                                                                                            ros::NodeHandle &nh_priv,
                                                                                                            int canopen_id,
                                                                                                            std::string postfix,
                                                                                                            std::string can_tx_topic,
                                                                                                            double servo_zero_point,
                                                                                                            double k_red_ratio,
                                                                                                            bool oppsite)
                           
    : canopen_id_(canopen_id),
      postfix_(postfix),
      inited(false),
      servo_zero_point_(servo_zero_point),
      nh_(nh),
      nh_priv_(nh_priv),
      k_red_ratio_(k_red_ratio),
      oppsite_(oppsite)
{
    if(oppsite_)
        servo_zero_point_ = - servo_zero_point_;
    // servo_encoder_pub = nh_.advertise<common::servo_encoder>("/servo_encoder" + postfix_, 1000); //
    can_tx_pub = nh_.advertise<common::can>(can_tx_topic, 1000);

    nh_priv_.getParam("angle_updir_flag", angle_updir_flag);
    nh_priv_.getParam("angle_downdir_flag", angle_downdir_flag);
    last_send_time_ = ros::Time::now().toSec();
    last_recv_time_ = ros::Time::now().toSec();
    servo_cmd_cache.sc_theta = 0;
    servo_cmd_cache.sc_vel = 0;

    ros::WallDuration(1).sleep();
    ros::spinOnce();
    ros::WallDuration(0.2).sleep();
    ros::spinOnce();
}

servo_driver_banyunche::~servo_driver_banyunche()
{

}

void servo_driver_banyunche::set_manual_flag(int manual_flag){
    is_manual_flag = manual_flag;
}

void servo_driver_banyunche::set_zero_point(double servo_zero_point)
{
    servo_zero_point_ = servo_zero_point;
}



int servo_driver_banyunche::can_recv(const common::can::ConstPtr &msg, double &theta)
{

    if (msg->id == 0x280)
    {
        static int heartbeat_ = msg->datas[7];
        static int last_heartbeat = heartbeat_;

        int16_t theta_;
        uint8_t *tmp_p = (uint8_t *)&theta_;
        tmp_p[0] = msg->datas[2];
        tmp_p[1] = msg->datas[1];
        theta_ = -theta_ / 100.0f;
        // std::cout<<"1 theta_ = "<< (int16_t)theta_<<std::endl;
        theta_ = (theta_ > 90) ? 90 : theta_;
        theta_ = (theta_ < -90) ? -90 : theta_;
        // std::cout<<"1 theta_ = "<< (int16_t)theta_<<std::endl;

        double agv_theta = theta_;
        // std::cout<<"1 agv_theta = "<< agv_theta<<std::endl;

        int16_t ang_l;
        ang_l = agv_theta;// (int16_t)((msg->datas[1] << 8 & 0xff00) | (msg->datas[2] & 0x00ff));
        if (!angle_updir_flag)
        {
            ang_l = 0 - ang_l;
        }
        // std::cout << "2 ang_l: " << ang_l << "servo_zero_point_ " << servo_zero_point_<< std::endl;
        
        theta = ang_l - servo_zero_point_;

        // std::cout << "3 theta: " << theta << std::endl;
        static double time1 = ros::Time::now().toSec();
        static double time2 = time1;

        time1 = ros::Time::now().toSec();

        if (heartbeat_ == last_heartbeat)
        {
            if ((time2 - time1) > 1)
            {
                //报错
                ROS_ERROR("CAN丢失\n");
                // return;
            }
        }
        else
            time2 = time1;

        last_heartbeat = heartbeat_;
        heartbeat_++;
    }
    last_recv_time_ = ros::Time::now().toSec();
    // can_send_callback();
}


void servo_driver_banyunche::can_downloadCallback1(const common::can::ConstPtr &msg)
{
    if (msg->id == 0x280)
    {
        static int heartbeat_ = msg->datas[7];
        static int last_heartbeat = heartbeat_;

        int16_t theta_;
        // uint8_t *tmp_p = (uint8_t *)&theta_;
        // tmp_p[0]=(unsigned)msg->datas[2];
        // tmp_p[1]=(unsigned)msg->datas[1];
        int16_t h,l;
        h = msg->datas[1];
        l=  msg->datas[2];
        theta_ = (h << 8) | l;//先将高位左移，在与低位相与
        theta_ = -theta_/100.0f;
    //     std::cout << "msg->datas[2]" << (unsigned)msg->datas[2] << "msg->datas[1]" << (unsigned)msg->datas[1] << std::endl;
    //    std::cout<<"agv_theta = "<<(int)theta_<<std::endl;

        theta_ =(theta_>90)?90:theta_;
        theta_ =(theta_<-90)?-90:theta_;

        agv_theta_ = theta_;
        // std::cout<<"1 agv_theta = "<< agv_theta_<<std::endl;
    

        int16_t ang_l;
        ang_l = agv_theta_;// (int16_t)((msg->datas[1] << 8 & 0xff00) | (msg->datas[2] & 0x00ff));
        if (!angle_updir_flag)
        {
            ang_l = 0 - ang_l;
        }
        // std::cout << "2 ang_l: " << ang_l << "servo_zero_point_ " << servo_zero_point_<< std::endl;
        
        agv_theta_ = ang_l - servo_zero_point_;


        static double time1 = ros::Time::now().toSec();
        static double time2 = time1;

        time1 = ros::Time::now().toSec();

        if (heartbeat_ == last_heartbeat)
        {
            if ((time2 - time1) >1)
            {
                //报错
                ROS_ERROR("CAN丢失\n");
                // return;
            }
        }
        else
            time2 = time1;

        last_heartbeat  = heartbeat_;
        heartbeat_++;

    }
      last_recv_time_ = ros::Time::now().toSec();
}


void servo_driver_banyunche::set_servo_cmd(const double &theta)
{
    last_send_time_ = ros::Time::now().toSec();
    float theta_ = theta;
    // send_pose_cmd(theta_);
}




/***************************************************************
  *  @brief     接收common::servo_cmd，下发舵轮转角
  *  @param         消息
 **************************************************************/
void servo_driver_banyunche::car_downloadCallback(const common::servo_cmd::ConstPtr &msg)
{
    //add 2022/6/20
    // robot_state_judge(msg);
    int car_type = 3;
    if (car_type == 3)
    {
        double theta_cmd = msg->sc_theta + servo_zero_point_;

        // std::cout << "theta_cmd " << theta_cmd << "   msg->sc_theta " 
        // << msg->sc_theta << "   servo_zero_point_: " << servo_zero_point_<< std::endl; 

        theta_cmd = (theta_cmd > 90) ? 90 : theta_cmd;
        theta_cmd = (theta_cmd < -90) ? -90 : theta_cmd;

        can_s1.id = 0x270;
        can_s1.len = 8;
        can_s1.datas.resize(can_s1.len);

        //  can_s1.datas[0] = 1;
        can_s1.datas[0] = is_manual_flag;

        // can_s1.datas[1] = tmp_p[1];
        // can_s1.datas[2] = tmp_p[0];

        can_s1.datas[1] = (int16_t)(-theta_cmd * 100) >> 8 & 0xff;
        can_s1.datas[2] = (int16_t)(-theta_cmd * 100) & 0xff;

        can_s1.datas[3] = 0;
        can_s1.datas[4] = 0;
        can_s1.datas[5] = 0;
        can_s1.datas[6] = 0;
        can_s1.datas[7] = heartbeat1++;
        // can_s1.datas[7] = 0;
        can_tx_pub.publish(can_s1);
    }
}



/***************************************************************
  *  @brief     定时can数据下发
  *  @param        
  *  @note      备注
  *  @Sample usage:     函数的使用方法 
 **************************************************************/
void servo_driver_banyunche::can_send_callback()
{
    // 自动模式不发送心跳
    if(is_manual_flag == 1) return;

    can_s1.id = 0x270;
    can_s1.len = 8;
    can_s1.datas.resize(can_s1.len);

    can_s1.datas[0] = is_manual_flag;
    can_s1.datas[1] = 0;
    can_s1.datas[2] = 0;
    can_s1.datas[3] = 0;
    can_s1.datas[4] = 0;
    can_s1.datas[5] = 0;
    can_s1.datas[6] = 0;
    can_s1.datas[7] = heartbeat1++;
    can_tx_pub.publish(can_s1); 
}








int servo_driver_banyunche::timer_can_send(void)
{
    if (inited)
    {
        double cur_time = ros::Time::now().toSec();
        //  std::cout.precision(16);
        //     std::cout <<"cur_time:"<<cur_time<<"   last_recv_time_:"<< last_recv_time_ <<"  "<<cur_time - last_recv_time_ <<std::endl;

        if (cur_time - last_recv_time_ > 0.2)
        {
            servo_cmd_cache.sc_theta = 0;
            std::cout << postfix_ << " servo encoder too old" << std::endl;
            return -1;
        }
        can_send_callback();
        return 0;
    }
    else
    {
        inited = true;
    }
}
