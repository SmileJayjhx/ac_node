#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int16MultiArray.h"

#include "common/peripheral_uart.h"
#include <dynamic_reconfigure/server.h>
#include <ac_node/ServoConfig.h>
#include <fstream>
#include <string>
#include "ac_driver.h"
// #include "servo_driver_banyunche.h"
#include "curtis_driver.h"
#include "mima_driver.h"
#include "servo_driver.h"
#include "rebot_driver.h"
#include <YJL_driver.h>
#include "common/ERROR.h"
#include "common/omv_servo_cmd.h"
#include "common/omv_servo_encoder.h"
#include "std_msgs/Int8.h"

bool Curtis_Steer_Enable;
int left_canopen_id;
int right_canopen_id;
vector<std::shared_ptr<ac_driver>> ac;
std::string config_file_path;
std::string config_file = "servo_zero_point.yaml";
ros::Publisher err_pub;
double steering_theta = 0;
double k_passive_red_ratio;
common::servo_encoder se;
double near_threshold;
double servo_near_threshold;
ros::Publisher servo_encoder_pub;
int is_manual_mode;
bool fork_need_reset = false;
// int controller_state_id;
// int controller_error_id;
ros::Publisher reset_curtis_pub;
std::string vehicle_brand;
std::string vehicle_type;
void save_zero_point(double save_zero_point)
{
    std::string temp_file = config_file_path + config_file;
    std::string config_temp_file_ = temp_file;
    std::ofstream foutput_temp(config_temp_file_);
    foutput_temp << "# 舵轮角度零点     " << '\n'
                 << '\n';
    foutput_temp << "servo_zero_point:   " << save_zero_point << '\n';
    foutput_temp << std::flush;
    foutput_temp.close();
}
double servo_zero_point;
double left_servo_zero_point;
double right_servo_zero_point;
void reconfig(ac_node::ServoConfig &config, uint32_t level)
{
    static bool save_data = false;
    servo_zero_point = config.servo_zero_point;
    for (int i = 0; i < ac.size(); i++)
    {
        ac[i]->set_zero_point(config.servo_zero_point);
    }

    // sd_banyunche->set_zero_point(config.servo_zero_point);
    std::cout << servo_zero_point << std::endl;
    if (config.save_data && !save_data)
    {
        save_zero_point(servo_zero_point);
    }
    save_data = config.save_data;
}

void initializeDrivers(const ros::NodeHandle& nh, const ros::NodeHandle& nh_priv) {
    std::string vehicle_brand, vehicle_type;
    std::string servo_can_tx_topic, curtis_can_tx_topic;
    int car_rpm_max;
    double servo_zero_point, left_servo_zero_point, right_servo_zero_point;
    double k_passive_red_ratio, near_threshold, servo_near_threshold;
    bool curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable;
    int left_canopen_id, right_canopen_id;
    nh_priv.getParam("main_servo_zero_point", servo_zero_point);
    nh_priv.getParam("left_servo_zero_point", left_servo_zero_point);
    nh_priv.getParam("right_servo_zero_point", right_servo_zero_point);
    nh_priv.getParam("fre", fre);
    nh_priv.getParam("timeout_ms", timeout_ms);
    nh_priv.getParam("servo_can_tx_topic", servo_can_tx_topic);
    nh_priv.getParam("curtis_can_tx_topic", curtis_can_tx_topic);
    nh_priv.getParam("servo_can_rx_topic", servo_can_rx_topic);
    nh_priv.getParam("can_rx_topic", can_rx_topic);
    nh_priv.getParam("Curtis_Steer_Enable", Curtis_Steer_Enable);

    nh_priv.getParam("car_rpm_max", car_rpm_max);
    nh_priv.getParam("k_passive_red_ratio", k_passive_red_ratio);
    nh_priv.getParam("near_threshold", near_threshold);
    nh_priv.getParam("servo_near_threshold", servo_near_threshold);
    nh_priv.getParam("config_file_path", config_file_path);
    nh_priv.getParam("zero_point_calibration_enable", zero_point_calibration_enable);
    nh_priv.getParam("curtis_theta_oppsite", curtis_theta_oppsite);
    nh_priv.getParam("curtis_vel_oppsite", curtis_vel_oppsite);
    nh_priv.getParam("left_canopen_id", left_canopen_id);
    nh_priv.getParam("right_canopen_id", right_canopen_id);
    nh_priv.getParam("vehicle_type", vehicle_type);
    nh_priv.getParam("vehicle_brand", vehicle_brand);
    nh_priv.getParam("cmd_topic", cmd_topic);

    if (vehicle_brand == "xilin") {
        if (vehicle_type == "single") {
            auto cd = std::make_shared<curtis_driver>(nh, curtis_can_tx_topic, car_rpm_max, servo_zero_point, curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable, near_threshold);
            ac.push_back(cd);
            ROS_INFO("Single mode for Xilin activated.");
        } else if (vehicle_type == "omv") {
            auto cd = std::make_shared<curtis_driver>(nh, curtis_can_tx_topic, car_rpm_max, servo_zero_point, curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable, near_threshold);
            auto sd_left = std::make_shared<servo_driver>(nh, left_canopen_id, "left", servo_can_tx_topic, left_servo_zero_point, k_passive_red_ratio, true, servo_near_threshold);
            auto sd_right = std::make_shared<servo_driver>(nh, right_canopen_id, "right", servo_can_tx_topic, right_servo_zero_point, k_passive_red_ratio, true, servo_near_threshold);
            ac.push_back(cd);
            ac.push_back(sd_left);
            ac.push_back(sd_right);
            ROS_INFO("OMV mode for Xilin activated.");
        }
    } else if (vehicle_brand == "rebot") {
        auto rd = std::make_shared<rebot_driver>(nh, curtis_can_tx_topic, car_rpm_max, servo_zero_point, curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable, near_threshold);
        ac.push_back(rd);
        ROS_INFO("Rebot driver activated.");
    } else if (vehicle_brand == "mima") {
        auto md = std::make_shared<mima_driver>(nh, curtis_can_tx_topic, car_rpm_max, servo_zero_point, curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable, near_threshold);
        ac.push_back(md);
        ROS_INFO("Mima driver activated.");
    } else if (vehicle_brand == "YJL") {
        auto yd = std::make_shared<YJL_driver>(nh, curtis_can_tx_topic, car_rpm_max, servo_zero_point, curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable, near_threshold);
        ac.push_back(yd);
        ROS_INFO("YJL driver activated.");
    }

    // 公共的部分：初始化servo_encoder_pub，这部分只需要执行一次
    servo_encoder_pub = nh.advertise<common::servo_encoder>("/servo_encoder", 1000);
}


void handle_curtis_reset()
{
    static double reset_ac_time;
    static bool reset_flag = false;
    bool need_reset=false;
    int need_reset_num=0;
    for (int i = 0; i < ac.size(); i++)
    {
        std::cout<<"ac.size()"<<ac.size()<<std::endl;
        std::cout<<"ac[i]->need_reset()"<<ac[i]->need_reset()<<std::endl;
        if (ac[i]->need_reset()==true)
        {
            need_reset_num++;
             std::cout<<"ac[i]->need_reset()"<<ac[i]->need_reset()<<std::endl;
            
        }
    }
  if(need_reset_num== ac.size())
  {
     need_reset = true;
  }
  else 
  {
    need_reset = false;
  }
     std::cout << "reset_flag" <<reset_flag<<"need_reset"<<need_reset<<"fork_need_reset"<<fork_need_reset<< std::endl;
    if  ((!reset_flag) && (need_reset))
    {
         std::cout << "reset" << std::endl;
    }

     std::cout<<((!reset_flag) && (need_reset))<<std::endl;
     std::cout<< (((!reset_flag) && (need_reset)) || (fork_need_reset))<<std::endl;
    
    if (((!reset_flag) && (need_reset)) || (fork_need_reset)) //&& (cd->need_reset())
    {
        std::cout << "begin of reset curtis" << std::endl;
        reset_ac_time = ros::Time::now().toSec();
        for (int i = 0; i < ac.size(); i++)
        {
            ac[i]->disable_send(); // cd->disable_send();
        }

        std_msgs::Int8 msg;
        reset_curtis_pub.publish(msg);
        reset_flag = true;
        fork_need_reset = false;
        return;
    }
    if (reset_flag)
    {
        if (ros::Time::now().toSec() - reset_ac_time > 7.0)
        {
            std::cout << "end of reset curtis" << std::endl;
            for (int i = 0; i < ac.size(); i++)
            {
                ac[i]->init(); //           cd->init();

                ac[i]->enable_send(); // cd->enable_send();
            }

            reset_flag = false;
        }
    }
}

void fork_reset_ac_Callback(const std_msgs::Int8::ConstPtr &msg)
{
    if (fork_need_reset == false)
        fork_need_reset = true;
}
void can_send_callback(const ros::TimerEvent &)
{
    for (int i = 0; i < ac.size(); i++)
    {
        ac[i]->timer_can_send();
    }
    se.header.stamp = ros::Time::now();;
     servo_encoder_pub.publish(se);
    if (is_manual_mode == 1)
        handle_curtis_reset();
}

void servo_can_recv_callback(const common::can::ConstPtr &msg)
{
    std::string error_msg;
    for (int i = 0; i < ac.size(); i++)
    {
        ac[i]->controller_state_report(msg, se, error_msg);
    }
}

void can_recv_callback(const common::can::ConstPtr &msg)
{
    int res = 0;
    std::string error_msg;
    for (int i = 0; i < ac.size(); i++)
    {
        ac[i]->controller_state_report(msg, se, error_msg);
         ac[i]->controller_error_report(msg, error_msg);
    }

    if (0 != res)
    {
        common::ERROR e;
        e.header.stamp = ros::Time::now();
        e.error_id = res;
        e.error_str.data = error_msg;
        err_pub.publish(e);
    }
}

// bool judge_angle_near(double thetax, double thetay, double threshold)
// {
//     double err = thetax - thetay;
//     while (err < -180)
//         err += 360;
//     while (err > 180)
//         err -= 360;
//     // std::cout << "err:" << thetax << " " << thetay << " " << err << std::endl;
//     if (fabs(err) < threshold)
//         return true;
//     else
//         return false;
// }

void servo_cmd_callback(const common::servo_cmd::ConstPtr &msg)
{

    bool syn_enable = true;

    if (is_manual_mode == 1)
    {
        for (int i = 0; i < ac.size(); i++)
        {
            if (!(ac[i]->judge_angle_near(msg, se)))
            {
                syn_enable = false;
            }
        }
        for (int i = 0; i < ac.size(); i++)
        {
            std::cout<<"enter_servo_cmd_callback"<<std::endl;
            ac[i]->set_servo_cmd(msg,syn_enable);
        }
    }
    else
    {
        
        return;
    }
}

void get_maual_auto(const common::peripheral_uart::ConstPtr &msg)
{
    /*
        接口板数据： 1-手动     0-自动
        AGV协议数据:    1-自动  0-手动
    */
    static int last_mode = 0;
    if (msg->m_iManualModeFlag == 0)
    {
        is_manual_mode = 1;
    }
    else if (msg->m_iManualModeFlag == 1)
    {
        is_manual_mode = 0;
    }
    if ((last_mode == 0) && (is_manual_mode == 1))
    {

        for (int i = 0; i < ac.size(); i++)
        {
            ac[i]->init();
        }
        // cd->init();
    }
    last_mode = is_manual_mode;
    for(int i = 0; i < ac.size(); i++)
    {
        ac[i]->set_manual_mode(is_manual_mode);
    }
}

int main(int argc, char **argv)
{
    // 初始化节点
    ros::init(argc, argv, "ac_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::Timer timer;

    std::string servo_can_tx_topic;
    std::string curtis_can_tx_topic;
    std::string servo_can_rx_topic;
    std::string can_rx_topic;
    std::string cmd_topic;
    int car_rpm_max;

    int fre;
    int timeout_ms;
    bool curtis_theta_oppsite = false;
    bool curtis_vel_oppsite = false;
    bool zero_point_calibration_enable = false;



    err_pub = nh.advertise<common::ERROR>("ERROR", 10);
    if (vehicle_brand == "xilin")
    {
        if (vehicle_type == "single")
        {
            std::shared_ptr<ac_driver> cd;
            cd = std::make_shared<curtis_driver>(nh, curtis_can_tx_topic, car_rpm_max,
                                                 servo_zero_point, curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable, near_threshold);
            ac.push_back(cd);
            ROS_ERROR("single called! servo_encoder_pub created!");
            servo_encoder_pub = nh.advertise<common::servo_encoder>("/servo_encoder", 1000);
        }
        else if (vehicle_type == "omv")
        {
            std::shared_ptr<ac_driver> cd;
            std::shared_ptr<ac_driver> sd_left;
            std::shared_ptr<ac_driver> sd_right;
            cd = std::make_shared<curtis_driver>(nh, curtis_can_tx_topic, car_rpm_max,
                                                 servo_zero_point, curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable, near_threshold);
            sd_left = std::make_shared<servo_driver>(nh, left_canopen_id, "left",
                                                     servo_can_tx_topic, left_servo_zero_point, k_passive_red_ratio, true, servo_near_threshold);
            sd_right = std::make_shared<servo_driver>(nh, right_canopen_id, "right",
                                                      servo_can_tx_topic, right_servo_zero_point, k_passive_red_ratio, true, servo_near_threshold);
            ac.push_back(cd);
            ac.push_back(sd_left);
            ac.push_back(sd_right);
            ROS_ERROR("omv called! servo_encoder_pub created!");
            servo_encoder_pub = nh.advertise<common::servo_encoder>("/omv_servo_encoder", 1000);
        }
    }
    else if (vehicle_brand == "rebot")
    {
        std::shared_ptr<rebot_driver> rd;
        rd = std::make_shared<rebot_driver>(nh, curtis_can_tx_topic, car_rpm_max,
                                            servo_zero_point, curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable, near_threshold);
        ac.push_back(rd);
        ROS_ERROR("rebot called! servo_encoder_pub created!");
        servo_encoder_pub = nh.advertise<common::servo_encoder>("/servo_encoder", 1000);
    }

    else if (vehicle_brand == "mima")
    {
        std::shared_ptr<mima_driver> md;
        md = std::make_shared<mima_driver>(nh, curtis_can_tx_topic, car_rpm_max,
                                           servo_zero_point, curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable, near_threshold);
        ac.push_back(md);
        servo_encoder_pub = nh.advertise<common::servo_encoder>("/servo_encoder", 1000);
    }
    else if (vehicle_brand == "YJL")
    {
        std::shared_ptr<YJL_driver> yd;
        yd = std::make_shared<YJL_driver>(nh, curtis_can_tx_topic, car_rpm_max,
                                           servo_zero_point, curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable, near_threshold);
        ac.push_back(yd);
        servo_encoder_pub = nh.advertise<common::servo_encoder>("/servo_encoder", 1000);
    }
    
    ros::Subscriber sub_omv_servo_cmd_ = nh.subscribe(cmd_topic, 8, servo_cmd_callback);
    // servo_encoder_pub = nh.advertise<common::servo_encoder>("/servo_encoder", 10); //
    reset_curtis_pub = nh.advertise<std_msgs::Int8>("/reset_curtis_cmd", 10); //

    ros::Subscriber fork_reset_ac_sub = nh.subscribe("/fork_reset_curtis_cmd", 10, fork_reset_ac_Callback);
    
    dynamic_reconfigure::Server<ac_node::ServoConfig> srv; 
    if (zero_point_calibration_enable)
    {
        srv.setCallback(boost::bind(&reconfig, _1, _2));
    }
    ros::Subscriber curtis_can_rx_sub = nh.subscribe(can_rx_topic, 1000, can_recv_callback);
    ros::Subscriber servo_can_rx_sub = nh.subscribe(servo_can_rx_topic, 1000, servo_can_recv_callback);
    ros::Subscriber maunal_mode_sub = nh.subscribe("/peripheral_devs_state", 100, get_maual_auto);

    double period;
    if (fre > 2)
        period = 1.0f / fre;
    else
        period = 0.05f;
    se.se_vel = 0;
    se.se_theta = 0;
    timer = nh.createTimer(ros::Duration(0.05), can_send_callback);
    ros::spin();
    return 0;
}
