#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int16MultiArray.h"

#include "common/peripheral_uart.h"
#include <dynamic_reconfigure/server.h>
#include <ac_node/ServoConfig.h>
#include <fstream>
#include <string>

// #include "servo_driver_banyunche.h"
#include "rebot_driver.h"

#include "common/ERROR.h"
#include "common/omv_servo_cmd.h"
#include "common/omv_servo_encoder.h"
#include "std_msgs/Int8.h"
/*
*   主程序
* 接收servo_cmd转给 车体控制类，车体控制类解析成can消息直接发送
* 接收判断手动/自动
* 接收can消息传给 车体控制类，车体控制类直接发布servo_encoder
*/
bool Curtis_Steer_Enable;
// std::shared_ptr<servo_driver_banyunche> sd_banyunche;
std::shared_ptr<rebot_driver> rd;
std::string config_file_path;
std::string config_file = "servo_zero_point.yaml";
ros::Publisher err_pub;
double steering_theta = 0;
double k_passive_red_ratio;
common::servo_encoder se;
double near_threshold;
ros::Publisher servo_encoder_pub;
int is_manual_mode;
bool fork_need_reset = false;

ros::Publisher reset_curtis_pub;

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

void reconfig(ac_node::ServoConfig &config, uint32_t level)
{
    static bool save_data = false;
    servo_zero_point = config.servo_zero_point;
    rd->set_zero_point(config.servo_zero_point);
    std::cout << servo_zero_point << std::endl;
    if (config.save_data && !save_data)
    {
        save_zero_point(servo_zero_point);
    }
    save_data = config.save_data;
}

void handle_curtis_reset()
{
    static double reset_ac_time;
    static bool reset_flag = false;
    if (((!reset_flag) && (rd->need_reset()))|| (fork_need_reset))
    {
        std::cout << "begin of reset curtis" << std::endl;
        reset_ac_time = ros::Time::now().toSec();
        rd->disable_send();
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
            rd->init();
            rd->enable_send();
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
    rd->timer_can_send();
}

void servo_can_recv_callback(const common::can::ConstPtr &msg)
{
    // sd_banyunche->can_recv(msg, steering_theta);
    // sd_banyunche->can_downloadCallback1(msg);
    // steering_theta = sd_banyunche->agv_theta_;
    // std::cout << "steering_theta   "<< steering_theta << std::endl;
}

void rebot_can_recv_callback(const common::can::ConstPtr &msg)
{
    int res = 0;
    std::string error_msg;
    if (msg->id == 0x207)
    {
        res = rd->recv_207(msg, se, error_msg);
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


bool judge_angle_near(double thetax, double thetay, double threshold)
{
    double err = thetax - thetay;
    while (err < -180)
        err += 360;
    while (err > 180)
        err -= 360;
   // std::cout << "err:" << thetax << " " << thetay << " " << err << std::endl;
    if (fabs(err) < threshold)
        return true;
    else
        return false;
}

void servo_cmd_callback(const common::servo_cmd::ConstPtr &msg)
{
    std::cout << "servo_cmd_callback msg->sc_theta,msg->se_vel " << msg->sc_theta
        <<" "<< msg->sc_vel <<std::endl;
    if (is_manual_mode == 1)
    {
        rd->set_manual_mode(1);
        std::cout << "msg->sc_theta,se.se_theta,near_threshold: " << msg->sc_theta
         <<" "<< se.se_theta <<" "<< near_threshold <<std::endl;
        bool syn_enable = judge_angle_near(msg->sc_theta, se.se_theta , near_threshold);
        rd->set_servo_cmd(msg->sc_theta, msg->sc_vel, syn_enable);
    }
    else
    {
        rd->set_manual_mode(0);
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
    if (msg->m_iManualModeFlag == 0){
        is_manual_mode = 1;
    }else if (msg->m_iManualModeFlag == 1){
        is_manual_mode = 0;
    }
    if ((last_mode == 0) && (is_manual_mode == 1))
    {
        rd->init();
    }
    last_mode = is_manual_mode;
}

int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc, argv, "ac_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::Timer timer;

    std::string rebot_can_tx_topic;
    std::string rebot_can_rx_topic;

    int car_rpm_max;

    int fre;
    int timeout_ms;
    bool rebot_theta_oppsite = false;
    bool rebot_vel_oppsite = false;
    bool zero_point_calibration_enable = false;


    nh_priv.getParam("main_servo_zero_point", servo_zero_point);

    nh_priv.getParam("fre", fre);
    nh_priv.getParam("timeout_ms", timeout_ms);
    nh_priv.getParam("rebot_can_tx_topic", rebot_can_tx_topic);

    nh_priv.getParam("rebot_can_rx_topic", rebot_can_rx_topic);

    nh_priv.getParam("car_rpm_max", car_rpm_max);
    nh_priv.getParam("k_passive_red_ratio", k_passive_red_ratio);
    nh_priv.getParam("near_threshold", near_threshold);
    nh_priv.getParam("config_file_path", config_file_path);
    nh_priv.getParam("zero_point_calibration_enable", zero_point_calibration_enable);
    nh_priv.getParam("rebot_theta_oppsite", rebot_theta_oppsite);
    nh_priv.getParam("rebot_vel_oppsite", rebot_vel_oppsite);

    err_pub = nh.advertise<common::ERROR>("ERROR", 10);

    rd = std::make_shared<rebot_driver>(nh, rebot_can_tx_topic, car_rpm_max,
                                         servo_zero_point, rebot_theta_oppsite, rebot_vel_oppsite,Curtis_Steer_Enable);

    ros::Subscriber sub_omv_servo_cmd_ = nh.subscribe("/servo_cmd", 8, servo_cmd_callback);
    servo_encoder_pub = nh.advertise<common::servo_encoder>("/servo_encoder", 10); //
    reset_curtis_pub = nh.advertise<std_msgs::Int8>("/reset_curtis_cmd", 10);                  //

    ros::Subscriber fork_reset_ac_sub = nh.subscribe("/fork_reset_curtis_cmd", 10, fork_reset_ac_Callback);
    dynamic_reconfigure::Server<ac_node::ServoConfig> srv;
    if (zero_point_calibration_enable)
    {
        srv.setCallback(boost::bind(&reconfig, _1, _2));
    }
    // ros::Subscriber servo_can_rx_sub = nh.subscribe(servo_can_rx_topic, 1000, servo_can_recv_callback);
    ros::Subscriber curtis_can_rx_sub = nh.subscribe(rebot_can_rx_topic, 1000, rebot_can_recv_callback);

    ros::Subscriber maunal_mode_sub = nh.subscribe("/peripheral_devs_state", 100, get_maual_auto);

    double period;
    if (fre > 2)
        period = 1.0f / fre;
    else
        period = 0.05f;
    se.se_vel = 0;
    se.se_theta = 0;
    timer = nh.createTimer(ros::Duration(period), can_send_callback);
    ros::spin();
    return 0;
}