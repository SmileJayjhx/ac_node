#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int16MultiArray.h"

#include "common/peripheral_uart.h"
#include <dynamic_reconfigure/server.h>
#include <ac_node/ServoConfig.h>
#include <fstream>
#include <string>

#include "servo_driver.h"
#include "curtis_driver.h"

#include "common/ERROR.h"
#include "common/omv_servo_cmd.h"
#include "common/omv_servo_encoder.h"
#include "std_msgs/Int8.h"

int left_canopen_id;
int right_canopen_id;
bool Curtis_Steer_Enable;
std::shared_ptr<servo_driver> sd_left;
std::shared_ptr<servo_driver> sd_right;
std::shared_ptr<curtis_driver> cd;
std::string config_file_path;
std::string config_file = "servo_zero_point.yaml";
ros::Publisher err_pub;
double left_theta;
double right_theta;
double k_passive_red_ratio;
common::servo_encoder se;
double near_threshold;
ros::Publisher omv_servo_encoder_pub;
bool is_manual_mode;

ros::Publisher reset_curtis_pub;

void save_zero_point(double main_servo_zero_point, double left_servo_zero_point, double right_servo_zero_point)
{
    std::string temp_file = config_file_path + config_file;
    std::string config_temp_file_ = temp_file;
    std::ofstream foutput_temp(config_temp_file_);
    foutput_temp << "# 舵轮角度零点     " << '\n'
                 << '\n';
    foutput_temp << "servo_zero_point:   " << main_servo_zero_point << '\n';
    foutput_temp << "left_servo_zero_point:   " << left_servo_zero_point << '\n';
    foutput_temp << "right_servo_zero_point:   " << right_servo_zero_point << '\n';
    foutput_temp << std::flush;
    foutput_temp.close();
}

void reconfig(ac_node::ServoConfig &config, uint32_t level)
{
    static bool save_data = false;

    sd_left->set_zero_point(config.left_servo_zero_point);
    sd_right->set_zero_point(config.right_servo_zero_point);
    cd->set_zero_point(config.servo_zero_point);

    std::cout << "config.left_servo_zero_point :" << config.left_servo_zero_point << std::endl;
    std::cout << "config.right_servo_zero_point :" << config.right_servo_zero_point << std::endl;
    std::cout << "config.servo_zero_point :" << config.servo_zero_point << std::endl;
    if (config.save_data && !save_data)
    {
        save_zero_point(config.servo_zero_point,
                        config.left_servo_zero_point,
                        config.right_servo_zero_point);
    }
    save_data = config.save_data;
}

void handle_curtis_reset()
{
    static double reset_ac_time;
    static bool reset_flag = false;
    if ((!reset_flag) && (cd->need_reset()))
    {
        std::cout << "begin of reset curtis" << std::endl;
        reset_ac_time = ros::Time::now().toSec();
        cd->disable_send();
        std_msgs::Int8 msg;
        reset_curtis_pub.publish(msg);
        reset_flag = true;
        return;
    }
    if (reset_flag)
    {
        if (ros::Time::now().toSec() - reset_ac_time > 7.0)
        {
            std::cout << "end of reset curtis" << std::endl;
            cd->init();
            cd->enable_send();
            reset_flag = false;
        }
    }
}

void can_send_callback(const ros::TimerEvent &)
{
    sd_left->timer_can_send();
    sd_right->timer_can_send();
    cd->timer_can_send();

    common::omv_servo_encoder msg;
    msg.header.stamp = ros::Time::now();
    msg.se_vel = se.se_vel;
    msg.se_main_theta = se.se_theta;
    msg.se_left_theta = left_theta;
    msg.se_right_theta = right_theta;

    omv_servo_encoder_pub.publish(msg);

    if (!is_manual_mode)
        handle_curtis_reset();
}

void servo_can_recv_callback(const common::can::ConstPtr &msg)
{
    if ((msg->id & 0x7f) == left_canopen_id)
    {
        sd_left->can_recv(msg, left_theta);
    }
    else if ((msg->id & 0x7f) == right_canopen_id)
    {
        sd_right->can_recv(msg, right_theta);
    }
}

void curtis_can_recv_callback(const common::can::ConstPtr &msg)
{
    int res = 0;
    std::string error_msg;
    if (msg->id == 0x4f6)
    {
        res = cd->recv_4f6(msg, se, error_msg);
    }
    else if (msg->id == 0x4f5)
    {
        res = cd->recv_4f5(msg, error_msg);
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
    //  std::cout << "err:" << thetax << " " << thetay << " " << err << std::endl;
    if (fabs(err) < threshold)
        return true;
    else
        return false;
}

int manual_mode = 1;
void omv_servo_cmd_callback(const common::omv_servo_cmd::ConstPtr &msg)
{
    bool res_l = judge_angle_near(msg->sc_left_theta, left_theta, near_threshold);
    bool res_r = judge_angle_near(msg->sc_right_theta, right_theta, near_threshold);
    bool res_s = judge_angle_near(msg->sc_main_theta, se.se_theta, near_threshold);
    if (manual_mode == 0)
    {
        bool syn_enable = res_l && res_r && res_s;
        cd->set_servo_cmd(msg->sc_main_theta, msg->sc_vel, syn_enable);
        // if (res_l && res_r && res_s) //
        //     cd->set_servo_cmd(msg->sc_main_theta, msg->sc_vel);
        // else
        //     cd->set_servo_cmd(msg->sc_main_theta, 0);
        sd_left->set_servo_cmd(msg->sc_left_theta);
        sd_right->set_servo_cmd(msg->sc_right_theta);
    }
    else
    {
        sd_left->set_servo_cmd(0);
        sd_right->set_servo_cmd(0);
    }
}

void get_maual_auto(const common::peripheral_uart::ConstPtr &msg)
{
    /*
        接口板数据： 1-手动     0-自动
    */
    static int last_mode = 1;
    manual_mode = msg->m_iManualModeFlag;
    if ((last_mode == 1) && (manual_mode == 0))
    {
        cd->init();
    }
    last_mode = manual_mode;
    is_manual_mode = (manual_mode == 1);
}

int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc, argv, "ac_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::Timer timer;

    std::string servo_can_tx_topic;
    std::string curtis_can_tx_topic;
    std::string servo_can_rx_topic;
    std::string curtis_can_rx_topic;
    int car_rpm_max;

    double main_servo_zero_point;
    double left_servo_zero_point;
    double right_servo_zero_point;
    int fre;
    int timeout_ms;
    bool curtis_theta_oppsite = false;
    bool curtis_vel_oppsite = false;
    bool zero_point_calibration_enable = false;

    nh_priv.getParam("left_canopen_id", left_canopen_id);
    nh_priv.getParam("right_canopen_id", right_canopen_id);
    nh_priv.getParam("main_servo_zero_point", main_servo_zero_point);
    nh_priv.getParam("left_servo_zero_point", left_servo_zero_point);
    nh_priv.getParam("right_servo_zero_point", right_servo_zero_point);
    nh_priv.getParam("fre", fre);
    nh_priv.getParam("timeout_ms", timeout_ms);
    nh_priv.getParam("servo_can_tx_topic", servo_can_tx_topic);
    nh_priv.getParam("curtis_can_tx_topic", curtis_can_tx_topic);
    nh_priv.getParam("servo_can_rx_topic", servo_can_rx_topic);
    nh_priv.getParam("curtis_can_rx_topic", curtis_can_rx_topic);
    nh_priv.getParam("car_rpm_max", car_rpm_max);
    nh_priv.getParam("k_passive_red_ratio", k_passive_red_ratio);
    nh_priv.getParam("near_threshold", near_threshold);
    nh_priv.getParam("config_file_path", config_file_path);
    nh_priv.getParam("zero_point_calibration_enable", zero_point_calibration_enable);
    nh_priv.getParam("curtis_theta_oppsite", curtis_theta_oppsite);
    nh_priv.getParam("curtis_vel_oppsite", curtis_vel_oppsite);
    nh_priv.getParam("Curtis_Steer_Enable", Curtis_Steer_Enable);

    err_pub = nh.advertise<common::ERROR>("ERROR", 10);

    sd_left = std::make_shared<servo_driver>(nh, left_canopen_id, "left",
                                             servo_can_tx_topic, left_servo_zero_point, k_passive_red_ratio, true);
    sd_right = std::make_shared<servo_driver>(nh, right_canopen_id, "right",
                                              servo_can_tx_topic, right_servo_zero_point, k_passive_red_ratio, false);
    cd = std::make_shared<curtis_driver>(nh, curtis_can_tx_topic, car_rpm_max,
                                         main_servo_zero_point, curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable);

    ros::Subscriber sub_omv_servo_cmd_ = nh.subscribe("/omv_servo_cmd", 8, omv_servo_cmd_callback);
    omv_servo_encoder_pub = nh.advertise<common::omv_servo_encoder>("/omv_servo_encoder", 10); //
    reset_curtis_pub = nh.advertise<std_msgs::Int8>("/reset_curtis_cmd", 10);                  //

    dynamic_reconfigure::Server<ac_node::ServoConfig> srv;
    if (zero_point_calibration_enable)
    {
        srv.setCallback(boost::bind(&reconfig, _1, _2));
    }
    ros::Subscriber servo_can_rx_sub = nh.subscribe(servo_can_rx_topic, 1000, servo_can_recv_callback);
    ros::Subscriber curtis_can_rx_sub = nh.subscribe(curtis_can_rx_topic, 1000, curtis_can_recv_callback);

    ros::Subscriber maunal_mode_sub = nh.subscribe("/peripheral_devs_state", 100, get_maual_auto);

    double period;
    if (fre > 2)
        period = 1.0f / fre;
    else
        period = 0.05f;
    timer = nh.createTimer(ros::Duration(period), can_send_callback);
    ros::spin();
    return 0;
}