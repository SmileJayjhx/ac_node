#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int16MultiArray.h"

#include "common/peripheral_uart.h"
#include <dynamic_reconfigure/server.h>
#include <ac_node/ServoConfig.h>
#include <fstream>
#include <string>
#include "common/can.h"
#include "common/servo_cmd.h"
#include "common/servo_encoder.h"
#include "mima_driver.h"
#include "std_msgs/Int8.h"

std::string config_file_path;
std::string config_file = "servo_zero_point.yaml";
ros::Publisher err_pub;
ros::Publisher can_tx_pub;
double k_passive_red_ratio;
double near_threshold;
double servo_zero_point;
ros::Publisher servo_encoder_pub;
int is_manual_mode;
bool fork_need_reset = false;
std::shared_ptr<mima_driver> md;
common::servo_encoder servo_encoder;
common::servo_cmd servo_cmd;
std::string can_tx_topic = "/can0_tx";
std::string can_rx_topic = "/can0_rx";
int car_rpm_max;
ros::Time last_cmd_time;
int fre;
int timeout_ms;
bool theta_oppsite = false;
bool vel_oppsite = false;
bool zero_point_calibration_enable = false;
int get_param(ros::NodeHandle &nh_priv)
{
    bool result;
    result = nh_priv.getParam("main_servo_zero_point", servo_zero_point);
    if (!result)
    {
        printf("Parameter %s not found", "main_servo_zero_point");
        ROS_ERROR("Parameter %s not found", "main_servo_zero_point");
        return -1;
    }
    result = nh_priv.getParam("fre", fre);
    if (!result)
    {
        ROS_ERROR("Parameter %s not found", "fre");
        return -1;
    }
    // result = nh_priv.getParam("timeout_ms", timeout_ms);
    // if(!result) {
    //     ROS_ERROR("Parameter %s not found","timeout_ms");
    //     return -1;
    // }
    result = nh_priv.getParam("mima_can_tx_topic", can_tx_topic);
    if (!result)
    {
        ROS_ERROR("Parameter %s not found", "mima_can_tx_topic");
        return -1;
    }
    result = nh_priv.getParam("mima_can_rx_topic", can_rx_topic);
    if (!result)
    {
        ROS_ERROR("Parameter %s not found", "mima_can_rx_topic");
        return -1;
    }
    result = nh_priv.getParam("car_rpm_max", car_rpm_max);
    if (!result)
    {
        ROS_ERROR("Parameter %s not found", "car_rpm_max");
        return -1;
    }
    // result = nh_priv.getParam("k_passive_red_ratio", k_passive_red_ratio);
    // if(!result) {
    //     ROS_ERROR("Parameter %s not found","k_passive_red_ratio");
    //     return -1;
    // }
    result = nh_priv.getParam("near_threshold", near_threshold);
    if (!result)
    {
        ROS_ERROR("Parameter %s not found", "near_threshold");
        return -1;
    }
    result = nh_priv.getParam("config_file_path", config_file_path);
    if (!result)
    {
        ROS_ERROR("Parameter %s not found", "config_file_path");
        return -1;
    }
    result = nh_priv.getParam("zero_point_calibration_enable", zero_point_calibration_enable);
    if (!result)
    {
        ROS_ERROR("Parameter %s not found", "zero_point_calibration_enable");
        return -1;
    }
    result = nh_priv.getParam("theta_oppsite", theta_oppsite);
    if (!result)
    {
        ROS_ERROR("Parameter %s not found", "theta_oppsite");
        return -1;
    }
    result = nh_priv.getParam("vel_oppsite", vel_oppsite);
    if (!result)
    {
        ROS_ERROR("Parameter %s not found", "vel_oppsite");
        return -1;
    }
}

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

void reconfig(ac_node::ServoConfig &config, uint32_t level)
{
    static bool save_data = false;
    servo_zero_point = config.servo_zero_point;
    md->set_zero_point(config.servo_zero_point);
    std::cout << servo_zero_point << std::endl;
    if (config.save_data && !save_data)
    {
        save_zero_point(servo_zero_point);
    }
    save_data = config.save_data;
}

void timer_callback(const ros::TimerEvent &)
{
    // std::cout << "is_manual_mode: " << is_manual_mode << std::endl;
    // 超时
    if (ros::Time::now().toSec() - servo_cmd.header.stamp.toSec() > 0.2)
    {
        servo_cmd.sc_theta = 0;
        servo_cmd.sc_vel = 0;
        // std::cout << "超时 " << std::endl;
    }
    if (is_manual_mode == 1)
    {
        common::can can_tx_send_303;
        can_tx_send_303.id = 0x303; //CAN_ID
        can_tx_send_303.len = 8;    //CAN数据长度
        can_tx_send_303.datas.resize(can_tx_send_303.len);
        can_tx_send_303.header.stamp = ros::Time::now();
        can_tx_send_303.datas[0] = 0x00;
        can_tx_send_303.datas[1] = 0x00;
        can_tx_send_303.datas[2] = 0x00;
        can_tx_send_303.datas[3] = 0x00;
        can_tx_send_303.datas[4] = 0x00;
        can_tx_send_303.datas[5] = 0x00;
        can_tx_send_303.datas[6] = 0x00;
        can_tx_send_303.datas[7] = 0x00;

        common::can can_tx_send;
        can_tx_send.len = 8; //CAN数据长度
        can_tx_send.datas.resize(can_tx_send.len);
        can_tx_send.datas[0] = 0x00;
        can_tx_send.datas[1] = 0x00;
        can_tx_send.datas[2] = 0x00;
        can_tx_send.datas[3] = 0x00;
        can_tx_send.datas[4] = 0x00;
        can_tx_send.datas[5] = 0x00;
        can_tx_send.datas[6] = 0x00;
        can_tx_send.datas[7] = 0x00;
        common::can can_tx;
        std::cout << "timer_callback servo_cmd.sc_vel: " << servo_cmd.sc_vel << std::endl;
        std::cout << "timer_callback servo_cmd.sc_theta: " << servo_cmd.sc_theta << std::endl;
        md->solve_servo_cmd(servo_cmd, can_tx);
        std::cout << "md->need_reset(): " << md->need_reset() << std::endl;
        if (md->need_reset())
        {
            for (int i = 0; i < 20; i++)
            {
                can_tx_send.datas[0] = 0x00;
                can_tx_pub.publish(can_tx_send);
                ros::Duration(0.002).sleep();
                can_tx_pub.publish(can_tx_send_303);
                ros::Duration(0.002).sleep();
            }
            can_tx_send.datas[0] = 0x01;
            can_tx_pub.publish(can_tx_send);
            ros::Duration(0.002).sleep();
            can_tx_pub.publish(can_tx_send_303);
            ros::Duration(0.002).sleep();
            can_tx_send.datas[0] = can_tx.datas[0];
            can_tx_pub.publish(can_tx_send);
            ros::Duration(0.002).sleep();
            can_tx_pub.publish(can_tx_send_303);
            ros::Duration(0.002).sleep();
        }
        can_tx_send.id = can_tx.id;
        can_tx_send.len = can_tx.len;
        can_tx_send.datas[0] = can_tx.datas[0];
        can_tx_send.datas[1] = can_tx.datas[1];
        can_tx_send.datas[2] = can_tx.datas[2];
        can_tx_send.datas[3] = can_tx.datas[3];
        can_tx_send.datas[4] = can_tx.datas[4];
        can_tx_send.datas[5] = can_tx.datas[5];
        can_tx_send.datas[6] = can_tx.datas[6];
        can_tx_send.datas[7] = can_tx.datas[7];
        can_tx_pub.publish(can_tx_send);
        ros::Duration(0.002).sleep();
        can_tx_pub.publish(can_tx_send_303);
    }
    else
    {
        return;
    }
}

void can_recv_callback(const common::can::ConstPtr &msg)
{
    if (msg->id == 0x183 || msg->id == 0x383)
    {
        md->solve_servo_encoder(*msg, servo_encoder);
        servo_encoder.header.stamp = ros::Time::now();
        std::cout << "servo_encoder.se_vel: " << servo_encoder.se_vel << " servo_encoder.se_theta: " << servo_encoder.se_theta << std::endl;
        servo_encoder_pub.publish(servo_encoder);
    }
}

void servo_cmd_callback(const common::servo_cmd::ConstPtr &msg)
{
    // // last_cmd_time = msg->header.stamp;
    // // std::cout << "msg->sc_vel: " << std::hex << msg->sc_vel << std::endl;
    // // std::cout << "msg->sc_theta: " << std::hex << msg->sc_theta << std::endl;
    // std::cout << "msg->sc_vel: "  << msg->sc_vel << std::endl;
    // std::cout << "msg->sc_theta: "  << msg->sc_theta << std::endl;
    servo_cmd.header.stamp = ros::Time::now();
    servo_cmd.sc_theta = msg->sc_theta;
    servo_cmd.sc_vel = msg->sc_vel;
    // std::cout << "servo_cmd_callback servo_cmd.sc_vel: "  << servo_cmd.sc_vel << std::endl;
    // std::cout << "servo_cmd_callback servo_cmd.sc_theta: "  << servo_cmd.sc_theta << std::endl;
    // if (is_manual_mode == 1)
    // {
    //     common::can can_tx;
    //     std::cout << "timer_callback servo_cmd.sc_vel: "  << servo_cmd.sc_vel << std::endl;
    //     std::cout << "timer_callback servo_cmd.sc_theta: "  << servo_cmd.sc_theta << std::endl;
    //     md->solve_servo_cmd(servo_cmd,can_tx);
    //     can_tx_pub.publish(can_tx);
    //     ros::Duration(0.025).sleep();
    //     can_tx.id = 0x303; //CAN_ID
    //     can_tx.len = 8;    //CAN数据长度
    //     can_tx.datas.resize(can_tx.len);
    //     can_tx.header.stamp = ros::Time::now();
    //     can_tx.datas[0] = 0x00; //前进
    //     can_tx.datas[1] = 0x00;
    //     can_tx.datas[2] = 0x00;
    //     can_tx.datas[3] = 0x00;
    //     can_tx.datas[4] = 0x00;
    //     can_tx.datas[5] = 0x00;
    //     can_tx.datas[6] = 0x00;
    //     can_tx.datas[7] = 0x00;
    //     can_tx_pub.publish(can_tx);
    // }
    // else
    // {
    //     return;
    // }
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
        // cd->init();
    }
    last_mode = is_manual_mode;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ac_driver_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ros::Timer timer;
    get_param(nh_priv);
    ros::Subscriber can_rx_sub = nh.subscribe(can_rx_topic, 100, can_recv_callback);
    ros::Subscriber maunal_mode_sub = nh.subscribe("/peripheral_devs_state", 100, get_maual_auto);

    servo_encoder_pub = nh.advertise<common::servo_encoder>("/servo_encoder", 10); //
    double period = 0.05;
    timer = nh.createTimer(ros::Duration(period), timer_callback);
    servo_cmd.sc_theta = 0;
    servo_cmd.sc_vel = 0;
    ros::Subscriber servo_cmd_sub = nh.subscribe("/servo_cmd", 8, servo_cmd_callback);
    can_tx_pub = nh.advertise<common::can>(can_tx_topic, 100);

    md = std::make_shared<mima_driver>(nh, car_rpm_max, servo_zero_point, theta_oppsite, vel_oppsite);

    dynamic_reconfigure::Server<ac_node::ServoConfig> srv;
    if (zero_point_calibration_enable) //零点校正
    {
        std::cout << "zero_point_calibration " << std::endl;
        srv.setCallback(boost::bind(&reconfig, _1, _2));
    }
    ros::spin();
    return 0;
}