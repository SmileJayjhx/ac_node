#ifndef AC_DRIVER_MANAGER_H
#define AC_DRIVER_MANAGER_H

#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include <mutex>
#include "common/peripheral_uart.h"
#include <dynamic_reconfigure/server.h>
#include <ac_node/ServoConfig.h>
#include <fstream>
#include <string>
#include "ac_driver.h"
#include "curtis_driver.h"
#include "mima_driver.h"
#include "servo_driver.h"
#include "rebot_driver.h"
#include <YJL_driver.h>
#include "common/ERROR.h"
#include "common/omv_servo_cmd.h"
#include "common/omv_servo_encoder.h"
#include "std_msgs/Int8.h"
#include <memory>
#include <vector>
#include <functional>

class ACDriverManager {
public:
    ACDriverManager();
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);
    void run();

private:
    // 成员变量
    bool Curtis_Steer_Enable;
    int left_canopen_id;
    int right_canopen_id;
    std::vector<std::shared_ptr<ac_driver>> ac_drivers;
    std::string config_file_path;
    std::string config_file;

    ros::Timer timer;
    ros::Publisher servo_encoder_pub;
    ros::Publisher reset_curtis_pub;
    ros::Publisher err_pub;
    ros::Subscriber sub_omv_servo_cmd_;
    ros::Subscriber fork_reset_ac_sub_;
    ros::Subscriber curtis_can_rx_sub_;
    ros::Subscriber servo_can_rx_sub_;
    ros::Subscriber manual_mode_sub_;

    double steering_theta;
    double k_passive_red_ratio;
    common::servo_encoder servo_encoder_msg;
    double near_threshold;
    double servo_near_threshold;
    int is_manual_mode;
    bool fork_need_reset;
    std::string vehicle_brand;
    std::string vehicle_type;
    double servo_zero_point;
    double left_servo_zero_point;
    double right_servo_zero_point;
    std::string servo_can_tx_topic;
    std::string curtis_can_tx_topic;
    std::string servo_can_rx_topic;
    std::string can_rx_topic;
    std::string cmd_topic;

    int car_rpm_max;
    bool curtis_theta_oppsite;
    bool curtis_vel_oppsite;
    bool zero_point_calibration_enable = false;

    // 私有函数
    void save_zero_point(double save_zero_point);
    void reconfig(ac_node::ServoConfig &config, uint32_t level);
    void initializeDrivers(ros::NodeHandle& nh);
    void handle_curtis_reset();
    void fork_reset_ac_Callback(const std_msgs::Int8::ConstPtr &msg);
    void can_send_callback(const ros::TimerEvent &);
    void servo_can_recv_callback(const common::can::ConstPtr &msg);
    void can_recv_callback(const common::can::ConstPtr &msg);
    void servo_cmd_callback(const common::servo_cmd::ConstPtr &msg);
    void get_manual_auto(const common::peripheral_uart::ConstPtr &msg);
};

#endif // AC_DRIVER_MANAGER_H
