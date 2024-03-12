#include "iostream"
#include "common/can.h"
#include "common/servo_cmd.h"
#include "common/servo_encoder.h"
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include "ac_driver.h"
// can通讯协议
#define car_forward_flag 1
#define car_back_flag 2
#define car_move_enable_flag 4
#define car_agv_enable_flag 8

using namespace std;


class YJL_driver : public ac_driver
{
    double servo_zero_point_;

    ros::Publisher servo_encoder_pub;

    bool inited;
    double last_recv_time_;
    double last_send_time_;
    common::servo_cmd servo_cmd_cache;
    ros::NodeHandle nh_;
    AGV_Status status;
    double car_rpm_xs;
    int car_rpm_max;
    int manual_mode_;
    uint8_t heartbeat;
    bool theta_oppsite_;
    bool vel_oppsite_;
    int init_cnt_;
    bool send_enable_;
    double encoder_vel_;
    double encoder_theta_;
    bool curtis_steer_enable_;
    double acc_k_;
    double threshold_;
    ros::Publisher can_tx_pub;
    bool judge_angle_near(const common::servo_cmd::ConstPtr &msg, common::servo_encoder se);

public:
    YJL_driver(ros::NodeHandle &nh,
                 std::string can_tx_topic,
                 int car_rpm_max,
                 double servo_zero_point,
                 bool theta_oppsite,
                 bool vel_oppsite,
                 bool curtis_steer_enable,
                 double threshold);
    ~YJL_driver();
    bool syn_enable_;
    void init(void);
    void set_manual_mode(int manual_mode); // 自动 1，手动 0
    // 写入最新速度，定时器回调才下发
    void set_servo_cmd(const common::servo_cmd::ConstPtr &msg,bool syn_enable);
    void set_zero_point(double servo_zero_point);
    // 下发控制指令
    void send_servo_cmd(common::servo_cmd &se);
    // 接收车体状态反馈
    int controller_error_report(const common::can::ConstPtr &msg, std::string &error_msg_s);

    int controller_state_report(const common::can::ConstPtr &msg, common::servo_encoder &se, std::string &error_msg_s);

    int timer_can_send(void);

    void disable_send(void);
    void enable_send(void);
    bool need_reset(void);
};