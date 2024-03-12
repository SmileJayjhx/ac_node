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

class curtis_driver : public ac_driver
{
    double servo_zero_point_;

    ros::Publisher servo_encoder_pub;
    ros::Publisher can_tx_pub;
    ros::Publisher curtis_absent_pub;

    bool inited;
    double last_recv_time_;
    double last_send_time_;
    common::servo_cmd servo_cmd_cache;
    ros::NodeHandle nh_;
    AGV_Status status;
    double car_rpm_xs;
    int car_rpm_max;
    uint8_t heartbeat;
    std::map<int, std::string> error_msg;
    bool theta_oppsite_;
    bool vel_oppsite_;
    int init_cnt_;
    bool send_enable_;
    double encoder_vel_;
    double encoder_theta_;
    bool curtis_steer_enable_;
    double acc_k_;
    double threshold_;
    int manual_mode_;

public:
    curtis_driver(ros::NodeHandle &nh,
                  std::string can_tx_topic,
                  int car_rpm_max,
                  double servo_zero_point,
                  bool theta_oppsite,
                  bool vel_oppsite,
                  bool curtis_steer_enable,
                  double threshold);
    ~curtis_driver();
    bool syn_enable_;
    void init(void);

    void send_servo_cmd(common::servo_cmd &se);
    void set_servo_cmd(const common::servo_cmd::ConstPtr &msg, bool syn_enable);
    void set_zero_point(double servo_zero_point);
    bool judge_angle_near(const common::servo_cmd::ConstPtr &msg, common::servo_encoder se);
    int controller_error_report(const common::can::ConstPtr &msg, std::string &error_msg_s);
    void set_manual_mode(int manual_mode);
    int controller_state_report(const common::can::ConstPtr &msg, common::servo_encoder &se, std::string &error_msg_s);
    int timer_can_send(void);

    void disable_send(void);
    void enable_send(void);
    bool need_reset(void);
};