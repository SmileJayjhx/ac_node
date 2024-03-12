#include "iostream"
#include "common/can.h"
#include "common/servo_cmd.h"
#include "common/servo_encoder.h"
#include "ros/ros.h"
#include <map>

#define NMT 0
#define TPDO1 0x180
#define RPDO1 0x200
#define TPDO2 0x280
#define RPDO2 0x300
#define TPDO3 0x380
#define RPDO3 0x400
#define TPDO4 0x480
#define RPDO4 0x500

class servo_driver_banyunche
{


    int canopen_id_;
    std::string postfix_;
    double servo_zero_point_;

    ros::Publisher servo_encoder_pub;
    ros::Publisher can_tx_pub;
    bool oppsite_;
    bool inited;
    double last_recv_time_;
    double last_send_time_;
    common::servo_cmd servo_cmd_cache;
    ros::NodeHandle nh_;
    ros::NodeHandle nh_priv_;
    double k_red_ratio_;
    uint8_t heartbeat1 = 0; // 心跳位
    int is_manual_flag = 0; // 模式
    common::can can_s1;
    uint8_t cmd_clearflag = 0; //cmd命令持续有效标志位

    //定义方向标志位
    bool angle_updir_flag;
    bool angle_downdir_flag;

public:
    servo_driver_banyunche(ros::NodeHandle &nh,
                 ros::NodeHandle &nh_priv,
                 int canopen_id,
                 std::string postfix,
                 std::string can_tx_topic,
                 double servo_zero_point,
                 double k_red_ratio,
                 bool oppsite);
    ~servo_driver_banyunche();
    void can_send_callback();
    void car_downloadCallback(const common::servo_cmd::ConstPtr &msg);
    void set_zero_point(double servo_zero_point);
    void set_manual_flag(int manual_flag);
    int can_recv(const common::can::ConstPtr &msg, double &theta);
    void set_servo_cmd(const double &theta);
    void send_pose_cmd(double theta_in);
    int timer_can_send(void);
    double agv_theta_;
    void can_downloadCallback1(const common::can::ConstPtr &msg);

};