#include "iostream"
#include "common/can.h"
#include "common/servo_cmd.h"
#include "common/servo_encoder.h"
#include "ros/ros.h"
#include <map>
#include "ac_driver.h"
#define NMT 0
#define TPDO1 0x180
#define RPDO1 0x200
#define TPDO2 0x280
#define RPDO2 0x300
#define TPDO3 0x380
#define RPDO3 0x400
#define TPDO4 0x480
#define RPDO4 0x500

class servo_driver : public ac_driver
{

    inline int ENABLE_FC(int x) { return ((x) + RPDO1); }
    inline int MODE_FC(int x) { return ((x) + RPDO2); }
    inline int SPEED_CMD_FC(int x) { return ((x) + RPDO3); }
    inline int POSITION_CMD_FC(int x) { return ((x) + RPDO4); }

    inline int STATUS_FC(int x) { return ((x) + TPDO1); }
    inline int ERROR_FC(int x) { return ((x) + TPDO2); }
    inline int SPEED_FEEDBACK_FC(int x) { return ((x) + TPDO3); }
    inline int POSITION_FEEDBACK_FC(int x) { return ((x) + TPDO4); }
    int canopen_id_;
    std::string postfix_;
    double servo_zero_point_;
    double encoder_vel_;
    double encoder_theta_;
    bool syn_enable_;
    int manual_mode_;
    ros::Publisher servo_encoder_pub;
    ros::Publisher can_tx_pub;
    bool oppsite_;
    bool inited;
    double last_recv_time_;
    double last_send_time_;
    common::servo_cmd servo_cmd_cache;
    ros::NodeHandle nh_;
    double k_red_ratio_;
    double threshold_;
    void init_pose_ctrl(void);
    void init_speed_ctrl(void);
    void enable_speed_ctrl(void);
    void enable_pose_ctrl(void);
    void disable_pose_ctrl(void);
    void disable_speed_ctrl(void);
    void send_pose_cmd(double theta, int vel);
    void send_speed_cmd(int vel);

public:
    servo_driver(ros::NodeHandle &nh,
                 int canopen_id,
                 std::string postfix,
                 std::string can_tx_topic,
                 double servo_zero_point,
                 double k_red_ratio,
                 bool oppsite,
                 double threshold);
    ~servo_driver();
    void set_zero_point(double servo_zero_point);
    void set_manual_mode(int manual_mode);
     int controller_state_report(const common::can::ConstPtr &msg, common::servo_encoder &se, std::string &error_msg_s);
    void set_servo_cmd(const common::servo_cmd::ConstPtr &msg, bool syn_enable);
    int timer_can_send(void);
    bool judge_angle_near(const common::servo_cmd::ConstPtr &msg, common::servo_encoder se);
    void init(void);
    void send_servo_cmd(common::servo_cmd &se);
    int controller_error_report(const common::can::ConstPtr &msg, std::string &error_msg_s);
    void disable_send(void);
    void enable_send(void);
    bool need_reset(void);
};