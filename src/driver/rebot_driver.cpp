#include <rebot_driver.h>
rebot_driver::rebot_driver(ros::NodeHandle &nh,
                           std::string can_tx_topic,
                           int car_rpm_max,
                           double servo_zero_point,
                           bool theta_oppsite,
                           bool vel_oppsite,
                           bool curtis_steer_enable,
                           double threshold)
    : inited(false),
      servo_zero_point_(servo_zero_point),
      nh_(nh),
      car_rpm_max(car_rpm_max),
      heartbeat(0),
      theta_oppsite_(theta_oppsite),
      vel_oppsite_(vel_oppsite),
      init_cnt_(60),
      send_enable_(false),
      syn_enable_(false),
      acc_k_(1.0),
      curtis_steer_enable_(curtis_steer_enable),
      threshold_(threshold)
{
    if (theta_oppsite_)
        servo_zero_point_ = -servo_zero_point_;
    can_tx_pub = nh_.advertise<common::can>(can_tx_topic, 1000);
    // std::cout << "can_tx_topic: " << can_tx_topic << std::endl;
    
    servo_encoder_pub = nh_.advertise<common::servo_encoder>("/servo_encoder", 1000); //
    last_send_time_ = ros::Time::now().toSec();
    last_recv_time_ = ros::Time::now().toSec();
    status = AGV_Status::INVALID;
    servo_cmd_cache.sc_theta = 0;
    servo_cmd_cache.sc_vel = 0;
    enable_send();
}

rebot_driver::~rebot_driver()
{
}

void rebot_driver::set_zero_point(double servo_zero_point)
{
    servo_zero_point_ = servo_zero_point;
    if (theta_oppsite_)
        servo_zero_point_ = -servo_zero_point_;
}
bool rebot_driver::judge_angle_near(const common::servo_cmd::ConstPtr &msg, common::servo_encoder se)
{
    double thetax=msg->sc_theta;
   double  thetay=se.se_theta;
    double err = thetax - thetay;
    while (err < -180)
        err += 360;
    while (err > 180)
        err -= 360;
    // std::cout << "err:" << thetax << " " << thetay << " " << err << std::endl;
    if (fabs(err) < threshold_)
        return true;
    else
        return false;
}
void rebot_driver::set_servo_cmd(const common::servo_cmd::ConstPtr &msg,bool syn_enable)
{
    ROS_ERROR_STREAM("enter set_servo_cmd ");
    double theta = msg->sc_theta;
    double vel = msg->sc_vel;
    if (theta_oppsite_)
        theta = -theta;
    if (vel_oppsite_)
        vel = -vel;
    syn_enable_ =  syn_enable;
    last_send_time_ = ros::Time::now().toSec();
    servo_cmd_cache.sc_theta = theta;
    servo_cmd_cache.sc_vel = vel;
    std::cout<<" servo_cmd_cache.sc_vel"<< servo_cmd_cache.sc_vel<<std::endl;
}

// theta :绝对角度，单位度
// vel：运行速度，无符号，单位转每分钟
void rebot_driver::send_servo_cmd(common::servo_cmd &sc)
{
    static double theta_out = 0;
    int16_t theta = sc.sc_theta + servo_zero_point_;
    // std::cout << "send_servo_cmd sc.sc_theta:  " << sc.sc_theta << std::endl;
    // std::cout << "send_servo_cmd servo_zero_point_:  " << servo_zero_point_ << std::endl;
    // std::cout << "send_servo_cmd theta:  " << theta << std::endl;
    int16_t sc_angle;
    int16_t sc_speed;
    if (theta > 90)
        theta = 90;
    else if (theta < -90)
        theta = -90;
    sc_angle = (int16_t)(theta * 100);
    sc_speed = (int16_t)sc.sc_vel;
    // std::cout << "send_servo_cmd sc_speed:  " << sc_speed << std::endl;
    // std::cout << "send_servo_cmd sc_angle:  " << sc_angle << std::endl;
    common::can can_s;
    can_s.id = 0x213; // CAN_ID
    can_s.len = 8;    // CAN数据长度
    can_s.datas.resize(can_s.len);
    can_s.header.stamp = ros::Time::now();
    if (sc_speed > 0)
    {
        can_s.datas[0] = 0x85; // 前进
        can_s.datas[1] = 0;
        can_s.datas[2] = 0x20;
        can_s.datas[3] = 0x20;
        can_s.datas[4] = sc_angle & 0x00ff;
        can_s.datas[5] = (sc_angle >> 8) & 0x00ff;
        can_s.datas[6] = uint8_t(sc_speed & 0x00ff);
        can_s.datas[7] = uint8_t((sc_speed >> 8) & 0x00ff);
    }
    else if (sc_speed < 0)
    {
        int16_t sc_speed_ll = 0 - sc_speed;
        can_s.datas[0] = 0x89;
        can_s.datas[1] = 0;
        can_s.datas[2] = 0x20;
        can_s.datas[3] = 0x20;
        can_s.datas[4] = sc_angle & 0x00ff;
        can_s.datas[5] = (sc_angle >> 8) & 0x00ff;
        can_s.datas[6] = uint8_t(sc_speed_ll & 0x00ff);
        can_s.datas[7] = uint8_t((sc_speed_ll >> 8) & 0x00ff);
    }
    else
    {
        can_s.datas[0] = 0x83;
        can_s.datas[1] = 0;
        can_s.datas[2] = 0;
        can_s.datas[3] = 0;
        can_s.datas[4] = sc_angle & 0x00ff;
        can_s.datas[5] = (sc_angle >> 8) & 0x00ff;
        can_s.datas[6] = 0;
        can_s.datas[7] = 0;
    }
    if (manual_mode_ == 0)
    {
        can_s.datas[0] = 0x80; // 手动模式，要释放
    }
    // 发布can_tx消息
    can_tx_pub.publish(can_s);
    // can_tx_pub.publish(can_s2);
}

int rebot_driver::controller_error_report(const common::can::ConstPtr &msg, std::string &error_msg_s)
{
    return 0;
}

// 收到车体的反馈，解析舵轮的角度和线速度
int rebot_driver::controller_state_report(const common::can::ConstPtr &msg, common::servo_encoder &se, std::string &error_msg_s)
{
    last_recv_time_ = ros::Time::now().toSec();
    int16_t ang_l, speed_l;

    if (msg->id == 0x207)
    {
        speed_l = (int16_t)((msg->datas[7] << 8 & 0xff00) | (msg->datas[6] & 0x00ff));
        ang_l = (int16_t)((msg->datas[5] << 8 & 0xff00) | (msg->datas[4] & 0x00ff));
        // if (speed_updir_flag == false)
        // {
        //     speed_l = 0 - speed_l;
        // }
        // if (!angle_updir_flag)
        // {
        //     ang_l = 0 - ang_l;
        // }
        se.se_vel = speed_l;
        se.se_theta = ang_l / 100.0f - servo_zero_point_;
        se.se_theta = ang_l / 100.0f;
        // std::cout << "recv_207 se.se_vel:  " << se.se_vel << std::endl;
        // std::cout << "recv_207 ang_l:  " << ang_l << std::endl;
        // std::cout << "recv_207 se.se_theta:  " << se.se_theta << std::endl;
        se.header.stamp = msg->header.stamp;
        servo_encoder_pub.publish(se);
    }

    encoder_vel_ = se.se_vel;
    encoder_theta_ = se.se_theta;
    return 0;
}

int rebot_driver::timer_can_send(void)
{
    double cur_time = ros::Time::now().toSec();
    if (cur_time - last_recv_time_ > 0.2)
    {
        encoder_vel_ = 0;
        std::cout << "main servo encoder too old" << std::endl;
    }
    if (cur_time - last_send_time_ > 0.2)
    {
        // std::cout << "main servo cmd too old" << std::endl;
        servo_cmd_cache.sc_theta = 0;
        servo_cmd_cache.sc_vel = 0;
        // return -2;
    }
    // std::cout << "syn_enable_(0 校正轮子姿态;1 正常行使): " << syn_enable_ << std::endl;
    if (!syn_enable_)
    {
        common::servo_cmd servo_cmd_;
        servo_cmd_.sc_theta = servo_cmd_cache.sc_theta;
        servo_cmd_.sc_vel = 0;
        send_servo_cmd(servo_cmd_);
    }
    else
    {
        common::servo_cmd servo_cmd_;
        servo_cmd_.sc_theta = servo_cmd_cache.sc_theta;
        servo_cmd_.sc_vel = servo_cmd_cache.sc_vel * acc_k_;
        send_servo_cmd(servo_cmd_);
        // std::cout << "send_servo_cmd" << std::endl;
    }
    return 0;
}

bool rebot_driver::need_reset(void)
{
}

void rebot_driver::set_manual_mode(int manual_mode)
{
    manual_mode_ = manual_mode;
}

void rebot_driver::init(void)
{
    init_cnt_ = 60;
}

void rebot_driver::disable_send(void)
{
    send_enable_ = false;
}

void rebot_driver::enable_send(void)
{
    send_enable_ = true;
}
