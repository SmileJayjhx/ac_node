#include "mima_driver.h"
mima_driver::mima_driver(ros::NodeHandle &nh,
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
    can_tx_pub = nh_.advertise<common::can>("/mima_move", 1000);
    std::cout << "can_tx_topic: " << can_tx_topic << std::endl;
    servo_encoder_pub = nh_.advertise<common::servo_encoder>("/servo_encoder", 1000); //
    last_send_time_ = ros::Time::now().toSec();
    last_recv_time_ = ros::Time::now().toSec();
    status = AGV_Status::INVALID;
    servo_cmd_cache.sc_theta = 0;
    servo_cmd_cache.sc_vel = 0;
    enable_send();
}
mima_driver::~mima_driver()
{
}

void mima_driver::set_zero_point(double servo_zero_point)
{
    servo_zero_point_ = servo_zero_point;
    if (theta_oppsite_)
        servo_zero_point_ = -servo_zero_point_;
}

bool mima_driver::judge_angle_near(const common::servo_cmd::ConstPtr &msg, common::servo_encoder se)
{
    double thetax=msg->sc_theta;
    double thetay=se.se_theta;
    double err = thetax - thetay;
    while (err < -180)
        err += 360;
    while (err > 180)
        err -= 360;
    std::cout << "err:" << thetax << " " << thetay << " " << err << std::endl;
    if (fabs(err) < threshold_)
        return true;
    else
        return false;
}

void mima_driver::set_servo_cmd(const common::servo_cmd::ConstPtr &msg,bool syn_enable) // double theta, int vel, bool syn_enable
{
    std::cout<<"enter mima set_servo_cmd"<<std::endl;
    double theta;
    double vel;
    theta =msg->sc_theta;
    vel = msg->sc_vel;
    if (theta_oppsite_)
        theta = -theta;
    if (vel_oppsite_)
        vel = -vel;
    std::cout<<"vel"<<vel<<"msg->sc_vel"<<msg->sc_vel<<std::endl;
    std::cout<<"theta"<<theta<<"msg->sc_theta"<<msg->sc_theta<<std::endl;
    syn_enable_ = syn_enable;
    last_send_time_ = ros::Time::now().toSec();
    servo_cmd_cache.sc_theta = theta;
    servo_cmd_cache.sc_vel = vel;
    std::cout<<"servo_cmd_cache.sc_theta"<<servo_cmd_cache.sc_theta<<std::endl;
    std::cout<<"servo_cmd_cache.sc_vel"<<servo_cmd_cache.sc_vel<<std::endl;
}

// theta :绝对角度，单位度
// vel：运行速度，无符号，单位转每分钟
void mima_driver::send_servo_cmd(common::servo_cmd &sc)
{
    int16_t input_angle = (int16_t)(sc.sc_theta + servo_zero_point_);
    int16_t input_speed =(int16_t) sc.sc_vel;
    std::cout<<"input_speed"<<input_speed<<"sc.sc_vel"<<sc.sc_vel<<std::endl;
    // input_angle = input_angle + servo_zero_point_;
    // std::cout << "debug 2 " << std::hex << sc.sc_theta << std::endl;
    if (fabs(input_speed) != 0 && fabs(input_speed) < 50)
    {
        input_speed = (50*(input_speed/fabs(input_speed)));
    }
    if (input_speed > 4000)
    {
        input_speed = (4000*(input_speed/fabs(input_speed)));
    }
    if (input_angle > 90)
        input_angle = 90;
    else if (input_angle < -90)
        input_angle = -90;

    common::can can_tx;
    can_tx.id = 0x203; // CAN_ID
    can_tx.len = 8;    // CAN数据长度
    can_tx.datas.resize(can_tx.len);
    can_tx.header.stamp = ros::Time::now();
    if (input_speed < 0)
    {
        can_tx.datas[0] = 0x05; // 后退
        input_speed = -input_speed;
    }
    else if(input_speed > 0)
    {
        can_tx.datas[0] = 0x03; // 前进
        input_speed = input_speed;
    }
    else
    {
        can_tx.datas[0] = 0x01; // 前进
        input_speed = input_speed;
    }

    // std::cout << "solve_servo_cmd input_angle:  " << input_angle << std::endl;
    // std::cout << "solve_servo_cmd se_theta_:  " << se_theta_ << std::endl;

    int16_t sc_angle;
    int16_t sc_speed;
    sc_speed = (int16_t)input_speed;
    sc_angle = (int16_t)(input_angle * 100);
    std::cout << "solve_servo_cmd sc_speed:  " << sc_speed << std::endl;
    std::cout << "solve_servo_cmd sc_angle:  " << sc_angle << std::endl;
    std::cout << "solve_servo_cmd servo_zero_point_:  " << servo_zero_point_ << std::endl;
    can_tx.datas[1] = uint8_t(sc_speed & 0x00ff);
    can_tx.datas[2] = uint8_t((sc_speed >> 8) & 0x00ff);
    can_tx.datas[5] = uint8_t(sc_angle & 0x00ff);
    can_tx.datas[6] = uint8_t((sc_angle >> 8) & 0x00ff);
    can_tx.datas[7] = 0x00;
    can_tx_pub.publish(can_tx);
    ros::Duration(0.002).sleep();
    common::can can_tx_send_303;
    can_tx_send_303.id = 0x303; // CAN_ID
    can_tx_send_303.len = 8;    // CAN数据长度
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
    can_tx_pub.publish(can_tx_send_303);
    std::cout << "-------------------------------------------------------------- \n\n"
              << std::endl;
}

int mima_driver::controller_error_report(const common::can::ConstPtr &msg, std::string &error_msg_s)
{
    return 0;
}

// 收到车体的反馈，解析舵轮的角度和线速度
int mima_driver::controller_state_report(const common::can::ConstPtr &msg, common::servo_encoder &se, std::string &error_msg_s)
{
    if (msg->id == 0x183)
    {
        uint8_t tmp_data[2];
        tmp_data[0] = msg->datas[0];
        tmp_data[1] = msg->datas[1];
        int16_t * tmp_i =( int16_t*)(& tmp_data[0] );
        encoder_vel_ = * tmp_i;

        if (vel_oppsite_)
            encoder_vel_ = -encoder_vel_;
    }
   else  if (msg->id == 0x383)
    {
        int16_t se_theta;
        uint8_t data_high, data_low;
        data_low = msg->datas[0];
        data_high = msg->datas[1];
        se_theta = data_high;
        se_theta = se_theta << 8 | data_low;
        std::cout << "data_high: " << data_high<<"data_low" <<data_low<< std::endl;
        std::cout << "solve_servo_encoder se_theta: " << se_theta << std::endl;
        if (theta_oppsite_)
            encoder_theta_ = -(((double)se_theta) / 100 - servo_zero_point_);
        else
            encoder_theta_ = ((double)se_theta) / 100 - servo_zero_point_;
        std::cout << "after_solve_servo_encoder se_theta_: " << encoder_theta_ << std::endl;
    }
    else 
        return 0;
    se.header.stamp = msg->header.stamp;
    se.se_vel = encoder_vel_;
    se.se_theta = encoder_theta_;
    servo_encoder_pub.publish(se);
    last_recv_time_ = ros::Time::now().toSec();
    return 0;
}

int mima_driver::timer_can_send(void)
{
    double cur_time = ros::Time::now().toSec();
    if (cur_time - last_recv_time_ > 0.2)
    {
        encoder_vel_ = 0;
        std::cout << "main servo encoder too old" << std::endl;
    }
    if (cur_time - last_send_time_ > 0.2)
    {
        std::cout << "main servo cmd too old" << std::endl;
        servo_cmd_cache.sc_theta = 0; // 下发的数值
        servo_cmd_cache.sc_vel = 0;
        // return -2;
    }
    std::cout<<"syn_enable_"<<syn_enable_<<std::endl;
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
        std::cout << "servo_cmd_.sc_theta"<<servo_cmd_.sc_theta << std::endl;
        std::cout << "servo_cmd_.sc_vel"<<servo_cmd_.sc_vel << std::endl;
        send_servo_cmd(servo_cmd_);
        std::cout << "send_servo_cmd" << std::endl;
    }
    return 0;
}

bool mima_driver::need_reset(void)
{
}

void mima_driver::set_manual_mode(int manual_mode)
{
    manual_mode_ = manual_mode;
}

void mima_driver::init(void)
{
    init_cnt_ = 60;
}

void mima_driver::disable_send(void)
{
    send_enable_ = false;
}

void mima_driver::enable_send(void)
{
    send_enable_ = true;
}
