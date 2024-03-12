
#include <servo_driver.h>

servo_driver::servo_driver(ros::NodeHandle &nh,
                           int canopen_id,
                           std::string postfix,
                           std::string can_tx_topic,
                           double servo_zero_point,
                           double k_red_ratio,
                           bool oppsite,
                           double threshold)

    : canopen_id_(canopen_id),
      postfix_(postfix),
      inited(false),
      servo_zero_point_(servo_zero_point),
      nh_(nh),
      k_red_ratio_(k_red_ratio),
      oppsite_(oppsite),
      threshold_(threshold),
      syn_enable_(false)
{
    if (oppsite_)
        servo_zero_point_ = -servo_zero_point_;
    // servo_encoder_pub = nh_.advertise<common::servo_encoder>("/servo_encoder" + postfix_, 1000); //
    can_tx_pub = nh_.advertise<common::can>(can_tx_topic, 1000);

    last_send_time_ = ros::Time::now().toSec();
    last_recv_time_ = ros::Time::now().toSec();
    servo_cmd_cache.sc_theta = 0;
    servo_cmd_cache.sc_vel = 0;

    ros::WallDuration(3).sleep();
    init_pose_ctrl();
    ros::spinOnce();
    ros::WallDuration(0.2).sleep();
    ros::spinOnce();
    ros::WallDuration(0.2).sleep();
    enable_pose_ctrl();
    ros::spinOnce();
}

servo_driver::~servo_driver()
{
}

void servo_driver::init_pose_ctrl(void)
{ // 0 : 01 id
    common::can can_tx_buf;
    can_tx_buf.id = NMT;
    can_tx_buf.len = 2;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = 0x01;
    can_tx_buf.datas[1] = canopen_id_;
    can_tx_pub.publish(can_tx_buf);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
    // 300 : 01
    can_tx_buf.id = MODE_FC(canopen_id_);
    can_tx_buf.len = 1;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = 0x01;
    can_tx_pub.publish(can_tx_buf);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
}

void servo_driver::init_speed_ctrl(void)
{ // 0 : 01 id
    common::can can_tx_buf;
    can_tx_buf.id = NMT;
    can_tx_buf.len = 2;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = 0x01;
    can_tx_buf.datas[1] = canopen_id_;
    can_tx_pub.publish(can_tx_buf);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
    // 300 : 03
    can_tx_buf.id = MODE_FC(canopen_id_);
    can_tx_buf.len = 1;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = 0x03;
    can_tx_pub.publish(can_tx_buf);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
}

void servo_driver::set_manual_mode(int manual_mode)
{
    manual_mode_ = manual_mode;
}
void servo_driver::enable_speed_ctrl(void)
{
    // 200 : 06 00
    common::can can_tx_buf;
    can_tx_buf.id = ENABLE_FC(canopen_id_);
    can_tx_buf.len = 2;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = 0x06;
    can_tx_buf.datas[1] = 0;
    can_tx_pub.publish(can_tx_buf);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();

    // 200 : 07 00
    can_tx_buf.id = ENABLE_FC(canopen_id_);
    can_tx_buf.len = 2;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = 0x07;
    can_tx_buf.datas[1] = 0;
    can_tx_pub.publish(can_tx_buf);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();

    // 200 : 0f 00
    can_tx_buf.id = ENABLE_FC(canopen_id_);
    can_tx_buf.len = 2;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = 0x0f;
    can_tx_buf.datas[1] = 0;
    can_tx_pub.publish(can_tx_buf);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
}
void servo_driver::enable_pose_ctrl(void)
{
    // 200 : 06 00
    common::can can_tx_buf;
    can_tx_buf.id = ENABLE_FC(canopen_id_);
    can_tx_buf.len = 2;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = 0x06;
    can_tx_buf.datas[1] = 0;
    can_tx_pub.publish(can_tx_buf);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();

    // 200 : 07 00
    can_tx_buf.id = ENABLE_FC(canopen_id_);
    can_tx_buf.len = 2;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = 0x07;
    can_tx_buf.datas[1] = 0;
    can_tx_pub.publish(can_tx_buf);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();

    // 200 : 0f 00
    can_tx_buf.id = ENABLE_FC(canopen_id_);
    can_tx_buf.len = 2;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = 0x0f;
    can_tx_buf.datas[1] = 0;
    can_tx_pub.publish(can_tx_buf);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
}

void servo_driver::disable_pose_ctrl(void)
{
    // 200 : 05 00
    common::can can_tx_buf;
    can_tx_buf.id = ENABLE_FC(canopen_id_);
    can_tx_buf.len = 2;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = 0x05;
    can_tx_buf.datas[1] = 0x0;
    can_tx_pub.publish(can_tx_buf);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();

    // 300 : 01
    can_tx_buf.id = MODE_FC(canopen_id_);
    can_tx_buf.len = 1;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = 0x01;
    can_tx_pub.publish(can_tx_buf);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
}
void servo_driver::set_zero_point(double servo_zero_point)
{
    servo_zero_point_ = k_red_ratio_ * servo_zero_point;
    if (oppsite_)
        servo_zero_point_ = -servo_zero_point_;
}
// theta :绝对角度，单位度
// vel：运行速度，无符号，单位转每分钟
void servo_driver::send_pose_cmd(double theta, int vel)
{
    // 500 : theta + turns + vel
    theta += servo_zero_point_;
    int theta_int = 0x10000 * theta / 360.0;
    common::can can_tx_buf;
    can_tx_buf.id = POSITION_CMD_FC(canopen_id_);
    can_tx_buf.len = 8;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = theta_int;
    can_tx_buf.datas[1] = theta_int >> 8;
    can_tx_buf.datas[2] = theta_int >> 16;
    can_tx_buf.datas[3] = theta_int >> 24;
    can_tx_buf.datas[4] = vel;
    can_tx_buf.datas[5] = vel >> 8;
    can_tx_buf.datas[6] = vel >> 16;
    can_tx_buf.datas[7] = vel >> 24;
    can_tx_pub.publish(can_tx_buf);
    ros::spinOnce();

    // 200 : 3f 00
    can_tx_buf.id = ENABLE_FC(canopen_id_);
    can_tx_buf.len = 2;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = 0x3f;
    can_tx_buf.datas[1] = 0x0;
    can_tx_pub.publish(can_tx_buf);
    ros::spinOnce();
}

void servo_driver::disable_speed_ctrl(void)
{
    // 200 : 05 00
    common::can can_tx_buf;
    can_tx_buf.id = ENABLE_FC(canopen_id_);
    can_tx_buf.len = 2;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = 0x05;
    can_tx_buf.datas[1] = 0x0;
    can_tx_pub.publish(can_tx_buf);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();

    // 300 : 03
    can_tx_buf.id = MODE_FC(canopen_id_);
    can_tx_buf.len = 1;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = 0x03;
    can_tx_pub.publish(can_tx_buf);
    ros::spinOnce();
    ros::WallDuration(0.1).sleep();
}
void servo_driver::send_speed_cmd(int vel)
{
    // 500 : theta + turns + vel
    common::can can_tx_buf;
    can_tx_buf.id = SPEED_CMD_FC(canopen_id_);
    can_tx_buf.len = 6;
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = vel;
    can_tx_buf.datas[1] = vel >> 8;
    can_tx_buf.datas[2] = vel >> 16;
    can_tx_buf.datas[3] = vel >> 24;
    can_tx_buf.datas[4] = 0;
    can_tx_buf.datas[5] = 0;
    can_tx_pub.publish(can_tx_buf);
}

int servo_driver::controller_state_report(const common::can::ConstPtr &msg, common::servo_encoder &se, std::string &error_msg_s)
{
    if ((msg->id & 0x7f) == canopen_id_)
    {
    if (msg->id == STATUS_FC(canopen_id_))
    {
    }
    else if (msg->id == STATUS_FC(canopen_id_))
    {
    }
    else if (msg->id == SPEED_FEEDBACK_FC(canopen_id_))
    {
    }
    else if (msg->id == POSITION_FEEDBACK_FC(canopen_id_))
    {
        double tmp_theta = 0;
        int theta_int;
        if (msg->len != 8)
        {
            std::cout << "pose feedback error" << std::endl;
            return -1;
        }
        theta_int = msg->datas[4] |
                    ((int)(msg->datas[5]) << 8) |
                    ((int)(msg->datas[6]) << 16) |
                    ((int)(msg->datas[7]) << 24);
        tmp_theta = (double)theta_int / 0x10000 * 360 - servo_zero_point_;
        if (oppsite_)
            tmp_theta = -tmp_theta;
        if(postfix_=="left")
        {
            se.se_left_theta = tmp_theta;
            se.se_left_theta = -se.se_left_theta / k_red_ratio_;
            // encoder_vel_ = se.se_vel;
            encoder_theta_ = se.se_left_theta;
        }
        else if(postfix_=="right")
        {
            se.se_right_theta = tmp_theta;
            se.se_right_theta = -se.se_right_theta / k_red_ratio_;
            // encoder_vel_ = se.se_vel;
            encoder_theta_ = se.se_right_theta;
        }    
        last_recv_time_ = ros::Time::now().toSec();
    }
    }
}
bool servo_driver::judge_angle_near(const common::servo_cmd::ConstPtr &msg, common::servo_encoder se)
{ 
    double thetax;
    double thetay;
    std::cout<<"postfix_"<<postfix_<<std::endl;
        if(postfix_=="left")
        {
          thetax=msg->sc_left_theta;
            thetay=se.se_left_theta;
        }
        else if(postfix_=="right")
        {
            thetax=msg->sc_right_theta;
            thetay=se.se_right_theta;
        }
        else
        {
            std::cout<<"judge_angle_near postfix_ is error"<<std::endl;
            return false;
        }
    double err = thetax - thetay;
    while (err < -180)
        err += 360;
    while (err > 180)
        err -= 360;
    //  std::cout << "err:" << thetax << " " << thetay << " " << err << std::endl;
    if (fabs(err) < threshold_)
        return true;
    else
        return false;
}
void servo_driver::set_servo_cmd(const common::servo_cmd::ConstPtr &msg, bool syn_enable)
{
        double theta=0;
        double se_theta = encoder_theta_;
         if(postfix_=="left")
        {
            theta= msg->sc_left_theta;
            // se_theta = se.se_left_theta;
        }
        else if(postfix_=="right")
        {
            theta = msg->sc_right_theta;
            // se_theta = se.se_right_theta;
        }
        else {
            std::cout<<"postfix_ error"<<std::endl;
            return;
        }
        std::cout<<"theta"<<theta<<std::endl;

        std::cout<<"se_theta"<<se_theta<<std::endl;
        // double err = theta - se_theta;
        // if(theta > 180) theta = theta - 360;
        // if(theta < 180)theta = theta + 360;
        // if(fabs(theta - se_theta) > 90){
        //     if(theta > 0) theta = theta -180;
        //     if(theta < 0)theta = theta +180;
        // }

        last_send_time_ = ros::Time::now().toSec();
        servo_cmd_cache.sc_theta = -k_red_ratio_ * theta;
        std::cout<<"k_red_ratio_"<<k_red_ratio_<<std::endl;
        if (oppsite_)
            servo_cmd_cache.sc_theta = -servo_cmd_cache.sc_theta;
        std::cout<<"debug set servo cmd"<<servo_cmd_cache.sc_theta<<std::endl;
}

int servo_driver::timer_can_send(void)
{
    if (inited)
    {
        double cur_time = ros::Time::now().toSec();
        //  std::cout.precision(16);
        //     std::cout <<"cur_time:"<<cur_time<<"   last_recv_time_:"<< last_recv_time_ <<"  "<<cur_time - last_recv_time_ <<std::endl;

        if (cur_time - last_recv_time_ > 0.2)
        {
            std::cout << postfix_ << " servo encoder too old" << std::endl;
            return -1;
        }
        if (cur_time - last_send_time_ > 0.2)
        {
            servo_cmd_cache.sc_theta = 0;
            // std::cout << postfix_ << " servo cmd too old" << std::endl;
            // return -2;
        }

        send_pose_cmd(servo_cmd_cache.sc_theta, 9.16 * 2000);
        // send_speed_cmd(servo_cmd_cache.sc_vel);
        return 0;
    }
    else
    {
        init_pose_ctrl();
        inited = true;
    }
}

void servo_driver::init(void)
{
    return;
}
void servo_driver::send_servo_cmd(common::servo_cmd &se)
{
    return;
}
int servo_driver::controller_error_report(const common::can::ConstPtr &msg, std::string &error_msg_s)
{
    return 0;
}
void servo_driver::disable_send(void)
{
    return;
}
void servo_driver::enable_send(void)
{
    return;
}
bool servo_driver::need_reset(void)
{
    return true;
}