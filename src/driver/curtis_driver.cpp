#include <curtis_driver.h>
curtis_driver::curtis_driver(ros::NodeHandle &nh,
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
    // servo_encoder_pub = nh_.advertise<common::servo_encoder>("/servo_encoder", 1000); //
    can_tx_pub = nh_.advertise<common::can>(can_tx_topic, 1000);
    curtis_absent_pub = nh_.advertise<std_msgs::Int8>("/curtis_absent", 1000);

    last_send_time_ = ros::Time::now().toSec();
    last_recv_time_ = ros::Time::now().toSec();
    status = AGV_Status::INVALID;
    servo_cmd_cache.sc_theta = 0;
    servo_cmd_cache.sc_vel = 0;
    car_rpm_xs = car_rpm_max / 255.0;

    error_msg.insert(make_pair(51, "error 51:CAN Comm Fault"));
    error_msg.insert(make_pair(52, "error 52:Severe Steering Fault"));
    error_msg.insert(make_pair(53, "error 53:Steering Fault"));
    error_msg.insert(make_pair(56, "error 56:Extra SRO Fault"));
    error_msg.insert(make_pair(57, "error 57:Throttle PotHigh"));
    error_msg.insert(make_pair(59, "error 59:CAN 1222 PDO Fault"));
    error_msg.insert(make_pair(61, "error 61:PLC PDO Fault"));
    error_msg.insert(make_pair(62, "error 62:PLC Heartbea t Fault"));
    error_msg.insert(make_pair(63, "error 63:CAN Pump PDO Fault"));
    error_msg.insert(make_pair(64, "error 64:CAN 3210T PDO Fault"));
    error_msg.insert(make_pair(65, "error 65:1352eXm Parameter Fault"));
    error_msg.insert(make_pair(66, "error 66:AGV Traction SRO Fault"));
    error_msg.insert(make_pair(67, "error 67:Unmatched Pump Controller"));
    enable_send();
}

curtis_driver::~curtis_driver()
{
}

void curtis_driver::set_zero_point(double servo_zero_point)
{
    servo_zero_point_ = servo_zero_point;
}

void curtis_driver::set_manual_mode(int manual_mode)
{
    manual_mode_ = manual_mode;
}

// theta :绝对角度，单位度
// vel：运行速度，无符号，单位转每分钟
void curtis_driver::send_servo_cmd(common::servo_cmd &sc)
{
    double theta;

    if (theta_oppsite_)
        theta = -sc.sc_theta + servo_zero_point_;
    else
        theta = sc.sc_theta + servo_zero_point_;

    theta = (theta > 100) ? 100 : theta;
    theta = (theta < -100) ? -100 : theta;

    int16_t sc_angle_int16;
    int16_t sc_speed_int16;

    sc_angle_int16 = (int16_t)(theta * 10);
    sc_speed_int16 = (int16_t)sc.sc_vel;
    std::cout<<"debug sc.sc_vel"<<sc.sc_vel<<std::endl;
    if (vel_oppsite_)
        sc_speed_int16 = -sc_speed_int16;

    common::can can_tx_buf;

    can_tx_buf.id = 0x3f0; // CAN_ID
    can_tx_buf.len = 8;    // CAN数据长度
    can_tx_buf.datas.resize(can_tx_buf.len);
    can_tx_buf.datas[0] = 0; // 清零车辆控制位
    if ((status == AGV_Status::AGV) && (init_cnt_ == 0))
    {
        if (sc_speed_int16 > 0)
        {
            can_tx_buf.datas[0] = car_agv_enable_flag | car_move_enable_flag | car_forward_flag;
            can_tx_buf.datas[1] = (uint8_t)(sc_speed_int16 / car_rpm_xs);
            std::cout<<"debug 8963"<<can_tx_buf.datas[1]<<std::endl;
        }
        else if (sc_speed_int16 < 0)
        {
            can_tx_buf.datas[0] = car_agv_enable_flag | car_move_enable_flag | car_back_flag;
            can_tx_buf.datas[1] = (uint8_t)(0 - (sc_speed_int16) / car_rpm_xs);
            std::cout<<"debug 6335"<<can_tx_buf.datas[1]<<std::endl;
        }
        else
        {
            can_tx_buf.datas[0] = car_agv_enable_flag | car_move_enable_flag | 0x00;
            can_tx_buf.datas[1] = (uint8_t)(0);
            std::cout<<"debug 99666"<<can_tx_buf.datas[1]<<std::endl;
        }
        can_tx_buf.datas[3] = sc_angle_int16 & 0x00ff;
        can_tx_buf.datas[2] = (sc_angle_int16 >> 8) & 0x00ff;
    }
    else
    {
        if (init_cnt_ > 0)
            init_cnt_--;
        can_tx_buf.datas[0] = car_agv_enable_flag | car_move_enable_flag;
        can_tx_buf.datas[1] = 0;
        can_tx_buf.datas[2] = 0;
        can_tx_buf.datas[3] = 0;
        can_tx_buf.datas[4] = 0;
        can_tx_buf.datas[5] = 0;
        can_tx_buf.datas[6] = 0;
    }

    can_tx_buf.datas[7] = heartbeat++;
    std::cout<<"debug 8454"<<std::endl;
    can_tx_pub.publish(can_tx_buf);
}

bool curtis_driver::need_reset(void)
{
    static int num_high_speed = 0;
    static int num_low_speed = 0;
    static int num_theta_reset = 0;

    // std::cout << "servo_cmd_cache.sc_vel:" << servo_cmd_cache.sc_vel << std::endl;
    // std::cout << "encoder_vel_:" << encoder_vel_ << std::endl;

    // std::cout << "init_cnt_:" << init_cnt_ << std::endl;
    // std::cout << "send_enable_:" << send_enable_ << std::endl;
    // std::cout << "num_high_speed:" << num_high_speed << std::endl;
    // std::cout << "num_low_speed:" << num_low_speed << std::endl;
    // std::cout << "num_theta_reset:" << num_theta_reset << std::endl;
    if (num_low_speed > 0)
    {
        acc_k_ = 1.0 + 0.02 * num_low_speed;
    }
    else if (num_high_speed > 0)
    {
        acc_k_ = 1.0 + 0.01 * num_high_speed;
    }
    else
        acc_k_ = 1.0;
    if ((fabs(servo_cmd_cache.sc_vel) > 80) && (fabs(encoder_vel_) < 5) && (init_cnt_ == 0) && send_enable_)
        num_high_speed++;
    else
        num_high_speed = 0;

    if ((fabs(servo_cmd_cache.sc_vel) > 30) && (fabs(encoder_vel_) < 5) && (init_cnt_ == 0) && send_enable_)
        num_low_speed++;
    else
        num_low_speed = 0;
    if (curtis_steer_enable_)
    {
        if ((fabs(servo_cmd_cache.sc_theta - encoder_theta_) > 20) && (init_cnt_ == 0) && send_enable_)
        {
            std::cout << "servo_cmd_cache.sc_theta" << servo_cmd_cache.sc_theta << std::endl;
            std::cout << "encoder_theta_" << encoder_theta_ << std::endl;
            std::cout << "fabs(servo_cmd_cache.sc_theta - encoder_theta_)" << fabs(servo_cmd_cache.sc_theta - encoder_theta_) << std::endl;
            num_theta_reset++;
        }
        else
            num_theta_reset = 0;
    }
    else
    {
        num_theta_reset = 0;
    }
    if ((num_high_speed > 80) || (num_low_speed > 220) || (num_theta_reset > 220))
    {
        num_high_speed = 0;
        num_low_speed = 0;
        num_theta_reset = 0;
        std::cout << "------------------------------------------------------------------------" << std::endl;
        return true;
    }
    else
        return false;
}

void curtis_driver::init(void)
{
    init_cnt_ = 60;
}

void curtis_driver::disable_send(void)
{
    send_enable_ = false;
}

void curtis_driver::enable_send(void)
{
    send_enable_ = true;
}

int curtis_driver::controller_state_report(const common::can::ConstPtr &msg, common::servo_encoder &se, std::string &error_msg_s)
{
    int16_t ang_l, speed_l;
    if(msg->id == 0x4f6)
    {
        if (msg->len != 8)
        {
            std::cout << "recv_4f6 pose feedback error" << std::endl;
             return -1;
        }
    speed_l = (int16_t)(((int16_t)(msg->datas[3]) << 8 & 0xff00) | (msg->datas[4] & 0x00ff));
    ang_l = (int16_t)(((int16_t)(msg->datas[1]) << 8 & 0xff00) | (msg->datas[2] & 0x00ff));
    status = (AGV_Status)msg->datas[6];
    if (status == AGV_Status::CAN_ERROR)
        return -2;
    se.se_vel = speed_l;

    if (vel_oppsite_)
        se.se_vel = -se.se_vel;
    if (theta_oppsite_)
        se.se_theta = -(ang_l / 10.0f - servo_zero_point_);
    else
        se.se_theta = ang_l / 10.0f - servo_zero_point_;
    se.header.stamp = msg->header.stamp;
    last_recv_time_ = ros::Time::now().toSec();
    // ROS_INFO("vel = %f,theta = %f,mode = %d,flag=%d,xs=%f,max=%d",car_s.se_vel,car_s.se_theta,msg->datas[6],speed_downdir_flag,car_rpm_xs,car_rpm_max);
    if (curtis_steer_enable_)
    {
        // servo_encoder_pub.publish(se);
    }
    encoder_vel_ = se.se_vel;
    encoder_theta_ = se.se_theta;
    return 0;

    }
}


    


int curtis_driver::controller_error_report(const common::can::ConstPtr &msg, std::string &error_msg_s)
{
    if(msg->id == 0x4f5)
    {
    if (msg->len < 2)
    {
        std::cout << "recv_4f5 pose feedback error" << std::endl;
        error_msg_s = "4f5 len != 8";
        return -1;
    }
    if (msg->datas[1] != 0)
    {
        std::map<int, std::string>::iterator it = error_msg.find(msg->datas[1]);
        if (it != error_msg.end())
        {
            error_msg_s = error_msg[msg->datas[1]];
            return msg->datas[1];
        }
    }
    return 0;
    }
}
bool curtis_driver::judge_angle_near(const common::servo_cmd::ConstPtr &msg, common::servo_encoder se)
{
    double thetax=msg->sc_theta;
    double thetay=se.se_theta;
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
void curtis_driver::set_servo_cmd(const common::servo_cmd::ConstPtr &msg,bool syn_enable)
{

    syn_enable_ = syn_enable;
    last_send_time_ = ros::Time::now().toSec();
    servo_cmd_cache.sc_theta = msg->sc_theta;
    servo_cmd_cache.sc_vel = msg->sc_vel;
    std::cout<<"debug curtis_driver set_servo_cmd"<<servo_cmd_cache.sc_theta<<std::endl;
    std::cout<<"debug curtis_driver sc_vel"<<servo_cmd_cache.sc_vel<<std::endl;
}

int curtis_driver::timer_can_send(void)
{
    static bool encoder_absent = false;
    bool absent;
    if (inited)
    {
        double cur_time = ros::Time::now().toSec();
        if (cur_time - last_recv_time_ > 0.5)
        {
            encoder_vel_ = 0;
            std::cout << "main servo encoder too old" << std::endl;
            absent = true;
        }
        else
            absent = false;

        if (absent != encoder_absent)
        {
            std_msgs::Int8 msg;
            msg.data = absent ? 1 : 0;
            curtis_absent_pub.publish(msg);
            encoder_absent = absent;
            // if (absent)
            //     return -1;
        }

        if (cur_time - last_send_time_ > 0.5)
        {
            // std::cout << "main servo cmd too old" << std::endl;
            servo_cmd_cache.sc_theta = 0;
            servo_cmd_cache.sc_vel = 0;
            // return -2;
        }
        std::cout<<"send_enable_"<<send_enable_<<std::endl;
        if (send_enable_)
        {
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
                send_servo_cmd(servo_cmd_);
            }
        }
        return 0;
    }
    else
    {
        inited = true;
    }
}
