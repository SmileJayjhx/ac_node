#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Int16MultiArray.h"
#include "common/can.h"
#include "common/servo_cmd.h"
#include "common/servo_encoder.h"
#include "common/peripheral_uart.h"
#include <dynamic_reconfigure/server.h>
#include <ac_node/ServoConfig.h>
#include <fstream>
#include <string>

#include "common/motion_state.h"

//can通讯协议
#define car_forward_flag 1
#define car_back_flag 2
#define car_move_enable_flag 4
#define car_agv_enable_flag 8

//车体can通信相关参数

uint8_t can_cont = 0;      //ac使能命令计数
uint8_t cmd_cont = 100;    //命令有效时间计数（50ms）
uint8_t cmd_clearflag = 0; //cmd命令持续有效标志位
uint8_t heartbeat = 0;     //心跳位
uint8_t heartbeat1 = 0;
common::can can_s;         //can传递数据
common::can can_s2;         //can传递数据
common::can can_s1;
int16_t sc_angle;          //获取命令中的速度
int16_t sc_speed;          //获取命令中的角度

int16_t agv_speed;
double agv_theta;
double servo_zero_point = 0;
//定义车体can跟控制端发布消息
ros::Publisher car_updata_pub;
ros::Publisher can_updata_pub;

ros::Publisher mainControl2turn_pub;

ros::Publisher  robot_state_pub;
int motion = common::motion_state::STOP;

//定义方向标志位
bool speed_updir_flag;
bool speed_downdir_flag;
bool angle_updir_flag;
bool angle_downdir_flag;
bool can_send_flag;
int car_rpm_max;
int car_type = 1;
float car_rpm_xs;
//测试参数
int16_t speed = -10;
int16_t angle = -16;

std::string config_file_path;
std::string config_file = "servo_zero_point.yaml";
bool zero_point_calibration_enable = false;

int is_manual_flag = 0;

unsigned int state = common::motion_state::STOP;

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
    std::cout << servo_zero_point << std::endl;
    if (config.save_data && !save_data)
    {
        save_zero_point(servo_zero_point);
    }
    save_data = config.save_data;
}

void robot_state_judge(const common::servo_cmd::ConstPtr &msg)
{
    static common::motion_state state;
    state.header.stamp = ros::Time::now();
    if (msg->sc_vel > 0  && ( msg->sc_theta < 10.00 && msg->sc_theta > -10.00))   
    {    
        state.motion_state = common::motion_state::FORWARD;
        // std::cout<<"机器人 直行"<<std::endl;
    }
    else if (msg->sc_vel > 0  && msg->sc_theta > 10.00)     
    {   
        state.motion_state = common::motion_state::FORWARD_RIGHT;
        // std::cout<<"机器人 前右转弯"<<std::endl;
    } 
    else if (msg->sc_vel > 0  && msg->sc_theta < -10.00)     
    {  
        state.motion_state = common::motion_state::FORWARD_LEFT;
        // std::cout<<"机器人 前左转弯"<<std::endl;
    }
    else if (msg->sc_vel < 0   && ( msg->sc_theta < 10.00 && msg->sc_theta > -10.00))     
    {  
        state.motion_state = common::motion_state::BACK;
        // std::cout<<"机器人 后退"<<std::endl;
    }
    else if (msg->sc_vel < 0 && msg->sc_theta > 10.00)       //
    {      
        state.motion_state = common::motion_state::BACK_RIGHT;
        // std::cout<<"机器人 右后转弯"<<std::endl;
    }
    else if (msg->sc_vel < 0  &&  msg->sc_theta <-10.00)        //
    {  
        state.motion_state = common::motion_state::BACK_LEFT;
        // std::cout<<"机器人 左后转弯"<<std::endl;
    }  
    else if ( msg->sc_vel  == 0)  //
    {
        state.motion_state = common::motion_state::STOP;
        // std::cout<<"机器人 静止"<<std::endl;
    }
    robot_state_pub.publish(state);
}

/*根据计算的RPM，舵轮转角打包成车体can数据下发*/
void car_downloadCallback(const common::servo_cmd::ConstPtr &msg)
{
    //add 2022/6/20
    // robot_state_judge(msg);

    static double theta_out=0;
    int16_t theta = msg->sc_theta + servo_zero_point;
    int16_t sc_angle_lin;
    int16_t sc_speed_lin;


    std::cout << "car_type: " << car_type << std::endl; 

    if (car_type == 2)
    {
        if (theta > 90)
            theta = 90;
        else if (theta < -90)
            theta = -90;
        sc_angle_lin = (int16_t)(theta * 100);
        sc_speed_lin = (int16_t)msg->sc_vel;
    }
    else if (car_type == 1 || car_type == 3)
    {
        if (theta > 90)
            theta = 90;
        else if (theta < -90)
            theta = -90;
        sc_angle_lin = (int16_t)(theta * -10) ;
        sc_speed_lin = (int16_t)msg->sc_vel;
    }
    cmd_clearflag = 1; //清除命令延时计数
    //确定速度方向一致性
    if (speed_downdir_flag == false)
    {
        sc_speed = 0 - sc_speed_lin;
    }
    else
    {
        sc_speed = sc_speed_lin;
    }
    //确定角度方向一致性
    if (angle_downdir_flag == false)
    {
        sc_angle = 0 - sc_angle_lin;
    }
    else
    {
        sc_angle = sc_angle_lin;
    }
    if (can_send_flag == false)
    {
        can_s.id = 0x3f0; //CAN_ID
        can_s.len = 8;    //CAN数据长度
        can_s.datas.resize(can_s.len);
        can_s.datas[0] = 0; //清零车辆控制位

        //自动模式有效且cmd延时计数小于10，发送有效速度，否则发送自动模式使能指令
        if ((can_cont > 25))
        {
            //根据速度正负判断前进或者后退
            if (sc_speed > 0)
            {
                can_s.datas[0] = car_agv_enable_flag | car_move_enable_flag | car_forward_flag;
                can_s.datas[1] = (uint8_t)(sc_speed / car_rpm_xs);
            }
            else if (sc_speed < 0)
            {
                can_s.datas[0] = car_agv_enable_flag | car_move_enable_flag | car_back_flag;
                can_s.datas[1] = (uint8_t)(0 - (sc_speed) / car_rpm_xs);
            }
            else
            {
                can_s.datas[0] = car_agv_enable_flag | car_move_enable_flag | 0x00;
                can_s.datas[1] = (uint8_t)(0);
            }

            can_s.datas[3] = sc_angle & 0x00ff;
            can_s.datas[2] = (sc_angle >> 8) & 0x00ff;

        }
        else
        {
            can_s.datas[0] = car_agv_enable_flag | car_move_enable_flag;
            can_s.datas[1] = 0;
            can_s.datas[2] = 0;
            can_s.datas[3] = 0;
            can_s.datas[4] = 0;
            can_s.datas[5] = 0;
            can_s.datas[6] = 0;
        }
        //心跳位累加
        can_s.datas[7] = heartbeat++;
        if (heartbeat > 255)
        {
            heartbeat = 0;
        }

        //发布can_tx消息
        can_updata_pub.publish(can_s);
        //    ROS_INFO("en = %d,rpm = %d, ac=%d,cmd=%d",can_s.datas[0],can_s.datas[1],can_cont,cmd_cont);
    }
    if(car_type == 3)
    {
        double theta_cmd =  msg->sc_theta + servo_zero_point;

        // std::cout << "theta_cmd " << theta_cmd << "  msg->sc_theta: " 
        // << msg->sc_theta << "   servo_zero_point: " << servo_zero_point << std::endl; 

        theta_cmd =(theta_cmd>90)?90:theta_cmd;
        theta_cmd =(theta_cmd<-90)?-90:theta_cmd;
        theta_out = theta_out+ 0.25*(theta_cmd - theta_out);
        can_s1.id = 0x270;
        can_s1.len = 8;
        can_s1.datas.resize(can_s1.len);

        //  can_s1.datas[0] = 1;
        can_s1.datas[0] = is_manual_flag;

        // can_s1.datas[1] = tmp_p[1];
        // can_s1.datas[2] = tmp_p[0];

        can_s1.datas[1] = (int16_t)(-theta_out * 100)>>8&0xff;
        can_s1.datas[2] = (int16_t)(-theta_out * 100)&0xff;

        can_s1.datas[3] = 0;
        can_s1.datas[4] = 0;
        can_s1.datas[5] = 0;
        can_s1.datas[6] = 0;
        can_s1.datas[7] = heartbeat1++;
        // can_s1.datas[7] = 0;
        mainControl2turn_pub.publish(can_s1); 
    }
}

/*收到can模块数据后分包成RPM,舵轮转角上传*/
void can_downloadCallback(const common::can::ConstPtr &msg)
{
    // ROS_INFO("%d,%d,%d,%d,%d,%d,%d,%d",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5],msg->data[6],msg->data[7]);
    common::servo_encoder car_s;
    int16_t ang_l, speed_l;
    if (car_type == 2)
    {
        if (msg->id == 0x207)
        {
            speed_l = (int16_t)((msg->datas[7] << 8 & 0xff00) | (msg->datas[6] & 0x00ff));
            ang_l = (int16_t)((msg->datas[5] << 8 & 0xff00) | (msg->datas[4] & 0x00ff));
            if (speed_updir_flag == false)
            {
                speed_l = 0 - speed_l;
            }
            if (!angle_updir_flag)
            {
                ang_l = 0 - ang_l;
            }
            car_s.se_vel = speed_l;
            car_s.se_theta = ang_l / 100.0f - servo_zero_point;
            // car_s.se_theta = agv_theta;
            car_s.header.stamp = msg->header.stamp;
            std::cout << "can_downloadCallback car_type == 2 " <<std::endl;
            car_updata_pub.publish(car_s);
        }
       
    }
    else if (car_type == 1)
    {
        if (msg->id == 0x4f6)
        {
            speed_l = (int16_t)((msg->datas[3] << 8 & 0xff00) | (msg->datas[4] & 0x00ff));
            ang_l = (int16_t)((msg->datas[1] << 8 & 0xff00) | (msg->datas[2] & 0x00ff));
            if (speed_updir_flag == false)
            {
                speed_l = 0 - speed_l;
            }
            if (!angle_updir_flag)
            {
                ang_l = 0 - ang_l;
            }
            car_s.se_vel = speed_l;

            car_s.se_theta = ang_l / 10.0f - servo_zero_point;
            if (msg->datas[6] == 0x01)
            {
                can_cont++; //ac控制器发送计数
            }
            else
            {
                can_cont = 0;
            }
            //防止自动有效计数溢出
            if (can_cont > 25)
            {
                can_cont = 100;
            }

            car_s.header.stamp = msg->header.stamp;
            // ROS_INFO("vel = %f,theta = %f,mode = %d,flag=%d,xs=%f,max=%d",car_s.se_vel,car_s.se_theta,msg->datas[6],speed_downdir_flag,car_rpm_xs,car_rpm_max);
            car_updata_pub.publish(car_s);
        }
    }
    else if(car_type == 3)
    {
        if (msg->id == 0x4f6)  //ketisi
        {
            speed_l = (int16_t)((msg->datas[3] << 8 & 0xff00) | (msg->datas[4] & 0x00ff));
            ang_l = agv_theta;// (int16_t)((msg->datas[1] << 8 & 0xff00) | (msg->datas[2] & 0x00ff));
            if (speed_updir_flag == false)
            {
                speed_l = 0 - speed_l;
            }
            if (!angle_updir_flag)
            {
                ang_l = 0 - ang_l;
            }
            car_s.se_vel = speed_l;
            std::cout << "2 ang_l: " << ang_l << "servo_zero_point " << servo_zero_point<< std::endl;
            car_s.se_theta = ang_l - servo_zero_point;
            std::cout << "3 car_s.se_theta: " << car_s.se_theta << std::endl;
            if (msg->datas[6] == 0x01)
            {
                can_cont++; //ac控制器发送计数
            }
            else
            {
                can_cont = 0;
            }
            //防止自动有效计数溢出
            if (can_cont > 25)
            {
                can_cont = 100;
            }
            std::cout << "4 car_s.se_vel   " << car_s.se_vel << "   car_s.se_theta    " << car_s.se_theta << std::endl;
            car_s.header.stamp = msg->header.stamp;
            // ROS_INFO("vel = %f,theta = %f,mode = %d,flag=%d,xs=%f,max=%d",car_s.se_vel,car_s.se_theta,msg->datas[6],speed_downdir_flag,car_rpm_xs,car_rpm_max);
            car_updata_pub.publish(car_s);
        }

    }

    if (msg->id == 0x280)
    {
        return;
        static int heartbeat_ = msg->datas[7];
        static int last_heartbeat = heartbeat_;

        int16_t theta_;
        uint8_t *tmp_p = (uint8_t *)&theta_;
        tmp_p[0]=msg->datas[1];
        tmp_p[1]=msg->datas[2];
        theta_ = theta_/100.0f;
        theta_ =(theta_>90)?90:theta_;
        theta_ =(theta_<-90)?-90:theta_;

        agv_theta = -theta_;

        static double time1 = ros::Time::now().toSec();
        static double time2 = time1;

        time1 = ros::Time::now().toSec();

        if (heartbeat_ == last_heartbeat)
        {
            if ((time2 - time1) >1)
            {
                //报错
                ROS_ERROR("CAN丢失\n");
            }
        }
        else
            time2 = time1;

        last_heartbeat  = heartbeat_;

    }
}

void can_downloadCallback1(const common::can::ConstPtr &msg)
{
    if (msg->id == 0x280)
    {
        static int heartbeat_ = msg->datas[7];
        static int last_heartbeat = heartbeat_;

        int16_t theta_;
        uint8_t *tmp_p = (uint8_t *)&theta_;
        tmp_p[0]=msg->datas[2];
        tmp_p[1]=msg->datas[1];
        theta_ = -theta_/100.0f;

      //  std::cout<<"agv_theta = "<<(int)theta_<<std::endl;

        theta_ =(theta_>90)?90:theta_;
        theta_ =(theta_<-90)?-90:theta_;

        agv_theta = theta_;
        std::cout<<"1 agv_theta = "<< agv_theta<<std::endl;
        static double time1 = ros::Time::now().toSec();
        static double time2 = time1;

        time1 = ros::Time::now().toSec();

        if (heartbeat_ == last_heartbeat)
        {
            if ((time2 - time1) >1)
            {
                //报错
                ROS_ERROR("CAN丢失\n");
                // return;
            }
        }
        else
            time2 = time1;

        last_heartbeat  = heartbeat_;
        heartbeat_++;

    }
}

//定时下发can数据（50ms）
void can_send_callback(const ros::TimerEvent &)
{
    if (car_type == 2)
    {
        can_s2.id = 0x313;
        can_s2.len = 8;    //CAN数据长度
        can_s2.datas.resize(can_s2.len);
        can_s2.header.stamp = ros::Time::now();
        can_s2.datas[0] = 0;
        can_s2.datas[1] = 0;
        can_s2.datas[2] = 0;
        can_s2.datas[3] = 0;
        can_s2.datas[4] = 0;
        can_s2.datas[5] = 0;
        can_s2.datas[6] = 0;
        can_s2.datas[7] = 0;

        can_s.id = 0x213; //CAN_ID
        can_s.len = 8;    //CAN数据长度
        can_s.datas.resize(can_s.len);
        can_s.header.stamp = ros::Time::now();
        cmd_cont++; //cmd发送延迟计数
        //cmd延时计数防止溢出
        if (cmd_cont > 10)
        {
            cmd_cont = 100;
        }
        if (cmd_clearflag == 1)
        {
            cmd_cont = 0;
            cmd_clearflag = 0;
        }
        if (cmd_cont < 10)
        {
            if (sc_speed > 0)
            {
                can_s.datas[0] = 0x85;
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
                can_s.datas[4] = 0;
                can_s.datas[5] = 0;
                can_s.datas[6] = 0;
                can_s.datas[7] = 0;
            }
        }
        else
        {
            can_s.datas[0] = 0x83;
            can_s.datas[1] = 0;
            can_s.datas[2] = 0;
            can_s.datas[3] = 0;
            can_s.datas[4] = 0;
            can_s.datas[5] = 0;
            can_s.datas[6] = 0;
            can_s.datas[7] = 0;
        }
    }
    else if (car_type == 1 || car_type == 3)
    {
        //如意车型通信协议
        can_s.id = 0x3f0; //CAN_ID
        can_s.len = 8;    //CAN数据长度
        can_s.datas.resize(can_s.len);
        can_s.datas[0] = 0; //清零车辆控制位

        can_s.header.stamp = ros::Time::now();
        cmd_cont++; //cmd发送延迟计数
        //cmd延时计数防止溢出
        if (cmd_cont > 10)
        {
            cmd_cont = 100;
        }
        if (cmd_clearflag == 1)
        {
            cmd_cont = 0;
            cmd_clearflag = 0;
        }
        //自动模式有效且cmd延时计数小于10，发送有效速度，否则发送自动模式使能指令
        if ((can_cont > 25) && (cmd_cont < 10))
        {
            //根据速度正负判断前进或者后退
            if (sc_speed > 0)
            {
                can_s.datas[0] = car_agv_enable_flag | car_move_enable_flag | car_forward_flag;
                can_s.datas[1] = (uint8_t)(sc_speed / car_rpm_xs);
            }
            else if (sc_speed < 0)
            {
                can_s.datas[0] = car_agv_enable_flag | car_move_enable_flag | car_back_flag;
                can_s.datas[1] = (uint8_t)(0 - (sc_speed) / car_rpm_xs);
            }
            else
            {
                can_s.datas[0] = car_agv_enable_flag | car_move_enable_flag | 0x00;
                can_s.datas[1] = (uint8_t)(0);
            }
            can_s.datas[3] = sc_angle & 0x00ff;
            can_s.datas[2] = (sc_angle >> 8) & 0x00ff;
        }
        else
        {
            can_s.datas[0] = car_agv_enable_flag | car_move_enable_flag;
            can_s.datas[1] = 0;
            can_s.datas[2] = 0;
            can_s.datas[3] = 0;
            can_s.datas[4] = 0;
            can_s.datas[5] = 0;
            can_s.datas[6] = 0;
        }
        //心跳位累加
        can_s.datas[7] = heartbeat++;
    }
    //发布can_tx消息
    can_updata_pub.publish(can_s);
    can_updata_pub.publish(can_s2);
    if((car_type == 3)&&(is_manual_flag==0))
    {
        can_s1.id = 0x270;
        can_s1.len = 8;
        can_s1.datas.resize(can_s1.len);

        can_s1.datas[0] = is_manual_flag;
        can_s1.datas[1] = 0;
        can_s1.datas[2] = 0;
        can_s1.datas[3] = 0;
        can_s1.datas[4] = 0;
        can_s1.datas[5] = 0;
        can_s1.datas[6] = 0;
        can_s1.datas[7] = heartbeat1++;
        mainControl2turn_pub.publish(can_s1); 
    }
}

void get_maual_auto(const common::peripheral_uart::ConstPtr &msg)
{
    /*
        接口板数据： 1-手动     0-自动
        AGV协议数据:    1-自动  0-手动
    */
    if (msg->m_iManualModeFlag == 0){
        is_manual_flag = 1;
    }else if (msg->m_iManualModeFlag == 1){
        is_manual_flag = 0;
    }
    // is_manual_flag = (msg->m_iManualModeFlag == 0)?1:msg->m_iManualModeFlag;
    // ROS_INFO("is_manual = %d", is_manual_flag);
}

int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc, argv, "ac_node");
    ros::NodeHandle m;
    ros::NodeHandle nh_priv("~");
    ros::Timer timer1;
    agv_theta = 0.0;
    //加载参数

    nh_priv.getParam("angle_updir_flag", angle_updir_flag);
    nh_priv.getParam("angle_downdir_flag", angle_downdir_flag);
    nh_priv.getParam("speed_updir_flag", speed_updir_flag);
    nh_priv.getParam("speed_downdir_flag", speed_downdir_flag);

    // nh_priv.param<bool>("angle_downdir_flag", angle_downdir_flag, false);
    // nh_priv.param<bool>("speed_updir_flag", speed_updir_flag, false);
    // nh_priv.param<bool>("speed_downdir_flag", speed_downdir_flag, false);
    // nh_priv.param<uint8_t>("angle_dirup_flag", angle_updir_flag, false);

    nh_priv.getParam("car_rpm_max", car_rpm_max);
    nh_priv.getParam("can_send_flag", can_send_flag);
    nh_priv.getParam("car_type", car_type);
    nh_priv.getParam("servo_zero_point", servo_zero_point);
    nh_priv.getParam("config_file_path", config_file_path);
    nh_priv.getParam("zero_point_calibration_enable", zero_point_calibration_enable);
    dynamic_reconfigure::Server<ac_node::ServoConfig> srv;
    if (zero_point_calibration_enable)
    {
        srv.setCallback(boost::bind(&reconfig, _1, _2));
    }
    //    nh_priv.param<int>("car_rpm_max", car_rpm_max, 2300);
    //计算RPM映射单位
    car_rpm_xs = car_rpm_max / 255.0;

    car_updata_pub = m.advertise<common::servo_encoder>("/servo_encoder", 100);
    can_updata_pub = m.advertise<common::can>("/can_tx", 100);

    // mainControl2turn_pub = j.advertise<common::can>("/can_tx", 1000);
    mainControl2turn_pub = m.advertise<common::can>("/can1_tx", 100);

    ros::Subscriber car_download = m.subscribe("/servo_cmd", 100, car_downloadCallback);
    ros::Subscriber can_download = m.subscribe("/can_rx", 100, can_downloadCallback);

    ros::Subscriber can_download1 = m.subscribe("/can1_rx", 100, can_downloadCallback1);

    ros::Subscriber maun_auto_sub = m.subscribe("/peripheral_devs_state" , 100  , get_maual_auto);

    robot_state_pub = m.advertise<common::motion_state>("/motion_state" , 100);
    ROS_INFO("CAR_T=%d", car_type);
    // can_send_flag = true;
    // ROS_INFO("CAN_Send_flag = %d" , can_send_flag);
    if (can_send_flag)
    {
        timer1 = m.createTimer(ros::Duration(0.05), can_send_callback);
    }

    ros::Rate looprate(10);
    ros::spin();
    return 0;
}