#pragma once

#include "iostream"
#include "common/can.h"
#include "common/servo_cmd.h"
#include "common/servo_encoder.h"
#include "ros/ros.h"
#include "std_msgs/Int8.h"


enum class AGV_Status
{
    INVALID,
    AGV,
    MANUAL,
    CAN_ERROR
};

class ac_driver
{
public:
    virtual void init(void) = 0;
    virtual void send_servo_cmd(common::servo_cmd &se) = 0;
    virtual void set_servo_cmd(const common::servo_cmd::ConstPtr &msg,bool syn_enable) = 0;
    virtual void set_zero_point(double servo_zero_point) = 0;
    virtual int controller_error_report(const common::can::ConstPtr &msg, std::string &error_msg_s) = 0;
    virtual int controller_state_report(const common::can::ConstPtr &msg, common::servo_encoder &se, std::string &error_msg_s) = 0;
    virtual int timer_can_send(void) = 0;
    virtual void disable_send(void) = 0;
    virtual void enable_send(void) = 0;
    virtual bool need_reset(void) = 0;
    virtual void set_manual_mode(int manual_mode)=0;
    virtual bool judge_angle_near(const common::servo_cmd::ConstPtr &msg, common::servo_encoder se) = 0;
};