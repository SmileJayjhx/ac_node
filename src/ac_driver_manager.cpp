#include "ac_driver_manager.h"

ACDriverManager::ACDriverManager() : Curtis_Steer_Enable(false), left_canopen_id(0), right_canopen_id(0),
                                     config_file_path(""), config_file("servo_zero_point.yaml"), steering_theta(0.0),
                                     k_passive_red_ratio(0.0), is_manual_mode(-1), fork_need_reset(false) 
{
    servo_encoder_msg.se_vel = 0;
    servo_encoder_msg.se_theta = 0;
}

void ACDriverManager::run() {
  ros::spin();
}

int main(int argc, char **argv) {
  ROS_ERROR("getting into main");
  ros::init(argc, argv, "ac_driver_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  ACDriverManager manager;
  manager.init(nh, nh_priv);
  manager.run();
  return 0;
}

void ACDriverManager::init(ros::NodeHandle& nh, ros::NodeHandle& nh_priv) {
  // param init
  nh_priv.getParam("main_servo_zero_point", servo_zero_point);
  nh_priv.getParam("left_servo_zero_point", left_servo_zero_point);
  nh_priv.getParam("right_servo_zero_point", right_servo_zero_point);
  nh_priv.getParam("servo_can_tx_topic", servo_can_tx_topic);
  nh_priv.getParam("curtis_can_tx_topic", curtis_can_tx_topic);
  nh_priv.getParam("Curtis_Steer_Enable", Curtis_Steer_Enable);
  nh_priv.getParam("car_rpm_max", car_rpm_max);
  nh_priv.getParam("k_passive_red_ratio", k_passive_red_ratio);
  nh_priv.getParam("near_threshold", near_threshold);
  nh_priv.getParam("servo_near_threshold", servo_near_threshold);
  nh_priv.getParam("config_file_path", config_file_path);
  nh_priv.getParam("curtis_theta_oppsite", curtis_theta_oppsite);
  nh_priv.getParam("curtis_vel_oppsite", curtis_vel_oppsite);
  nh_priv.getParam("left_canopen_id", left_canopen_id);
  nh_priv.getParam("right_canopen_id", right_canopen_id);
  nh_priv.getParam("vehicle_type", vehicle_type);
  nh_priv.getParam("vehicle_brand", vehicle_brand);
  nh_priv.getParam("servo_can_rx_topic", servo_can_rx_topic);
  nh_priv.getParam("can_rx_topic", can_rx_topic);
  nh_priv.getParam("cmd_topic", cmd_topic);
  nh_priv.getParam("zero_point_calibration_enable", zero_point_calibration_enable);

  // driver init
  initializeDrivers(nh);

  // subscriber init
  sub_omv_servo_cmd_ = nh.subscribe(cmd_topic, 8, &ACDriverManager::servo_cmd_callback, this);
  fork_reset_ac_sub_ = nh.subscribe("/fork_reset_curtis_cmd", 10, &ACDriverManager::fork_reset_ac_Callback, this);
  curtis_can_rx_sub_ = nh.subscribe(can_rx_topic, 1000, &ACDriverManager::can_recv_callback, this);
  servo_can_rx_sub_ = nh.subscribe(servo_can_rx_topic, 1000, &ACDriverManager::servo_can_recv_callback, this);
  manual_mode_sub_ = nh.subscribe("/peripheral_devs_state", 100, &ACDriverManager::get_manual_auto, this);

  // publisher init
  err_pub = nh.advertise<common::ERROR>("ERROR", 10);
  reset_curtis_pub = nh.advertise<std_msgs::Int8>("/reset_curtis_cmd", 10); 
  servo_encoder_pub = nh.advertise<common::servo_encoder>("/servo_encoder", 1000);

  // timer init
  timer = nh.createTimer(ros::Duration(0.05), boost::bind(&ACDriverManager::can_send_callback, this, _1));
  // dynamic reconfig init
  dynamic_reconfigure::Server<ac_node::ServoConfig> srv; 
  if (zero_point_calibration_enable)
  {
  srv.setCallback(boost::bind(&ACDriverManager::reconfig, this, _1, _2));
  }
}

void ACDriverManager::initializeDrivers(ros::NodeHandle& nh) {
  if (vehicle_brand == "xilin")
  {
    if (vehicle_type == "single")
    {
      std::shared_ptr<ac_driver> cd;
      cd = std::make_shared<curtis_driver>(nh, curtis_can_tx_topic, car_rpm_max,
                                            servo_zero_point, curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable, near_threshold);
      ac_drivers.push_back(cd);
    }
    else if (vehicle_type == "omv")
    {
      std::shared_ptr<ac_driver> cd;
      std::shared_ptr<ac_driver> sd_left;
      std::shared_ptr<ac_driver> sd_right;
      cd = std::make_shared<curtis_driver>(nh, curtis_can_tx_topic, car_rpm_max,
                                            servo_zero_point, curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable, near_threshold);
      sd_left = std::make_shared<servo_driver>(nh, left_canopen_id, "left",
                                                servo_can_tx_topic, left_servo_zero_point, k_passive_red_ratio, true, servo_near_threshold);
      sd_right = std::make_shared<servo_driver>(nh, right_canopen_id, "right",
                                                servo_can_tx_topic, right_servo_zero_point, k_passive_red_ratio, true, servo_near_threshold);
      ac_drivers.push_back(cd);
      ac_drivers.push_back(sd_left);
      ac_drivers.push_back(sd_right);
    }
  }
  else if (vehicle_brand == "rebot")
  {
    std::shared_ptr<rebot_driver> rd;
    rd = std::make_shared<rebot_driver>(nh, curtis_can_tx_topic, car_rpm_max,
                                        servo_zero_point, curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable, near_threshold);
    ac_drivers.push_back(rd);
  }

  else if (vehicle_brand == "mima")
  {
    std::shared_ptr<mima_driver> md;
    md = std::make_shared<mima_driver>(nh, curtis_can_tx_topic, car_rpm_max,
                                        servo_zero_point, curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable, near_threshold);
    ac_drivers.push_back(md);
  }
  else if (vehicle_brand == "YJL")
  {
    std::shared_ptr<YJL_driver> yd;
    yd = std::make_shared<YJL_driver>(nh, curtis_can_tx_topic, car_rpm_max,
                                        servo_zero_point, curtis_theta_oppsite, curtis_vel_oppsite, Curtis_Steer_Enable, near_threshold);
    ac_drivers.push_back(yd);
  }
  ROS_INFO("Rebot driver activated.");
}

void ACDriverManager::reconfig(ac_node::ServoConfig &config, uint32_t level)
{
    static bool save_data = false;
    servo_zero_point = config.servo_zero_point;
    for (int i = 0; i < ac_drivers.size(); i++)
    {
        ac_drivers[i]->set_zero_point(config.servo_zero_point);
    }

    // sd_banyunche->set_zero_point(config.servo_zero_point);
    std::cout << servo_zero_point << std::endl;
    if (config.save_data && !save_data)
    {
        save_zero_point(servo_zero_point);
    }
    save_data = config.save_data;
}

void ACDriverManager::handle_curtis_reset()
{
  static double reset_ac_time;
  static bool reset_flag = false;
  bool need_reset=false;
  int need_reset_num=0;
  for (int i = 0; i < ac_drivers.size(); i++)
  {
    ROS_INFO_STREAM("ac_drivers.size()"<< ac_drivers.size());
    ROS_INFO_STREAM("ac_drivers[i]->need_reset(): "<<ac_drivers[i]->need_reset());
    if (ac_drivers[i]->need_reset()==true)
    {
      need_reset_num++;
      ROS_INFO_STREAM("ac_drivers[i]->need_reset(): " << ac_drivers[i]->need_reset());
    }
  }
  if(need_reset_num== ac_drivers.size())
  {
    need_reset = true;
  }
  else 
  {
    need_reset = false;
  }
  ROS_INFO_STREAM("reset_flag: " <<reset_flag<<" need_reset: "<<need_reset<<" fork_need_reset: "<<fork_need_reset);
  if  ((!reset_flag) && (need_reset))
  {
    ROS_INFO_STREAM("reset");
  }
  ROS_INFO_STREAM((!reset_flag) && (need_reset));
  ROS_INFO_STREAM(((!reset_flag) && (need_reset)) || (fork_need_reset));
  if (((!reset_flag) && (need_reset)) || (fork_need_reset)) //&& (cd->need_reset())
  {
    ROS_INFO_STREAM(" begin of reset curtis");
    reset_ac_time = ros::Time::now().toSec();
    for (int i = 0; i < ac_drivers.size(); i++)
    {
      ac_drivers[i]->disable_send(); // cd->disable_send();
    }
    std_msgs::Int8 msg;
    reset_curtis_pub.publish(msg);
    reset_flag = true;
    fork_need_reset = false;
    return;
  }
  if (reset_flag)
  {
    if (ros::Time::now().toSec() - reset_ac_time > 7.0)
    {
      ROS_INFO_STREAM("end of reset curtis");
      for (int i = 0; i < ac_drivers.size(); i++)
      {
        ac_drivers[i]->init(); //           cd->init();
        ac_drivers[i]->enable_send(); // cd->enable_send();
      }
      reset_flag = false;
    }
  }
}

void ACDriverManager::fork_reset_ac_Callback(const std_msgs::Int8::ConstPtr &msg)
{
    if (fork_need_reset == false)
        fork_need_reset = true;
}

void ACDriverManager::can_send_callback(const ros::TimerEvent &)
{
  for (int i = 0; i < ac_drivers.size(); i++)
  {
    ac_drivers[i]->timer_can_send();
  }
  servo_encoder_msg.header.stamp = ros::Time::now();;
  servo_encoder_pub.publish(servo_encoder_msg);
  if (is_manual_mode == 1)
    handle_curtis_reset();
}

void ACDriverManager::servo_can_recv_callback(const common::can::ConstPtr &msg)
{
  std::string error_msg;
  for (int i = 0; i < ac_drivers.size(); i++)
  {
    ac_drivers[i]->controller_state_report(msg, servo_encoder_msg, error_msg);
  }
}

void ACDriverManager::can_recv_callback(const common::can::ConstPtr &msg)
{
  int res = 0;
  std::string error_msg;
  for (int i = 0; i < ac_drivers.size(); i++)
  {
    ac_drivers[i]->controller_state_report(msg, servo_encoder_msg, error_msg);
    ac_drivers[i]->controller_error_report(msg, error_msg);
  }

  if (0 != res)
  {
    common::ERROR e;
    e.header.stamp = ros::Time::now();
    e.error_id = res;
    e.error_str.data = error_msg;
    err_pub.publish(e);
  }
}

void ACDriverManager::servo_cmd_callback(const common::servo_cmd::ConstPtr &msg)
{
  ROS_ERROR("enter servo_cmd_callback");
  ROS_ERROR_STREAM("is_manual_mode: "<< is_manual_mode);
  ROS_ERROR_STREAM("ac_drivers.size(): "<< ac_drivers.size());
  bool syn_enable = true;
  if (is_manual_mode == 1)
  {
    for (int i = 0; i < ac_drivers.size(); i++)
    {
      if (!(ac_drivers[i]->judge_angle_near(msg, servo_encoder_msg)))
      {
        syn_enable = false;
      }
    }
    for (int i = 0; i < ac_drivers.size(); i++)
    {
      ROS_ERROR_STREAM("Publishing servo_cmd");
      std::cout<<"enter_servo_cmd_callback"<<std::endl;
      ac_drivers[i]->set_servo_cmd(msg,syn_enable);
    }
  }
  else
  {
    return;
  }
}

void ACDriverManager::save_zero_point(double save_zero_point)
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

void ACDriverManager::get_manual_auto(const common::peripheral_uart::ConstPtr &msg)
{
  /*
      接口板数据： 1-手动     0-自动
      AGV协议数据:    1-自动  0-手动
  */
  int manualModeFlag = msg->m_iManualModeFlag;
  static int last_mode = 0;
  if (manualModeFlag == 0)
  {
    is_manual_mode = 1;
  }
  else if (manualModeFlag == 1)
  {
    is_manual_mode = 0;
  }

  if ((last_mode == 0) && (is_manual_mode == 1))
  {
    for (int i = 0; i < ac_drivers.size(); i++)
    {
      // ROS_WARN_STREAM("ac_drivers init...." << i << " ac_drivers.size(): " << ac_drivers.size());
      ac_drivers[i]->init();
    }
  }
  last_mode = is_manual_mode;
  for(int i = 0; i < ac_drivers.size(); i++)
  {
    // ROS_WARN_STREAM("set_manual_mode...." << i << " ac_drivers.size(): " << ac_drivers.size());
    ac_drivers[i]->set_manual_mode(is_manual_mode);
  }
}
