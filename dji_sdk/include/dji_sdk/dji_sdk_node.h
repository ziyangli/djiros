#ifndef __DJI_SDK_NODE_H__
#define __DJI_SDK_NODE_H__

#include <boost/bind.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>

#include <std_srvs/Empty.h>

#include "dji_sdk/dji_sdk.h"

#define C_G          (double) 9.80665
#define C_EARTH      (double) 6378137.0
#define C_PI         (double) 3.141592653589793

#define ENC_ON              1
#define ENC_OFF             0
#define SESSION_ACK_OFF     0
#define SESSION_ACK_MAYBE   1
#define SESSION_ACK_ON      2

#define CTRL_API_REFUSE     0x0000
#define CTRL_API_CLOSED     0x0001
#define CTRL_API_OPENED     0x0002
#define CTRL_API_APP_REFUSE 0x0003
#define CTRL_API_APP_CLOSED 0x0004

class DJISDKNode {

 public:
  DJISDKNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

 private:

  void init_publishers(ros::NodeHandle& nh) {
    // start ros publisher
    imu_pub  = nh.advertise<sensor_msgs::Imu>("imu", 1);
    gps_pub  = nh.advertise<sensor_msgs::NavSatFix>("gps", 1);
    mag_pub  = nh.advertise<sensor_msgs::MagneticField>("mag", 1);
    baro_pub = nh.advertise<sensor_msgs::FluidPressure>("baro", 1);
  }

  void init_services(ros::NodeHandle& nh) {
    attitude_control_service = nh.advertiseService("dji_sdk/attitude_control", &DJISDKNode::attitude_control_callback, this);
    sdk_permission_control_service = nh.advertiseService("dji_sdk/sdk_permission_control", &DJISDKNode::sdk_permission_control_callback, this);
    sync_flag_control_service = nh.advertiseService("dji_sdk/sync_flag_control", &DJISDKNode::sync_flag_control_callback, this);

    turn_on_api_service = nh.advertiseService("api", &DJISDKNode::api_srv_callback, this);
  }

  int init_parameters(ros::NodeHandle& nh_private);

  void broadcast_callback();

  bool process_waypoint(dji_sdk::Waypoint new_waypoint);

  void gps_convert_ned(float &ned_x, float &ned_y, double gps_t_lon, double gps_t_lat, double gps_r_lon, double gps_r_lat);

  // dji_sdk::LocalPosition gps_convert_ned(dji_sdk::GlobalPosition loc);

  bool attitude_control_callback(dji_sdk::AttitudeControl::Request& request, dji_sdk::AttitudeControl::Response& response);
  bool sdk_permission_control_callback(dji_sdk::SDKPermissionControl::Request& request, dji_sdk::SDKPermissionControl::Response& response);
  bool sync_flag_control_callback(dji_sdk::SyncFlagControl::Request& request, dji_sdk::SyncFlagControl::Response& response);

  bool api_srv_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& ret);
  static void api_ack_callback(ProHeader* header);

 private:

  static bool API_ON;  ///< indicate api on or not

  // activation
  char app_key[65];
  activate_data_t user_act_data;

  // Publishers:
  ros::Publisher imu_pub;
  ros::Publisher gps_pub;
  ros::Publisher baro_pub;
  ros::Publisher mag_pub;

  // Services:
  ros::ServiceServer attitude_control_service;
  ros::ServiceServer sdk_permission_control_service;
  ros::ServiceServer turn_on_api_service;
  ros::ServiceServer sync_flag_control_service;


};

#endif
