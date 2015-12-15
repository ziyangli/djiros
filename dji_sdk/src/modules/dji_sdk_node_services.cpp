#include "dji_sdk/dji_sdk_node.h"

bool DJISDKNode::attitude_control_callback(dji_sdk::AttitudeControl::Request& request, dji_sdk::AttitudeControl::Response& response) {
  attitude_data_t user_ctrl_data;

  user_ctrl_data.ctrl_flag = request.flag;
  user_ctrl_data.roll_or_x = request.x;
  user_ctrl_data.pitch_or_y = request.y;
  user_ctrl_data.thr_z = request.z;
  user_ctrl_data.yaw = request.yaw;

  DJI_Pro_Attitude_Control(&user_ctrl_data);

  response.result = true;
  return true;
}

bool DJISDKNode::sdk_permission_control_callback(dji_sdk::SDKPermissionControl::Request& request, dji_sdk::SDKPermissionControl::Response& response) {
  if (request.control_enable == 1) {
    printf("Request Control");
    DJI_Pro_Control_Management(1, NULL);
    response.result = true;
  }
  else if (request.control_enable == 0) {
    printf("Release Control");
    DJI_Pro_Control_Management(0, NULL);
    response.result = true;
  }
  else
    response.result = false;

  return true;
}

bool DJISDKNode::sync_flag_control_callback(dji_sdk::SyncFlagControl::Request& request, dji_sdk::SyncFlagControl::Response& response) {
  uint32_t frequency = request.frequency;
  DJI_Pro_Send_Sync_Flag(frequency);

  response.result = true;
  return true;
}

bool DJISDKNode::api_srv_callback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  static ros::Time previous_api_srv_stamp = ros::Time::now() - ros::Duration(1.5);

  if (API_ON) {
    ROS_INFO("ncore_bridge: ctrl. api already enabled.");
    return true;
  }

  ros::Time this_time = ros::Time::now();
  if (this_time - previous_api_srv_stamp > ros::Duration(1.0)) {
    ROS_INFO("ncore_bridge: enabling ctrl. api...");
    uint8_t send_data = 0x01;

    // need ack to toggle boolen
    DJI_Pro_App_Send_Data(SESSION_ACK_ON, ENC_OFF, MY_CTRL_CMD_SET, API_CTRL_MANAGEMENT, &send_data, sizeof(send_data), api_ack_callback, 500, 1); // api_ack_callback

    previous_api_srv_stamp = this_time;

    return false;
  }
  else {
    /* request too frequently */
    // ROS_INFO("ncore_bridge: ...");
    return false;
  }
}

/* /brief util function envoked by api_srv_callback
 *
 */
void DJISDKNode::api_ack_callback(ProHeader* header) {
  uint16_t ack_data = 0xFFFF;
  memcpy((uint8_t *)&ack_data, (uint8_t *)&header->magic, (header->length - EXC_DATA_SIZE));

  if (ack_data != CTRL_API_OPENED) {
    API_ON = false;
    ROS_WARN("ncore_bridge: use offboard swich to enable ctrl api!");
  }
  else {
    API_ON = true;
    ROS_INFO("ncore_bridge: FCU offboard enabled.");
  }
}
