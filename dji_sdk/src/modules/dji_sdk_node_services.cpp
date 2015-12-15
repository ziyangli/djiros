#include <dji_sdk/dji_sdk_node.h>

bool DJISDKNode::attitude_control_callback(dji_sdk::AttitudeControl::Request& request, dji_sdk::AttitudeControl::Response& response)
{
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

bool DJISDKNode::virtual_rc_enable_control_callback(dji_sdk::VirtualRCEnableControl::Request& request, dji_sdk::VirtualRCEnableControl::Response& response)
{
  virtual_rc_manager_t virtual_rc_manager;
  virtual_rc_manager.enable = request.enable;
  virtual_rc_manager.if_back_to_real = request.if_back_to_real;
  DJI_Pro_Virtual_RC_Manage(&virtual_rc_manager);

  response.result = true;
  return true;
}

bool DJISDKNode::virtual_rc_data_control_callback(dji_sdk::VirtualRCDataControl::Request& request, dji_sdk::VirtualRCDataControl::Response& response)
{
  virtual_rc_data_t virtual_rc_data;
  std::copy(request.channel_data.begin(), request.channel_data.end(), virtual_rc_data.channel_data);
  DJI_Pro_Virtual_RC_Send_Value(&virtual_rc_data);

  response.result = true;
  return true;
}

bool DJISDKNode::drone_arm_control_callback(dji_sdk::DroneArmControl::Request& request, dji_sdk::DroneArmControl::Response& response)
{
  uint8_t arm = request.arm;
  DJI_Pro_Arm_Control(arm);

  response.result = true;
  return true;
}

bool DJISDKNode::sync_flag_control_callback(dji_sdk::SyncFlagControl::Request& request, dji_sdk::SyncFlagControl::Response& response)
{
  uint32_t frequency = request.frequency;
  DJI_Pro_Send_Sync_Flag(frequency);

  response.result = true;
  return true;
}

bool DJISDKNode::message_frequency_control_callback(dji_sdk::MessageFrequencyControl::Request& request, dji_sdk::MessageFrequencyControl::Response& response)
{
  sdk_msgs_frequency_data_t message_frequency;
  std::copy(request.frequency.begin(), request.frequency.end(), message_frequency.std_freq);
  DJI_Pro_Set_Msgs_Frequency(&message_frequency);

  response.result = true;
  return true;
}
