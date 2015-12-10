/*
 * DJI_Pro_Link.h
 *
 *  Created on: Mar 12, 2015
 *      Author: wuyuwei
 */

#ifndef DJI_PRO_LINK_H_
#define DJI_PRO_LINK_H_

#include <stdint.h>

#define DJI_SDK_PRO_VER                     0

#define POLL_TICK							20  //unit is ms

typedef struct ProHeader
{
	unsigned int sof : 8; 			// 1byte
	unsigned int length : 10;
	unsigned int version : 6; 		// 2byte
	unsigned int session_id : 5;
	unsigned int is_ack : 1;
	unsigned int reversed0 : 2; 	// always 0

	unsigned int padding : 5;
	unsigned int enc_type : 3;
	unsigned int reversed1 : 24;

	unsigned int sequence_number : 16;
	unsigned int head_crc : 16;
	unsigned int magic[0];
} ProHeader;

typedef void (*ACK_Callback_Func)(ProHeader *pHeader);

typedef struct ProSendParameter {
  uint16_t       session_mode : 2; /**< 0: no ack; 1: ack maybe 2: must*/
  uint16_t       need_encrypt : 1;
  uint16_t       retry_time   : 13;
  uint16_t       ack_timeout;  //unit is ms
  uint32_t       length;
  unsigned char* buf;
  ACK_Callback_Func ack_callback;
} ProSendParameter;

typedef struct ProAckParameter {
  uint16_t       session_id   : 8;
  uint16_t       need_encrypt : 8;
  uint16_t       seq_num;
  uint32_t       length;
  unsigned char* buf;
} ProAckParameter;

unsigned int Get_TimeStamp(void);
void Pro_Link_Setup(void);
void Pro_Config_Comm_Encrypt_Key(const char *key);
int Pro_Ack_Interface(ProAckParameter *parameter);
int Pro_Send_Interface(ProSendParameter *parameter);
void Pro_Request_Interface(ProHeader *header);
void Pro_Link_Recv_Hook(ProHeader *header);
int  Pro_Send_Interface(ProSendParameter *parameter);
typedef void (*Req_Callback_Func)(ProHeader *pHeader);
void Pro_App_Recv_Set_Hook(Req_Callback_Func p_hook);

#endif /* DJI_PRO_LINK_H_ */
