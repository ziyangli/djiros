/*
 * DJI_Pro_Link.c
 *
 *  Created on: Mar 12, 2015
 *      Author: wuyuwei
 */

#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include "DJI_Pro_Link.h"
#include "DJI_Pro_Hw.h"
#include "DJI_Pro_Codec.h"
#include "DJI_Pro_Rmu.h"

static ACK_Callback_Func Call_APP_Func = 0;
static Req_Callback_Func APP_Recv_Hook = 0;

static void Send_Pro_Data(unsigned char* buf) {
  ProHeader* pHeader = (ProHeader*)buf;

#ifdef PLATFORM_QT
  DJI_Pro_Hw::Pro_Hw_Get_Instance()->Pro_Hw_Send(buf,pHeader->length);
#else
  Pro_Hw_Send(buf, pHeader->length);
#endif
}

void Pro_Link_Recv_Hook(ProHeader *header) {
  ProHeader* p2header;
  static ACK_Session_Tab* ack_session = Get_ACK_Session_Tab();
  static CMD_Session_Tab* cmd_session = Get_CMD_Session_Tab();

  // TODO: parse the protocol data stream here
  int id_to_check = header->session_id;
  if (header->is_ack == 1) {

    if (id_to_check == 1) {
      // handle ack for session 1
      if (cmd_session[1].status == 1 && cmd_session[1].ack_callback) {
        cmd_session[1].ack_callback(header);
        Get_Memory_Lock();
        Free_CMD_Session(&cmd_session[1]);
        Free_Memory_Lock();
      }
    }
    else if (id_to_check > 1 && id_to_check < 32) {
      // handle ack for sessions 2-31
      if (cmd_session[id_to_check].status == 1) {
        Get_Memory_Lock();
        p2header = (ProHeader*)cmd_session[id_to_check].mmu->mem;
        if (p2header->sequence_number == header->sequence_number) {

          if (p2header->session_id != id_to_check) {
            printf("Error!!!! session id mismatch!!!!");
          }
          // printf("%s: Recv Session %d ACK\n", __func__, p2header->session_id);
          Call_APP_Func = cmd_session[id_to_check].ack_callback;
          Free_CMD_Session(&cmd_session[id_to_check]);
          Free_Memory_Lock();
          if (Call_APP_Func) {
            Call_APP_Func(header);
          }
        }
        else {
          Free_Memory_Lock();
        }
      }
    }
  }
  else {
    // handle request(normal data)
    switch (id_to_check) {
      case 0:  // leave session 0 to user handler
        Pro_Request_Interface(header);
        break;
      default:
        if (ack_session[id_to_check - 1].status == ACK_SESSION_PROCESS) {
          printf("%s, This session is waiting for App ack: session id = %d, seq_num = %d\n", __func__, id_to_check, header->sequence_number);
        }
        else if (ack_session[id_to_check - 1].status == ACK_SESSION_IDLE) {
          if (id_to_check > 1) {
            // only change for session 2-32???
            ack_session[id_to_check - 1].status = ACK_SESSION_PROCESS;
          }
          Pro_Request_Interface(header);
        }
        else if (ack_session[id_to_check - 1].status == ACK_SESSION_USING) {
          Get_Memory_Lock();
          p2header = (ProHeader*)ack_session[id_to_check - 1].mmu->mem;
          if (header->sequence_number == p2header->sequence_number) {
            printf("%s: repeat ACK to remote, session id = %d, seq_num = %d\n", __func__, id_to_check, header->sequence_number);
            Send_Pro_Data(ack_session[id_to_check - 1].mmu->mem);
            Free_Memory_Lock();
          }
          else {
            printf("%s: same session, but new seq_num pkg, session id = %d, pre seq_num = %d, cur seq_num=%d\n", __func__, id_to_check, p2header->sequence_number, header->sequence_number);
            ack_session[id_to_check - 1].status = ACK_SESSION_PROCESS;
            Free_Memory_Lock();
            Pro_Request_Interface(header);
          }
        }
        break;
    }
  }
}

/**
 * @brief polling function
 *
 * polls global cmd sessions table and send data if
 * there is any,
 * this implementation is buggy since it overwhelms
 * the route when ack_timeout is not well defined.
 */
static void Send_Poll(void) {
  static CMD_Session_Tab* cmd_session = Get_CMD_Session_Tab();
  for (int i = 1 ; i < SESSION_TABLE_NUM ; i++) {
    if (cmd_session[i].status == 1) {
      // [TODO]: who frees session status?
      unsigned int cur_timestamp = Get_TimeStamp();
      if ((cur_timestamp - cmd_session[i].pre_timestamp)
          > cmd_session[i].ack_timeout) {
        // user's resposibility to asure ack_timeout is not too small
        Get_Memory_Lock();
        if (cmd_session[i].max_retry > 0) {
          if (cmd_session[i].cnt_send >= cmd_session[i].max_retry) {
            Free_CMD_Session(&cmd_session[i]);
          }
          else {
            Send_Pro_Data(cmd_session[i].mmu->mem);
            cmd_session[i].pre_timestamp = cur_timestamp;
            cmd_session[i].cnt_send++;
          }
        }
        else {
          Send_Pro_Data(cmd_session[i].mmu->mem);
          cmd_session[i].pre_timestamp = cur_timestamp;
        }
        Free_Memory_Lock();
      }
    }
  }
}

static void* PollThread(void* arg) {
  while (1) {
    Send_Poll();
    // extra sleep in case cmd session is empty
    usleep(POLL_TICK * 1000);
  }
  return NULL;
}

static int Start_PollThread(void) {
  pthread_t A_ARR;
  int ret = pthread_create(&A_ARR, 0, PollThread, NULL);
  if (ret != 0) {
    return -1;
  }
  return 0;
}

unsigned int Get_TimeStamp(void) {
  struct timeval cur_time;
  gettimeofday(&cur_time,NULL);
  return (cur_time.tv_sec * 1000) + (cur_time.tv_usec / 1000);
}

void Pro_Link_Setup(void) {
  DJI_Pro_Rmu_Setup();
  Start_PollThread();
}

void Pro_Config_Comm_Encrypt_Key(const char *key) {
  sdk_set_encrypt_key_interface(key);
}

static unsigned short Pro_Calc_Length(unsigned short size, unsigned short encrypt_flag) {
  unsigned short len = size + sizeof(ProHeader) + 4;

  if (encrypt_flag) {
    // encrypted data is 16bytes padded
    len += (16 - size % 16);
  }

  return len;
}

int Pro_Ack_Interface(ProAckParameter* params) {
  unsigned short ret = 0;
  ACK_Session_Tab* ack_session = (ACK_Session_Tab*)NULL;;

  if (params->length > PRO_PURE_DATA_MAX_SIZE) {
    printf("%s: %d: ERROR, length = %d is oversize\n", __func__, __LINE__, params->length);
    return -1;
  }

  if (params->session_id > 0 && params->session_id < 32) {
    Get_Memory_Lock();
    ack_session = Request_ACK_Session(params->session_id, Pro_Calc_Length(params->length, params->need_encrypt));
    if (ack_session == (ACK_Session_Tab*)NULL) {
      printf("%s: %d: ERROR, there is not enough memory\n", __func__, __LINE__);
      Free_Memory_Lock();
      return -1;
    }

    ret = sdk_encrypt_interface(ack_session->mmu->mem, params->buf, params->length, 1, params->need_encrypt, params->session_id,params->seq_num);

    if (ret == 0) {
      printf("%s: %d: encrypt ERROR\n", __func__, __LINE__);
      Free_Memory_Lock();
      return -1;
    }

    Send_Pro_Data(ack_session->mmu->mem);
    Free_Memory_Lock();
    ack_session->status = ACK_SESSION_USING;
    return 0;
  }

  return -1;
}

int Pro_Send_Interface(ProSendParameter *params) {
  unsigned short ret = 0;
  CMD_Session_Tab* cmd_session = NULL;
  static unsigned short global_seq_num = 0;

  if (params->length > PRO_PURE_DATA_MAX_SIZE) {
    printf("%s: %d: ERROR, length = %d is oversize\n", __func__, __LINE__, params->length);
    return -1;
  }

  switch (params->session_mode) {
    case 0:
      Get_Memory_Lock();
      cmd_session = Request_CMD_Session(CMD_SESSION_0, Pro_Calc_Length(params->length, params->need_encrypt));
      if (cmd_session == NULL) {
        Free_Memory_Lock();
        printf("%s: %d: ERROR, there is not enough memory\n", __func__, __LINE__);
        return -1;
      }
      ret = sdk_encrypt_interface(cmd_session->mmu->mem, params->buf, params->length, 0, params->need_encrypt, cmd_session->id, global_seq_num);
      if (ret == 0) {
        printf("%s: %d: encrypt ERROR\n", __func__, __LINE__);
        Free_CMD_Session(cmd_session);
        Free_Memory_Lock();
        return -1;
      }
      Send_Pro_Data(cmd_session->mmu->mem);
      global_seq_num++;
      Free_CMD_Session(cmd_session);
      Free_Memory_Lock();
      break;
    case 1:
      Get_Memory_Lock();
      cmd_session = Request_CMD_Session(CMD_SESSION_1, Pro_Calc_Length(params->length, params->need_encrypt));
      if (cmd_session == NULL) {
        Free_Memory_Lock();
        printf("%s: %d: ERROR, there is not enough memory\n", __func__, __LINE__);
        return -1;
      }
      if (global_seq_num == cmd_session->pre_seq_num) {
        global_seq_num++;
      }
      ret = sdk_encrypt_interface(cmd_session->mmu->mem, params->buf, params->length, 0, params->need_encrypt, cmd_session->id, global_seq_num);
      if (ret == 0) {
        printf("%s:%d:encrypt ERROR\n",__func__,__LINE__);
        Free_CMD_Session(cmd_session);
        Free_Memory_Lock();
        return -1;
      }
      cmd_session->pre_seq_num = global_seq_num++;
      cmd_session->ack_callback = params->ack_callback;
      cmd_session->ack_timeout = (params->ack_timeout > POLL_TICK) ? params->ack_timeout : POLL_TICK;

      cmd_session->pre_timestamp = Get_TimeStamp();
      cmd_session->cnt_send  = 1;
      // ack is not complusory for session 1
      cmd_session->max_retry = 1;

      Send_Pro_Data(cmd_session->mmu->mem);
      Free_Memory_Lock();
      break;
    case 2:
      Get_Memory_Lock();
      cmd_session = Request_CMD_Session(CMD_SESSION_AUTO, Pro_Calc_Length(params->length,params->need_encrypt));
      if (cmd_session == NULL) {
        Free_Memory_Lock();
        printf("%s: %d: ERROR, there is not enough memory\n", __func__, __LINE__);
        return -1;
      }
      if (global_seq_num == cmd_session->pre_seq_num) {
        global_seq_num++;
      }
      ret = sdk_encrypt_interface(cmd_session->mmu->mem, params->buf, params->length, 0, params->need_encrypt, cmd_session->id, global_seq_num);
      if (ret == 0) {
        printf("%s: %d: encrypt ERROR\n", __func__, __LINE__);
        Free_CMD_Session(cmd_session);
        Free_Memory_Lock();
        return -1;
      }
      cmd_session->pre_seq_num = global_seq_num++;
      cmd_session->ack_callback = params->ack_callback;
      cmd_session->ack_timeout = (params->ack_timeout > POLL_TICK) ? params->ack_timeout : POLL_TICK;
      cmd_session->pre_timestamp = Get_TimeStamp();
      cmd_session->cnt_send = 1;
      cmd_session->max_retry = params->retry_time;
      Send_Pro_Data(cmd_session->mmu->mem);
      Free_Memory_Lock();
      break;
  }
  return 0;
}

void Pro_App_Recv_Set_Hook(Req_Callback_Func p_hook) {
  APP_Recv_Hook = p_hook;
}

/// \brief protocal request interface
///
/// envoke common(non ack?) data handling function,
/// APP_Recv_Hook should normally be setup as
/// DJI_Pro_App_Recv_Req_Data()
void Pro_Request_Interface(ProHeader* header) {
  // TODO call app data handler interface here
  if (APP_Recv_Hook) {
    APP_Recv_Hook(header);
  }
  else {
    // which should never happen

    printf("%s: Recv request, session id = %d, seq_num = %d\n",
           __func__, header->session_id, header->sequence_number);
    if (header->session_id > 0) {
      ProAckParameter params;
      unsigned char buf[2] = {0, 0};
      params.session_id     = header->session_id;
      params.seq_num        = header->sequence_number;
      params.need_encrypt   = header->enc_type;
      params.buf            = buf;
      params.length         = sizeof(buf);
      Pro_Ack_Interface(&params);
    }
  }
}
