/*
 * DJI_Pro_Rmu.h
 *
 *  Created on: 24 Aug, 2015
 *      Author: wuyuwei
 */

#ifndef DJI_PRO_RMU_H_
#define DJI_PRO_RMU_H_

#include <stdint.h>

#include "DJI_Pro_Link.h"
#include "DJI_Pro_Config.h"

#define PRO_PURE_DATA_MAX_SIZE			1007   // 2^10 - pro_header_size

/* memory management unit */
#define STATIC_MEMORY_SIZE				MEMORY_SIZE
#define MMU_TABLE_NUM					32

/**
 * @brief memory management unit table
 *
 * manually allocate memory for safety and
 * efficiency in MCU usecase
 */
typedef struct MMU_Tab {
  uint8_t  id;      /**< table index */
  uint8_t  status;  /**< 1-> using, 0-> free */
  uint16_t len;     /**< size of allocated memory */
  uint8_t* mem;     /**< pointer to the memory */
} MMU_Tab;

/* session management unit */

#define ACK_SESSION_IDLE				0
#define ACK_SESSION_PROCESS				1
#define ACK_SESSION_USING				2

#define SESSION_TABLE_NUM				32
#define CMD_SESSION_0					0
#define CMD_SESSION_1					1
#define CMD_SESSION_AUTO				32

typedef struct CMD_Session_Tab {
  uint32_t id              : 5;  /**< session id */
  uint32_t status          : 1;  /**< 1->using; 0->free */
  uint32_t cnt_send        : 5;  /**< sending time */
  uint32_t max_retry       : 5;  /**< max sending time, 0 means inf*/
  uint32_t ack_timeout     : 16;
  MMU_Tab  *mmu;
  ACK_Callback_Func ack_callback;
  uint32_t pre_seq_num;
  uint32_t pre_timestamp;
} CMD_Session_Tab;

typedef struct ACK_Session_Tab {
  uint32_t session_id     : 5;
  uint32_t status         : 2;
  uint32_t res            : 25;
  MMU_Tab* mmu;
} ACK_Session_Tab;

extern CMD_Session_Tab * Get_CMD_Session_Tab(void);
extern ACK_Session_Tab * Get_ACK_Session_Tab(void);
extern void MMU_Setup(void);
extern void Free_Memory(MMU_Tab *mmu_tab);
extern MMU_Tab * Request_Memory(unsigned short size);
extern void Get_Memory_Lock(void);
extern void Free_Memory_Lock(void);
extern void Display_Memory_Info(void);

extern void Session_Setup(void);
extern CMD_Session_Tab * Request_CMD_Session(unsigned short session_id,unsigned short size);
extern void Free_CMD_Session(CMD_Session_Tab * session);
extern ACK_Session_Tab * Request_ACK_Session(unsigned short session_id,unsigned short size);
extern void Free_ACK_Session(ACK_Session_Tab * session);

extern void DJI_Pro_Rmu_Setup(void);

#endif /* DJI_PRO_RMU_H_ */
