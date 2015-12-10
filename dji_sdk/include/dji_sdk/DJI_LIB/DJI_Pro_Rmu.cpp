/*
 * DJI_Pro_Rmu.cpp
 * Des:RMU,means Resource Management Unit, includes memory and session management
 *  Created on: 24 Aug, 2015
 *      Author: wuyuwei
 */


#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include "DJI_Pro_Rmu.h"

static pthread_mutex_t mmu_lock = PTHREAD_MUTEX_INITIALIZER;

static MMU_Tab DJI_MMU_Tab[MMU_TABLE_NUM];
static unsigned char Static_Memory[STATIC_MEMORY_SIZE];

static CMD_Session_Tab DJI_CMD_Session_Tab[SESSION_TABLE_NUM];
static ACK_Session_Tab DJI_ACK_Session_Tab[SESSION_TABLE_NUM - 1]; //session 0 is a nak session id

CMD_Session_Tab* Get_CMD_Session_Tab(void) {
  return DJI_CMD_Session_Tab;
}

ACK_Session_Tab* Get_ACK_Session_Tab(void) {
  return DJI_ACK_Session_Tab;
}

/**
 * manaully allocate memory
 */
void MMU_Setup(void) {
  DJI_MMU_Tab[0].id     = 0;
  DJI_MMU_Tab[0].status = 1;
  DJI_MMU_Tab[0].len    = 0;
  DJI_MMU_Tab[0].mem    = Static_Memory;

  for (int i = 1; i < (MMU_TABLE_NUM - 1); i++) {
    DJI_MMU_Tab[i].id     = i;
    DJI_MMU_Tab[i].status = 0;
  }

  DJI_MMU_Tab[MMU_TABLE_NUM - 1].id     = MMU_TABLE_NUM - 1;
  DJI_MMU_Tab[MMU_TABLE_NUM - 1].status = 1;
  DJI_MMU_Tab[MMU_TABLE_NUM - 1].len    = 0;
  DJI_MMU_Tab[MMU_TABLE_NUM - 1].mem    = Static_Memory + STATIC_MEMORY_SIZE; // [TODO]: this point to an undefined location!!!
}

void Get_Memory_Lock(void) {
  pthread_mutex_lock(&mmu_lock);
}

void Free_Memory_Lock(void)
{
  pthread_mutex_unlock(&mmu_lock);
}

void Free_Memory(MMU_Tab* mmu_tab) {
  if (mmu_tab == (MMU_Tab*)NULL) {
    return;
  }

  if (mmu_tab->id == 0 || mmu_tab->id == (MMU_TABLE_NUM - 1)) {
    return;
  }

  mmu_tab->status = 0;
}

void Display_Memory_Info(void)
{
#ifdef SYS_MEM_DEBUG
  unsigned char i,j;
  unsigned char mmu_tab_used_num = 0;
  unsigned char mmu_tab_used_index[MMU_TABLE_NUM];

  unsigned char temp8;
  unsigned int temp32;

  static unsigned int g_line = 1;

  for(i = 0 ; i < MMU_TABLE_NUM ; i ++)
  {
    if(DJI_MMU_Tab[i].status == 1)
    {
      mmu_tab_used_index[mmu_tab_used_num ++] = DJI_MMU_Tab[i].id;
    }
  }

  for(i = 0 ; i < (mmu_tab_used_num - 1) ; i ++)
  {
    for(j = 0; j < (mmu_tab_used_num - i - 1) ; j ++)
    {
      if(DJI_MMU_Tab[mmu_tab_used_index[j]].mem >
         DJI_MMU_Tab[mmu_tab_used_index[j + 1]].mem)
      {
        temp8 = mmu_tab_used_index[j + 1];
        mmu_tab_used_index[j + 1] = mmu_tab_used_index[j];
        mmu_tab_used_index[j] = temp8;
      }
    }
  }

  printf("***************(%d)******************\n",g_line++);

  for(i = 0 ; i < mmu_tab_used_num - 1 ; i ++)
  {
    printf("<S=%08X L=%d I=%d,E=%08X>\n",(unsigned long)DJI_MMU_Tab[mmu_tab_used_index[i]].mem,
           DJI_MMU_Tab[mmu_tab_used_index[i]].len,
           DJI_MMU_Tab[mmu_tab_used_index[i]].id,
           (unsigned long)(DJI_MMU_Tab[mmu_tab_used_index[i]].mem +
                           DJI_MMU_Tab[mmu_tab_used_index[i]].len));

    if(DJI_MMU_Tab[mmu_tab_used_index[i + 1]].mem > (DJI_MMU_Tab[mmu_tab_used_index[i]].mem +
                                                     DJI_MMU_Tab[mmu_tab_used_index[i]].len))
    {
      temp32 = (unsigned int)(DJI_MMU_Tab[mmu_tab_used_index[i + 1]].mem -
                              DJI_MMU_Tab[mmu_tab_used_index[i]].mem) -
               DJI_MMU_Tab[mmu_tab_used_index[i]].len;
      printf("         --idle=%d--\n",temp32);
    }
    else
    {
      printf("         --idle=0--\n");
    }

  }

  printf("<S=%08X L=%d I=%d E=%08X>\n",(unsigned long)DJI_MMU_Tab[mmu_tab_used_index[mmu_tab_used_num - 1]].mem,
         DJI_MMU_Tab[mmu_tab_used_index[mmu_tab_used_num - 1]].len,
         DJI_MMU_Tab[mmu_tab_used_index[mmu_tab_used_num - 1]].id,
         (unsigned long)(DJI_MMU_Tab[mmu_tab_used_index[mmu_tab_used_num - 1]].mem +
                         DJI_MMU_Tab[mmu_tab_used_index[mmu_tab_used_num - 1]].len));
#endif
}

MMU_Tab* Request_Memory(unsigned short size) {
  unsigned int mem_used = 0;
  unsigned char i;
  unsigned char j = 0;
  unsigned char mmu_tab_used_num = 0;
  unsigned char mmu_tab_used_index[MMU_TABLE_NUM];

  unsigned int temp32;
  unsigned int temp_area[2] = {0xFFFFFFFF, 0xFFFFFFFF};

  unsigned int record_temp32 = 0;
  unsigned char magic_flag = 0;

  if (size > PRO_PURE_DATA_MAX_SIZE || size > STATIC_MEMORY_SIZE) {
    return NULL;
  }

  for (i = 0; i < MMU_TABLE_NUM; i++) {
    if (DJI_MMU_Tab[i].status == 1) {
      mem_used += DJI_MMU_Tab[i].len;
      mmu_tab_used_index[mmu_tab_used_num ++] = DJI_MMU_Tab[i].id;
    }
  }

  if (STATIC_MEMORY_SIZE < (mem_used + size)) {
    return (MMU_Tab *)0;
  }

  if(mem_used == 0)
  {
    DJI_MMU_Tab[1].mem = DJI_MMU_Tab[0].mem;
    DJI_MMU_Tab[1].len = size;
    DJI_MMU_Tab[1].status = 1;
    return &DJI_MMU_Tab[1];
  }

  for(i = 0 ; i < (mmu_tab_used_num - 1) ; i ++)
  {
    for(j = 0; j < (mmu_tab_used_num - i - 1) ; j ++)
    {
      if(DJI_MMU_Tab[mmu_tab_used_index[j]].mem >
         DJI_MMU_Tab[mmu_tab_used_index[j + 1]].mem)
      {
        mmu_tab_used_index[j + 1] ^= mmu_tab_used_index[j];
        mmu_tab_used_index[j] ^= mmu_tab_used_index[j + 1];
        mmu_tab_used_index[j + 1] ^= mmu_tab_used_index[j];
      }
    }
  }

  for(i = 0 ; i < (mmu_tab_used_num - 1) ; i ++)
  {
    temp32 = (unsigned int)(DJI_MMU_Tab[mmu_tab_used_index[i + 1]].mem -
                            DJI_MMU_Tab[mmu_tab_used_index[i]].mem);

    if((temp32 - DJI_MMU_Tab[mmu_tab_used_index[i]].len) >= size)
    {
      if(temp_area[1] > (temp32 - DJI_MMU_Tab[mmu_tab_used_index[i]].len))
      {
        temp_area[0] = DJI_MMU_Tab[mmu_tab_used_index[i]].id;
        temp_area[1] = temp32 - DJI_MMU_Tab[mmu_tab_used_index[i]].len;
      }
    }

    record_temp32 += temp32 - DJI_MMU_Tab[mmu_tab_used_index[i]].len;
    if(record_temp32 >= size && magic_flag == 0)
    {
      j = i;
      magic_flag = 1;
    }
  }

  if(temp_area[0] == 0xFFFFFFFF && temp_area[1] == 0xFFFFFFFF)
  {
    for(i = 0; i < j; i ++)
    {
      if(DJI_MMU_Tab[mmu_tab_used_index[i + 1]].mem
         >  (DJI_MMU_Tab[mmu_tab_used_index[i]].mem +
             DJI_MMU_Tab[mmu_tab_used_index[i]].len))
      {
        memmove(DJI_MMU_Tab[mmu_tab_used_index[i]].mem +
                DJI_MMU_Tab[mmu_tab_used_index[i]].len,
                DJI_MMU_Tab[mmu_tab_used_index[i + 1]].mem,
                DJI_MMU_Tab[mmu_tab_used_index[i + 1]].len);
        DJI_MMU_Tab[mmu_tab_used_index[i + 1]].mem = DJI_MMU_Tab[mmu_tab_used_index[i]].mem +
                                                     DJI_MMU_Tab[mmu_tab_used_index[i]].len;

        //printf("move id=%d\n",
        //       DJI_MMU_Tab[mmu_tab_used_index[i + 1]].id);
      }
    }

    for(i = 1 ; i < (MMU_TABLE_NUM - 1) ; i ++)
    {
      if(DJI_MMU_Tab[i].status == 0)
      {
        DJI_MMU_Tab[i].mem =
            DJI_MMU_Tab[mmu_tab_used_index[j]].mem +
            DJI_MMU_Tab[mmu_tab_used_index[j]].len;

        DJI_MMU_Tab[i].len = size;
        DJI_MMU_Tab[i].status = 1;
        return &DJI_MMU_Tab[i];
      }
    }
    return (MMU_Tab *)0;
  }

  for(i = 1 ; i < (MMU_TABLE_NUM - 1) ; i ++)
  {
    if(DJI_MMU_Tab[i].status == 0)
    {
      DJI_MMU_Tab[i].mem = DJI_MMU_Tab[temp_area[0]].mem +
                           DJI_MMU_Tab[temp_area[0]].len;

      DJI_MMU_Tab[i].len = size;
      DJI_MMU_Tab[i].status = 1;
      return &DJI_MMU_Tab[i];
    }
  }

  return (MMU_Tab *)NULL;
}

void Session_Setup(void) {
  for (int i = 0; i < SESSION_TABLE_NUM ; i++) {
    DJI_CMD_Session_Tab[i].id     = i;
    DJI_CMD_Session_Tab[i].status = 0;
    DJI_CMD_Session_Tab[i].mmu    = (MMU_Tab *)NULL;
  }

  for (int i = 0; i < (SESSION_TABLE_NUM - 1) ; i++) {
    DJI_ACK_Session_Tab[i].session_id = i + 1;
    DJI_ACK_Session_Tab[i].status = ACK_SESSION_IDLE;
    DJI_ACK_Session_Tab[i].mmu = (MMU_Tab *)NULL;
  }
}

/* request a cmd session for sending cmd data
 * when arg session_id = 0/1, which means select session 0/1 to send cmd
 * otherwise set arg session_id = CMD_SESSION_AUTO (32), which means auto select an idle session id between 2~31.
 */
CMD_Session_Tab* Request_CMD_Session(unsigned short session_id, unsigned short size) {
  int i;
  MMU_Tab* mmu = NULL;

  if (session_id == 0 || session_id == 1) {
    if (DJI_CMD_Session_Tab[session_id].status == 0) {
      i = session_id;
    }
    else {
      /* session is busy */
      printf("%s: %d: ERROR, session %d is busy\n", __func__, __LINE__, session_id);
      return NULL;
    }
  }
  else {
    for (i = 2; i < SESSION_TABLE_NUM; i++) {
      if (DJI_CMD_Session_Tab[i].status == 0) {
        DJI_CMD_Session_Tab[i].status = 1;
        break;
      }
    }
  }

  if (i < SESSION_TABLE_NUM) {
    mmu = Request_Memory(size);
    if (mmu == NULL) {
      DJI_CMD_Session_Tab[i].status = 0;
    }
    else {
      DJI_CMD_Session_Tab[i].mmu = mmu;
      return &DJI_CMD_Session_Tab[i];
    }
  }

  return NULL;
}

void Free_CMD_Session(CMD_Session_Tab* session) {
  if (session->status == 1) {
    Free_Memory(session->mmu);
    session->status = 0;
  }
}

ACK_Session_Tab* Request_ACK_Session(unsigned short session_id, unsigned short size) {
  if (session_id > 0 && session_id < 32) {
    // directly frees mmu!
    Free_Memory(DJI_ACK_Session_Tab[session_id - 1].mmu);
    MMU_Tab* mmu = Request_Memory(size);
    if (mmu) {
      DJI_ACK_Session_Tab[session_id - 1].mmu = mmu;
      return &DJI_ACK_Session_Tab[session_id - 1];
    }
  }
  return NULL;
}

void Free_ACK_Session(ACK_Session_Tab* session) {
  Free_Memory(session->mmu);
}

void DJI_Pro_Rmu_Setup(void) {
  MMU_Setup();
  Session_Setup();
}
