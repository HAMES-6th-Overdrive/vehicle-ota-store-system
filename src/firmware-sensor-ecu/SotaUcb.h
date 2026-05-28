#ifndef SOTA_UCB_H_
#define SOTA_UCB_H_

#include "Ifx_Types.h"

#define SOTA_OTA_FLAG_ADDR             0xAF000000UL
#define SOTA_OTA_FLAG_MAGIC            0xDEADBEEFUL

#define SOTA_UCB_SWAP_ORIG             0xAF402E00UL
#define SOTA_UCB_SWAP_COPY             0xAF403E00UL

#define SOTA_UCB_SWAP_ENTRY_SIZE       0x10UL
#define SOTA_UCB_SWAP_CONFIRM_OFFSET   0x08UL
#define SOTA_UCB_BLOCK_CONFIRM_OFFSET  0x1F0UL

#define SOTA_UCB_UNLOCKED_CODE         0x43211234UL
#define SOTA_UCB_CONFIRM_CODE          0x57B5327FUL

#define SOTA_SWAP_MARKER_A             0x00000055UL
#define SOTA_SWAP_MARKER_B             0x000000AAUL

#define SOTA_SCU_STMEM1_ADDR           0xF0036184UL
#define SOTA_SCU_STMEM1_SWAP_CFG_MASK  0x00030000UL
#define SOTA_SCU_STMEM1_SWAP_CFG_POS   16U
#define SOTA_SWAP_CFG_B                2U

void Sota_WriteDFlash8(uint32 addr, uint32 lo, uint32 hi);
void Sota_SetPendingUpdateFlag(uint32 firmwareSize, uint32 expectedCrc32);
void Sota_ClearPendingUpdateFlag(void);

boolean Sota_IsInitialized(void);
boolean Sota_IsGroupBActive(void);
void Sota_SwapToGroupA(void);
void Sota_SwapToGroupB(void);

#endif /* SOTA_UCB_H_ */
