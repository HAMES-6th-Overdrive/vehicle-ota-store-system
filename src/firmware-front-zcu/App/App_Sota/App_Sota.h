#ifndef APP_SOTA_H
#define APP_SOTA_H

#include "Ifx_Types.h"

/* UCB_SWAP 주소 */
#define UCB_SWAP_ORIG                 0xAF402E00UL   /* UCB23 */
#define UCB_SWAP_COPY                 0xAF403E00UL   /* UCB31 */
#define UCB_OTP0_ORIG                 0xAF404000UL   /* UCB32 — SWAPEN 비트 포함 */

/* UCB_SWAP entry layout */
#define UCB_SWAP_SIZE                 0x200UL
#define UCB_SWAP_ENTRY_SIZE           0x10UL
#define UCB_SWAP_ENTRY_CONFIRM_OFFSET 0x08UL

/* UCB block confirmation layout */
#define UCB_BLOCK_CONFIRM_OFFSET      0x1F0UL
#define UCB_STATE_UNLOCKED_CODE       0x43211234UL   /* 요청사항: 무조건 UNLOCKED 상태 */
#define UCB_CONFIRM_CODE              0x57B5327FUL   /* SWAP entry confirmation */

/* SOTA marker */
#define SWAP_MARKER_A                 0x00000055UL   /* Group A (표준 맵) */
#define SWAP_MARKER_B                 0x000000AAUL   /* Group B (교대 맵) */

/* SCU_STMEM1.SWAP_CFG: SSW가 이번 부팅에 실제 적용한 SOTA map 상태 */
#define SCU_STMEM1_ADDR               0xF0036184UL
#define SCU_STMEM1_SWAP_CFG_MASK      0x00030000UL
#define SCU_STMEM1_SWAP_CFG_POS       16U
#define SOTA_SWAP_CFG_NONE            0U
#define SOTA_SWAP_CFG_A               1U
#define SOTA_SWAP_CFG_B               2U
#define SOTA_SWAP_CFG_RESERVED        3U

/* DFLASH 8바이트 쓰기 (공유) */
void WriteDFlash8(uint32 addr, uint32 lo, uint32 hi);

/* 공개 인터페이스 */
void    SOTA_EnableSwapen  (void);
void    SOTA_InitialSetup  (void);
void    SOTA_SwapToGroupB  (void);
void    SOTA_SwapToGroupA  (void);
boolean SOTA_IsInitialized (void);
boolean SOTA_IsGroupBActive(void);

#endif /* APP_SOTA_H */
