#ifndef SOTA_UCB_H
#define SOTA_UCB_H

#include "Ifx_Types.h"

/* UCB_SWAP 주소 (매뉴얼 확인 완료) */
#define UCB_SWAP_ORIG    0xAF402E00UL   /* UCB23 */
#define UCB_SWAP_COPY    0xAF403E00UL   /* UCB31 */
#define UCB_OTP0_ORIG    0xAF404000UL   /* UCB32 — SWAPEN 비트 포함 */
#define UCB_CONFIRM_CODE 0x57B5327FUL
#define SWAP_MARKER_A    0x00000055UL   /* Group A (표준 맵) */
#define SWAP_MARKER_B    0x000000AAUL   /* Group B (교대 맵) */

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

#endif /* SOTA_UCB_H */