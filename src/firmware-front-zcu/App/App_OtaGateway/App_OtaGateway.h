#ifndef APP_OTA_GATEWAY_H_
#define APP_OTA_GATEWAY_H_

/**********************************************************************************************************************
 * \file App_OtaGateway.h
 * \brief FreeRTOS wrapper for ZCU OTA Gateway
 *
 * 역할:
 *  - OtaGateway/UdsOtaClient를 FreeRTOS task에서 주기적으로 실행한다.
 *  - App_Can에 저장된 0x601 UDS Response를 꺼내 UdsOtaClient_OnResponse()로 전달한다.
 *  - Pi/HPC/DoIP 계층에서 들어온 OTA_START / OTA_BLOCK / OTA_FINISH 요청을 command queue로 받아 처리한다.
 *
 * 현재 단계:
 *  - Download/Verify phase 담당
 *  - ZCU는 전체 firmware binary를 저장하지 않는다.
 *  - OTA_BLOCK에서는 현재 요청된 32-byte block만 받는다.
 *  - SOTA/UCB_SWAP activation은 아직 수행하지 않는다.
 *
 * CRC 모드:
 *  1. CRC known mode
 *     - AppOtaGateway_RequestDownload(size, crc32)
 *
 *  2. Late CRC mode
 *     - AppOtaGateway_RequestDownloadWithoutCrc(size)
 *     - 모든 block 전송 후 WAIT_FINAL_CRC 상태에서 대기
 *     - AppOtaGateway_SetFinalCrc(crc32) 호출 시 Sensor ECU 쪽 CRC 검증 진행
 *********************************************************************************************************************/

#include "Ifx_Types.h"
#include "FreeRTOS.h"

#include <stdint.h>

/* ============================================================
   Public API
   ============================================================ */

/**
 * @brief App_OtaGateway FreeRTOS task 시작
 *
 * 내부 동작:
 *  - OtaGateway_Init() 호출
 *  - command queue 생성
 *  - 1ms 주기 task 생성
 *  - task 안에서 0x601 응답 처리 + command 처리 + OtaGateway_MainFunction() 호출
 *
 * 주의:
 *  - AppCan_Start() 이후에 호출해야 한다.
 */
BaseType_t AppOtaGateway_Start(void);


/* ============================================================
   OTA command API
   ============================================================ */

/**
 * @brief OTA Download 시작 요청 - CRC known mode
 *
 * 상위 Pi/HPC/App 계층에서 firmware size와 CRC32를 시작 시점에 알고 있으면 호출한다.
 *
 * @param firmwareSize  전체 firmware binary size
 * @param firmwareCrc32 전체 firmware CRC32
 * @param waitTicks     queue send 대기 tick
 *
 * @return pdPASS / pdFAIL
 */
BaseType_t AppOtaGateway_RequestDownload(uint32_t firmwareSize,
                                         uint32_t firmwareCrc32,
                                         TickType_t waitTicks);


/**
 * @brief OTA Download 시작 요청 - Late CRC mode
 *
 * Pi/HPC -> ZCU DoIP 흐름처럼 CRC32가 마지막 0x37에서 들어오는 경우 사용한다.
 *
 * 이 함수는 firmwareSize만으로 download를 시작한다.
 * 모든 block 전송 후 App_OtaGateway는 WAIT_FINAL_CRC 상태가 된다.
 * 이후 AppOtaGateway_SetFinalCrc()가 호출되면 Sensor ECU 쪽 CRC 검증을 진행한다.
 *
 * @param firmwareSize 전체 firmware binary size
 * @param waitTicks    queue send 대기 tick
 *
 * @return pdPASS / pdFAIL
 */
BaseType_t AppOtaGateway_RequestDownloadWithoutCrc(uint32_t firmwareSize,
                                                   TickType_t waitTicks);


/**
 * @brief Late CRC mode에서 최종 CRC32 설정
 *
 * Pi/HPC -> ZCU DoIP 흐름에서 0x37 RequestTransferExit 단계에 CRC32가 들어오면 호출한다.
 *
 * 호출 전 확인 권장:
 *  - AppOtaGateway_IsWaitingFinalCrc() == TRUE
 *
 * @param firmwareCrc32 전체 firmware CRC32
 * @param waitTicks     queue send 대기 tick
 *
 * @return pdPASS / pdFAIL
 */
BaseType_t AppOtaGateway_SetFinalCrc(uint32_t firmwareCrc32,
                                     TickType_t waitTicks);


/**
 * @brief 현재 요청된 firmware block 제공
 *
 * 상위 Pi/HPC/App 계층에서 block data를 받으면 호출한다.
 *
 * 호출 전 확인 권장:
 *  - AppOtaGateway_IsWaitingBlock() == TRUE
 *  - blockIndex == AppOtaGateway_GetRequestedBlockIndex()
 *  - length == AppOtaGateway_GetRequestedLength()
 *
 * @param blockIndex 제공할 block index
 * @param data       block data pointer
 * @param length     block length. 보통 32 bytes, 마지막 block은 32보다 작을 수 있음
 * @param waitTicks  queue send 대기 tick
 *
 * @return pdPASS / pdFAIL
 */
BaseType_t AppOtaGateway_ProvideBlock(uint32_t blockIndex,
                                      const uint8_t *data,
                                      uint8_t length,
                                      TickType_t waitTicks);


/**
 * @brief 진행 중인 OTA Download 취소
 *
 * @param waitTicks queue send 대기 tick
 *
 * @return pdPASS / pdFAIL
 */
BaseType_t AppOtaGateway_Cancel(TickType_t waitTicks);

//0x11 단계에서 ECU Reset 요청 (jump to app)
BaseType_t AppOtaGateway_RequestSensorEcuReset(TickType_t waitTicks);

/* ============================================================
   상태 확인 API
   ============================================================ */

boolean AppOtaGateway_IsBusy(void);
boolean AppOtaGateway_IsWaitingBlock(void);
boolean AppOtaGateway_IsWaitingFinalCrc(void);
boolean AppOtaGateway_IsDone(void);
boolean AppOtaGateway_IsError(void);


/* ============================================================
   요청 block 정보 확인 API
   ============================================================ */

uint32_t AppOtaGateway_GetRequestedBlockIndex(void);
uint32_t AppOtaGateway_GetRequestedOffset(void);
uint8_t  AppOtaGateway_GetRequestedLength(void);


/* ============================================================
   진행률 확인 API
   ============================================================ */

uint8_t AppOtaGateway_GetProgress(void);

#endif /* APP_OTA_GATEWAY_H_ */
