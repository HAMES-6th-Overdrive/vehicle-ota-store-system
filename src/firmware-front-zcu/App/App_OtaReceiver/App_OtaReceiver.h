#ifndef APP_OTA_RECEIVER_H_
#define APP_OTA_RECEIVER_H_

/**********************************************************************************************************************
 * \file App_OtaReceiver.h
 * \brief ZCU OTA input adapter layer
 *
 * 역할:
 *  - Pi/HPC, UART, SOME/IP, DoIP 등 외부 입력 계층과 App_OtaGateway 사이의 얇은 중간 계층
 *  - 실제 통신 파싱은 하지 않고, OTA_START / OTA_BLOCK / OTA_FINAL_CRC API만 제공한다.
 *
 * 구조:
 *  External Input
 *      ↓
 *  App_OtaReceiver
 *      ↓
 *  App_OtaGateway
 *      ↓
 *  OtaGateway / UdsOtaClient
 *      ↓
 *  Sensor ECU
 *
 * 현재 단계:
 *  - Download/Verify phase만 담당
 *  - SOTA/UCB_SWAP activation은 아직 수행하지 않는다.
 *
 * CRC 모드:
 *  1. CRC known mode
 *     - AppOtaReceiver_StartDownload(size, crc32)
 *
 *  2. Late CRC mode
 *     - AppOtaReceiver_StartDownloadWithoutCrc(size)
 *     - 모든 block 전송 후 WAIT_FINAL_CRC 상태에서 대기
 *     - AppOtaReceiver_SetFinalCrc(crc32) 호출 시 Sensor ECU 쪽 CRC 검증 진행
 *********************************************************************************************************************/

#include "Ifx_Types.h"
#include "FreeRTOS.h"

#include <stdint.h>

/* ============================================================
   Public API
   ============================================================ */

/**
 * @brief OTA Receiver 초기화
 *
 * 현재는 debug 변수 초기화만 수행한다.
 * App_OtaGateway_Start() 이후에 호출하는 것을 권장한다.
 */
void AppOtaReceiver_Init(void);

/**
 * @brief 외부 입력 계층에서 OTA_START를 받았을 때 호출 - CRC known mode
 *
 * 시작 시점에 firmwareSize와 firmwareCrc32를 모두 알고 있는 경우 사용한다.
 *
 * @param firmwareSize  전체 firmware binary size
 * @param firmwareCrc32 전체 firmware CRC32
 * @param waitTicks     queue send 대기 tick
 *
 * @return pdPASS / pdFAIL
 */
BaseType_t AppOtaReceiver_StartDownload(uint32_t firmwareSize,
                                        uint32_t firmwareCrc32,
                                        TickType_t waitTicks);

/**
 * @brief 외부 입력 계층에서 OTA_START를 받았을 때 호출 - Late CRC mode
 *
 * Pi/HPC -> ZCU DoIP 흐름처럼 CRC32가 마지막 0x37 RequestTransferExit 단계에서 들어오는 경우 사용한다.
 *
 * 이 함수는 firmwareSize만으로 Sensor ECU download를 시작한다.
 * 모든 block 전송 완료 후 WAIT_FINAL_CRC 상태에서 대기한다.
 * 이후 AppOtaReceiver_SetFinalCrc()가 호출되면 Sensor ECU 쪽 CRC 검증이 진행된다.
 *
 * @param firmwareSize 전체 firmware binary size
 * @param waitTicks    queue send 대기 tick
 *
 * @return pdPASS / pdFAIL
 */
BaseType_t AppOtaReceiver_StartDownloadWithoutCrc(uint32_t firmwareSize,
                                                  TickType_t waitTicks);

/**
 * @brief Late CRC mode에서 최종 CRC32 설정
 *
 * Pi/HPC -> ZCU DoIP 흐름에서 0x37 RequestTransferExit 단계에 CRC32가 들어오면 호출한다.
 *
 * 호출 전 확인 권장:
 *  - AppOtaReceiver_IsWaitingFinalCrc() == TRUE
 *
 * @param firmwareCrc32 전체 firmware CRC32
 * @param waitTicks     queue send 대기 tick
 *
 * @return pdPASS / pdFAIL
 */
BaseType_t AppOtaReceiver_SetFinalCrc(uint32_t firmwareCrc32,
                                      TickType_t waitTicks);

/**
 * @brief 외부 입력 계층에서 OTA_BLOCK을 받았을 때 호출
 *
 * @param blockIndex 제공할 block index
 * @param data       block data pointer
 * @param length     block length. 보통 32 bytes, 마지막 block은 32보다 작을 수 있음
 * @param waitTicks  queue send 대기 tick
 *
 * @return pdPASS / pdFAIL
 */
BaseType_t AppOtaReceiver_ProvideBlock(uint32_t blockIndex,
                                       const uint8_t *data,
                                       uint8_t length,
                                       TickType_t waitTicks);

/**
 * @brief OTA 취소 요청
 *
 * @param waitTicks queue send 대기 tick
 *
 * @return pdPASS / pdFAIL
 */
BaseType_t AppOtaReceiver_Cancel(TickType_t waitTicks);

BaseType_t AppOtaReceiver_RequestSensorEcuReset(TickType_t waitTicks);

/* ============================================================
   Gateway state helper
   ============================================================ */

boolean AppOtaReceiver_IsBusy(void);
boolean AppOtaReceiver_IsWaitingBlock(void);
boolean AppOtaReceiver_IsWaitingFinalCrc(void);
boolean AppOtaReceiver_IsDone(void);
boolean AppOtaReceiver_IsError(void);

uint32_t AppOtaReceiver_GetRequestedBlockIndex(void);
uint32_t AppOtaReceiver_GetRequestedOffset(void);
uint8_t  AppOtaReceiver_GetRequestedLength(void);
uint8_t  AppOtaReceiver_GetProgress(void);

#endif /* APP_OTA_RECEIVER_H_ */
