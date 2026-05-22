#ifndef APP_CAN_H
#define APP_CAN_H

/**
 * @file App_Can.h
 * @brief FreeRTOS App-level CAN interface for TC375 MCMCAN.
 *
 * 설계 의도:
 *  - AppCan 하나가 TC375 MCMCAN 초기화, raw CAN/CAN FD 송수신, ISO-TP를 담당한다.
 *  - 상위 App은 CAN ID를 직접 알고 AppCan_RecvById() / AppCan_ReadLatestById()로 받는다.
 *  - AppCan은 0x201이 거리인지, 0x600이 UDS인지 같은 의미 해석을 하지 않는다.
 */

#include <stdint.h>

#include "FreeRTOS.h"

#define APP_CAN_CLASSIC_MAX_DLC               (8u)
#define APP_CAN_FD_MAX_DLC                    (64u)

#ifndef APP_CAN_INTERNAL_RX_QUEUE_SIZE
#define APP_CAN_INTERNAL_RX_QUEUE_SIZE        (32u)
#endif

#ifndef APP_CAN_TX_QUEUE_SIZE
#define APP_CAN_TX_QUEUE_SIZE                 (8u)
#endif

#ifndef APP_CAN_MAX_STD_RX_FILTERS
#define APP_CAN_MAX_STD_RX_FILTERS            (8u)
#endif

#ifndef APP_CAN_MAX_RAW_RX_OBJECTS
#define APP_CAN_MAX_RAW_RX_OBJECTS            (8u)
#endif

#ifndef APP_CAN_MAX_TP_CHANNELS
#define APP_CAN_MAX_TP_CHANNELS               (4u)
#endif

#ifndef APP_CAN_TP_MAX_PAYLOAD_SIZE
#define APP_CAN_TP_MAX_PAYLOAD_SIZE           (512u)
#endif

#define APP_CAN_TP_CLASSIC_DLC                (8u)
#define APP_CAN_TP_SINGLE_FRAME_MAX_PAYLOAD   (7u)
#define APP_CAN_TP_FIRST_FRAME_DATA_SIZE      (6u)
#define APP_CAN_TP_CONSEC_FRAME_DATA_SIZE     (7u)
#define APP_CAN_TP_ISO_MAX_PAYLOAD_SIZE       (4095u)

typedef enum AppCanTpRxType {
    APP_CAN_TP_RX_MESSAGE = 0,
    APP_CAN_TP_RX_ERROR
} AppCanTpRxType;

typedef enum AppCanTpError {
    APP_CAN_TP_ERROR_NONE = 0,
    APP_CAN_TP_ERROR_INVALID_PARAM,
    APP_CAN_TP_ERROR_BUSY,
    APP_CAN_TP_ERROR_TX_FAILED,
    APP_CAN_TP_ERROR_RX_OVERFLOW,
    APP_CAN_TP_ERROR_WRONG_SEQUENCE,
    APP_CAN_TP_ERROR_TIMEOUT_BS,
    APP_CAN_TP_ERROR_TIMEOUT_CR,
    APP_CAN_TP_ERROR_FLOW_CONTROL_OVERFLOW,
    APP_CAN_TP_ERROR_UNSUPPORTED_FRAME
} AppCanTpError;

typedef struct AppCanFrame {
    uint32_t id;                              /* Standard 11-bit CAN ID */
    uint8_t  length;                          /* 0..8 Classical, 0..64 CAN FD */
    uint8_t  is_fd;                           /* 0: Classical CAN, 1: CAN FD */
    uint8_t  data[APP_CAN_FD_MAX_DLC];
} AppCanFrame;

typedef struct AppCanTpRxMsg {
    AppCanTpRxType msg_type;
    uint8_t        channel_id;
    AppCanTpError  error;
    uint16_t       length;
    uint8_t        data[APP_CAN_TP_MAX_PAYLOAD_SIZE];
} AppCanTpRxMsg;

BaseType_t AppCan_Start(void);

/* Raw CAN 수신: App_Can.c의 raw RX object table에 등록된 CAN ID만 받을 수 있다. */
BaseType_t AppCan_RecvById(uint32_t can_id, AppCanFrame *frame, TickType_t wait_ticks);
BaseType_t AppCan_ReadLatestById(uint32_t can_id, AppCanFrame *frame);
UBaseType_t AppCan_GetPendingCountById(uint32_t can_id);

BaseType_t AppCan_SendClassic(uint32_t id, const uint8_t *data, uint8_t length);
BaseType_t AppCan_SendFd(uint32_t id, const uint8_t *data, uint8_t length);

/* ISO-TP, Classical CAN normal addressing */
BaseType_t AppCan_TpSend(uint8_t channel_id, const uint8_t *payload, uint16_t length);
BaseType_t AppCan_TpRecv(uint8_t channel_id, AppCanTpRxMsg *msg, TickType_t wait_ticks);
BaseType_t AppCan_TpIsBusy(uint8_t channel_id);

uint32_t AppCan_GetRxQueuedCount(void);
uint32_t AppCan_GetRxDropCount(void);
uint32_t AppCan_GetRxStoredCount(void);
uint32_t AppCan_GetRxNoOwnerCount(void);
uint32_t AppCan_GetTxQueuedCount(void);
uint32_t AppCan_GetTxSentCount(void);
uint32_t AppCan_GetTxBusyCount(void);

#endif /* APP_CAN_H */
