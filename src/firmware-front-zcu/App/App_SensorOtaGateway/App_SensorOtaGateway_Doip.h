#ifndef APP_SENSOR_OTA_GATEWAY_DOIP_H_
#define APP_SENSOR_OTA_GATEWAY_DOIP_H_

/**********************************************************************************************************************
 * \file App_SensorOtaGateway_Doip.h
 * \brief Sensor ECU OTA Gateway DoIP server
 *
 * 역할:
 *  - Pi/HPC가 TCP 13400으로 보내는 DoIP 메시지를 수신한다.
 *  - Routing Activation을 처리한다.
 *  - Diagnostic Message 안의 UDS payload를 App_SensorOtaGateway_Uds로 전달한다.
 *
 * 구조:
 *  Pi/HPC
 *      ↓ TCP / DoIP
 *  App_SensorOtaGateway_Doip
 *      ↓ UDS payload
 *  App_SensorOtaGateway_Uds
 *      ↓
 *  App_OtaReceiver
 *      ↓
 *  App_OtaGateway
 *      ↓ CAN FD 0x600 / 0x601
 *  Sensor ECU
 *
 * 주의:
 *  - 이 모듈은 ZCU Local OTA용 DoIP가 아니다.
 *  - Sensor ECU OTA 중계용 DoIP 입력부이다.
 *********************************************************************************************************************/

#include "Ifx_Types.h"

/* ============================================================
   DoIP config
   ============================================================ */

#define APP_SENSOR_OTA_GATEWAY_DOIP_PORT                 13401U
#define APP_SENSOR_OTA_GATEWAY_DOIP_HEADER_LEN           8U
#define APP_SENSOR_OTA_GATEWAY_DOIP_PROTOCOL_VERSION     0x02U
#define APP_SENSOR_OTA_GATEWAY_DOIP_RX_BUF_SIZE          4096U

/* ============================================================
   DoIP Payload Types
   ============================================================ */

#define APP_SENSOR_OTA_GATEWAY_DOIP_ROUTING_ACT_REQ      0x0005U
#define APP_SENSOR_OTA_GATEWAY_DOIP_ROUTING_ACT_RES      0x0006U
#define APP_SENSOR_OTA_GATEWAY_DOIP_DIAG_MESSAGE         0x8001U
#define APP_SENSOR_OTA_GATEWAY_DOIP_DIAG_MESSAGE_ACK     0x8002U

/* ============================================================
   DoIP Logical Addresses
   ============================================================ */

#define APP_SENSOR_OTA_GATEWAY_DOIP_TESTER_ADDR          0x0E00U  /* Pi / HPC */
#define APP_SENSOR_OTA_GATEWAY_DOIP_ZCU_ADDR             0x0001U  /* ZCU */

/* ============================================================
   DoIP state
   ============================================================ */

typedef enum
{
    APP_SENSOR_OTA_GATEWAY_DOIP_STATE_NONE = 0,
    APP_SENSOR_OTA_GATEWAY_DOIP_STATE_ACCEPTED,
    APP_SENSOR_OTA_GATEWAY_DOIP_STATE_ROUTING_ACTIVE,
    APP_SENSOR_OTA_GATEWAY_DOIP_STATE_CLOSING
} AppSensorOtaGatewayDoip_State_t;

/* ============================================================
   lwIP forward declaration
   ============================================================ */

typedef struct tcp_pcb tcpPcb2;
typedef struct pbuf    pBuf2;

/* ============================================================
   DoIP session
   ============================================================ */

typedef struct
{
    uint8    state;
    tcpPcb2 *pcb;
    pBuf2   *p;

    uint8   rxBuf[APP_SENSOR_OTA_GATEWAY_DOIP_RX_BUF_SIZE];
    uint16  rxLen;
} AppSensorOtaGatewayDoip_Session_t;

/* ============================================================
   Public API
   ============================================================ */

/**
 * @brief Sensor ECU OTA Gateway DoIP 서버 초기화
 *
 * 내부 동작:
 *  - TCP 13400 listen 준비
 *  - Routing Activation / Diagnostic Message 수신 준비
 *
 * 주의:
 *  - Ethernet/lwIP 초기화 이후 호출해야 한다.
 */
void AppSensorOtaGatewayDoip_Init(void);
void AppSensorOtaGatewayDoip_MainFunction(void);

boolean AppSensorOtaGatewayDoip_IsReady(void);

#endif /* APP_SENSOR_OTA_GATEWAY_DOIP_H_ */
