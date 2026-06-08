/**
 * @file    MCMCAN.h
 * @brief   Common MCMCAN Interface Header
 *
 * 공용 사용 대상:
 *  - ZCU
 *  - Motor ECU
 *  - Sensor ECU
 *
 * 주의:
 *  - CAN ID / Payload 구조체는 can_type_def.h에 정의
 *  - 이 파일은 공통 API 선언만 담당
 *  - 실제 RX Filter / TX 메시지 / RX 처리 로직은 ECU별 MCMCAN.c에서 구현
 */

#ifndef MCMCAN_H_
#define MCMCAN_H_

#include <stdint.h>
#include <string.h>

#include "Ifx_Types.h"
#include "IfxCan_Can.h"
#include "IfxCan.h"
#include "IfxCpu_Irq.h"
#include "IfxPort.h"

#include "can_type_def.h"


#define CAN_TX_RETRY_COUNT    1000000U

/* ============================================================
   Interrupt Priority
   ============================================================ */

#define ISR_PRIORITY_CAN_TX          2
#define ISR_PRIORITY_CAN_RX          1


/* ============================================================
   CAN Payload Buffer Size
   ============================================================ */

/*
 * IfxCan_Can_sendMessage() / readMessage()는 uint32 배열을 사용.
 *
 * Classical CAN 8 byte  = uint32 2개
 * CAN FD 64 byte        = uint32 16개
 */
#define CAN_CLASSIC_DATA_WORDS       2U
#define CANFD_DATA_WORDS             16U


/* ============================================================
   CAN Node 설정
   TC375 Lite Kit 내장 CAN Transceiver 기준
   ============================================================ */

/*
 * TC375 Lite Kit 기본 CAN Transceiver:
 *  - CAN Node 0
 *  - TX: P20.8
 *  - RX: P20.7
 *  - STB: P20.6
 *
 * 실제 pin 설정은 MCMCAN.c에서 처리
 */
#define MCMCAN_USED_NODE_ID          IfxCan_NodeId_0


/* ============================================================
   CAN TX Result
   ============================================================ */

typedef enum
{
    CAN_TX_OK = 0,
    CAN_TX_BUSY,
    CAN_TX_ERROR
} CanTxResult_t;


/* ============================================================
   MCMCAN Driver Context
   ============================================================ */

/*
 * 이 구조체는 공용으로 둬도 되지만,
 * 실제 g_mcmcan 전역 변수는 각 ECU의 MCMCAN.c에 하나씩 둔다.
 */
typedef struct
{
    IfxCan_Can_Config     canConfig;
    IfxCan_Can            canModule;

    IfxCan_Can_Node       canNode;
    IfxCan_Can_NodeConfig canNodeConfig;

    IfxCan_Filter         canFilter;

    IfxCan_Message        txMsg;
    IfxCan_Message        rxMsg;

    uint32                txData[CANFD_DATA_WORDS];
    uint32                rxData[CANFD_DATA_WORDS];

} McmcanType;


/* ============================================================
   Common API
   ============================================================ */

/**
 * @brief MCMCAN 초기화
 *
 * ECU별 MCMCAN.c에서 구현 내용이 달라짐.
 *
 * ZCU:
 *  - RX Filter: 0x201, 0x202, 0x601
 *
 * Motor ECU:
 *  - RX Filter: 0x080, 0x100
 *
 * Sensor ECU:
 *  - RX Filter: 0x080, 0x600
 */
void initMcmcan(void);


/**
 * @brief Classical CAN 메시지 송신
 *
 * @param id    11-bit Standard CAN ID
 * @param data  송신 데이터 포인터
 * @param dlc   0~8 byte
 *
 * @return CAN_TX_OK / CAN_TX_BUSY / CAN_TX_ERROR
 */
CanTxResult_t CanIf_sendClassic(uint32 id, const uint8_t *data, uint8_t dlc);


/**
 * @brief CAN FD 메시지 송신
 *
 * @param id      11-bit Standard CAN ID
 * @param data    송신 데이터 포인터
 * @param length  0~64 byte
 *
 * @return CAN_TX_OK / CAN_TX_BUSY / CAN_TX_ERROR
 */
CanTxResult_t CanIf_sendFd(uint32 id, const uint8_t *data, uint8_t length);


/**
 * @brief RX 메시지 처리 함수
 *
 * RX ISR 내부에서 호출.
 * ECU별 MCMCAN.c에서 switch-case 내용이 달라짐.
 */
void CanIf_onReceive(uint32 id, const uint8_t *data, uint8_t length);


/* ============================================================
   ZCU 송신용 Helper API
   필요한 ECU에서만 사용
   ============================================================ */

/*
 * 실제 구현:
 *  - ZCU용 MCMCAN.c: 정상 구현
 *  - Sensor ECU / Motor ECU: 필요 없으면 CAN_TX_ERROR stub 가능
 */

CanTxResult_t CanIf_sendVehicleState(const VehicleState_t *msg);
CanTxResult_t CanIf_sendVehicleControlCmd(const VehicleControlCmd_t *msg);


/* ============================================================
   Sensor ECU 송신용 Helper API
   필요한 ECU에서만 사용
   ============================================================ */

/*
 * IMU 센서는 현재 프로젝트에서 사용하지 않으므로
 * CanIf_sendImuData()는 제거한다.
 *
 * Sensor ECU:
 *  - 0x201 TofDistanceData
 *  - 0x202 SpeedData
 */

CanTxResult_t CanIf_sendTofDistanceData(const TofDistanceData_t *msg);
CanTxResult_t CanIf_sendSpeedData(const SpeedData_t *msg);


/* ============================================================
   OTA 송수신용 Helper API
   ============================================================ */

/*
 * ZCU:
 *  - CanIf_sendOtaRequest() 사용
 *
 * Sensor ECU:
 *  - CanIf_sendOtaResponse() 사용
 */

CanTxResult_t CanIf_sendOtaRequest(const uint8_t *payload, uint16_t length);
CanTxResult_t CanIf_sendOtaResponse(const uint8_t *payload, uint16_t length);


#endif /* MCMCAN_H_ */
