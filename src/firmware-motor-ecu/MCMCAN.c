/**********************************************************************************************************************
 * \file MCMCAN.c
 * \brief Motor ECU MCMCAN driver - Node0, Dedicated RX Buffer version
 *
 * Motor ECU 역할:
 *  - RX:
 *      0x100 VehicleControlCmd  -> RxBuffer0
 *
 *  - TX:
 *      현재 없음
 *
 * 설계 기준:
 *  - Motor ECU는 GearState를 받지 않는다.
 *  - Motor ECU는 CAN 0x080 VehicleState를 사용하지 않는다.
 *  - Motor ECU는 CAN 0x100 VehicleControlCmd만 수신한다.
 *
 * 사용 조건:
 *  - TC375 Lite Kit
 *  - MCMCAN Node0
 *  - CAN_TX: P20.8
 *  - CAN_RX: P20.7
 *  - CAN_STB: P20.6 LOW
 *********************************************************************************************************************/

#include "MCMCAN.h"
#include "IfxCpu.h"
#include <string.h>

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

McmcanType g_mcmcan;

/* ============================================================
   Motor ECU CAN 수신 상태 변수
   Cpu0_Main.c에서 읽어서 자동차 UART 송신에 사용
   ============================================================ */

static volatile VehicleControlCmd_t  g_motorLastCmd;
static volatile boolean              g_motorNewCmdFlag = FALSE;

/*
 * TC375 Lite Kit CAN0 Node0 pin mapping
 * TX: P20.8
 * RX: P20.7
 */
static const IfxCan_Can_Pins g_mcmcanPins =
{
    &IfxCan_TXD00_P20_8_OUT, IfxPort_OutputMode_pushPull,
    &IfxCan_RXD00B_P20_7_IN, IfxPort_InputMode_noPullDevice,
    IfxPort_PadDriver_cmosAutomotiveSpeed1
};


/*********************************************************************************************************************/
/*------------------------------------------------Private prototypes-------------------------------------------------*/
/*********************************************************************************************************************/

static void initCanTransceiver(void);
static void initCanModule(void);
static void initCanNode(void);
static void initMotorRxFilters(void);

static void setStandardFilter(uint8 filterNumber,
                              uint32 canId,
                              IfxCan_RxBufferId rxBufferId);

static void readUpdatedRxBuffers(void);
static void readRxBuffer(IfxCan_RxBufferId rxBufferId);

static uint8 getLengthFromDlc(IfxCan_DataLengthCode dlc);
static void copyWordsToBytes(uint8_t *byteBuffer,
                             const uint32 *wordBuffer,
                             uint16_t length);


/*********************************************************************************************************************/
/*------------------------------------------------Interrupt handlers-------------------------------------------------*/
/*********************************************************************************************************************/

IFX_INTERRUPT(canIsrRxHandler, 0, ISR_PRIORITY_CAN_RX);


/**
 * @brief CAN RX interrupt handler
 *
 * Dedicated RX Buffer 사용:
 *  - RxBuffer0: 0x100 VehicleControlCmd
 */
void canIsrRxHandler(void)
{
    IfxCan_Node_clearInterruptFlag(g_mcmcan.canNode.node,
                                   IfxCan_Interrupt_messageStoredToDedicatedRxBuffer);

    readUpdatedRxBuffers();
}


/*********************************************************************************************************************/
/*------------------------------------------------Public functions---------------------------------------------------*/
/*********************************************************************************************************************/

/**
 * @brief MCMCAN 초기화
 *
 * Motor ECU 기준:
 *  - Node0 사용
 *  - Loopback 사용 안 함
 *  - Dedicated RX Buffer 사용
 *  - RX Filter:
 *      0x100 -> RxBuffer0
 */
void initMcmcan(void)
{
    g_motorLastCmd.driveCmd    = DRIVE_CMD_STOP_VALUE;
    g_motorLastCmd.steeringCmd = STEERING_CMD_CENTER_VALUE;
    g_motorLastCmd.stopCmd     = STOP_CMD_GO;

    g_motorNewCmdFlag = FALSE;

    initCanTransceiver();
    initCanModule();
    initCanNode();
    initMotorRxFilters();
}


/**
 * @brief Motor ECU에서 새 VehicleControlCmd 읽기
 *
 * 새 0x100 메시지가 들어온 경우 TRUE 반환.
 * outCmd에 최신 명령을 복사하고 new flag를 클리어한다.
 */
boolean MotorCan_getLatestControlCmd(VehicleControlCmd_t *outCmd)
{
    boolean interruptState;
    boolean hasNewCmd;

    if(outCmd == NULL_PTR)
    {
        return FALSE;
    }

    interruptState = IfxCpu_disableInterrupts();

    hasNewCmd = g_motorNewCmdFlag;

    if(hasNewCmd == TRUE)
    {
        *outCmd = g_motorLastCmd;
        g_motorNewCmdFlag = FALSE;
    }

    IfxCpu_restoreInterrupts(interruptState);

    return hasNewCmd;
}


/**
 * @brief Motor ECU RX 메시지 처리
 */
void CanIf_onReceive(uint32 id, const uint8_t *data, uint8_t length)
{
    switch(id)
    {
        case CAN_ID_VEHICLE_CONTROL_CMD:
        {
            VehicleControlCmd_Frame_t frame;
            boolean interruptState;

            if((data == NULL_PTR) || (length < CAN_DLC_VEHICLE_CONTROL_CMD))
            {
                break;
            }

            memcpy(frame.raw, data, CAN_DLC_VEHICLE_CONTROL_CMD);

            interruptState = IfxCpu_disableInterrupts();

            g_motorLastCmd = frame.fields;
            g_motorNewCmdFlag = TRUE;

            IfxCpu_restoreInterrupts(interruptState);

            break;
        }

        default:
        {
            break;
        }
    }
}


/* ============================================================
   Motor ECU에서는 현재 CAN 송신을 사용하지 않음.
   공용 MCMCAN.h와 링크 에러 방지를 위해 함수는 정의만 해둔다.
   ============================================================ */

CanTxResult_t CanIf_sendClassic(uint32 id, const uint8_t *data, uint8_t dlc)
{
    (void)id;
    (void)data;
    (void)dlc;

    return CAN_TX_ERROR;
}


CanTxResult_t CanIf_sendFd(uint32 id, const uint8_t *data, uint8_t length)
{
    (void)id;
    (void)data;
    (void)length;

    return CAN_TX_ERROR;
}


CanTxResult_t CanIf_sendVehicleState(const VehicleState_t *msg)
{
    (void)msg;

    return CAN_TX_ERROR;
}


CanTxResult_t CanIf_sendVehicleControlCmd(const VehicleControlCmd_t *msg)
{
    (void)msg;

    return CAN_TX_ERROR;
}


CanTxResult_t CanIf_sendTofDistanceData(const TofDistanceData_t *msg)
{
    (void)msg;

    return CAN_TX_ERROR;
}


CanTxResult_t CanIf_sendSpeedData(const SpeedData_t *msg)
{
    (void)msg;

    return CAN_TX_ERROR;
}


CanTxResult_t CanIf_sendOtaRequest(const uint8_t *payload, uint16_t length)
{
    (void)payload;
    (void)length;

    return CAN_TX_ERROR;
}


CanTxResult_t CanIf_sendOtaResponse(const uint8_t *payload, uint16_t length)
{
    (void)payload;
    (void)length;

    return CAN_TX_ERROR;
}


/*********************************************************************************************************************/
/*------------------------------------------------Private functions--------------------------------------------------*/
/*********************************************************************************************************************/

/**
 * @brief TC375 Lite Kit CAN Transceiver 활성화
 *
 * CAN_STB = P20.6
 * LOW = Normal mode
 */
static void initCanTransceiver(void)
{
    IfxPort_setPinModeOutput(&MODULE_P20,
                             6,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);

    IfxPort_setPinLow(&MODULE_P20, 6);
}


/**
 * @brief CAN Module 초기화
 */
static void initCanModule(void)
{
    IfxCan_Can_initModuleConfig(&g_mcmcan.canConfig, &MODULE_CAN0);
    IfxCan_Can_initModule(&g_mcmcan.canModule, &g_mcmcan.canConfig);
}


/**
 * @brief CAN Node0 초기화
 */
static void initCanNode(void)
{
    IfxCan_Can_initNodeConfig(&g_mcmcan.canNodeConfig,
                              &g_mcmcan.canModule);

    g_mcmcan.canNodeConfig.nodeId = MCMCAN_USED_NODE_ID;

    /*
     * 실제 CAN 버스 사용.
     */
    g_mcmcan.canNodeConfig.busLoopbackEnabled = FALSE;

    /*
     * Node0 하나로 수신.
     * 나중에 Motor ECU가 상태 응답을 보낼 수도 있으므로 transmitAndReceive로 둔다.
     */
    g_mcmcan.canNodeConfig.frame.type = IfxCan_FrameType_transmitAndReceive;

    /*
     * CAN FD 프레임이 같은 버스에 존재할 수 있으므로 fdLong 모드 사용.
     * 실제 Motor ECU가 받는 0x100은 Classical CAN이다.
     */
    g_mcmcan.canNodeConfig.frame.mode = IfxCan_FrameMode_fdLong;

    /*
     * TC375 Lite Kit CAN0 Node0 pins
     */
    g_mcmcan.canNodeConfig.pins = &g_mcmcanPins;

    /*
     * RX Dedicated Buffer 사용.
     * CAN FD 버스 공존까지 고려해서 64 byte buffer로 설정.
     */
    g_mcmcan.canNodeConfig.rxConfig.rxMode = IfxCan_RxMode_dedicatedBuffers;
    g_mcmcan.canNodeConfig.rxConfig.rxBufferDataFieldSize = IfxCan_DataFieldSize_64;

    /*
     * TX는 현재 사용하지 않지만 기본 설정만 둔다.
     */
    g_mcmcan.canNodeConfig.txConfig.txMode = IfxCan_TxMode_dedicatedBuffers;
    g_mcmcan.canNodeConfig.txConfig.txBufferDataFieldSize = IfxCan_DataFieldSize_64;

    /*
     * Standard ID filter 1개 사용:
     * 0x100 VehicleControlCmd
     */
    g_mcmcan.canNodeConfig.filterConfig.messageIdLength = IfxCan_MessageIdLength_standard;
    g_mcmcan.canNodeConfig.filterConfig.standardListSize = 1U;
    g_mcmcan.canNodeConfig.filterConfig.extendedListSize = 0U;

    /*
     * 필터에 없는 Standard ID는 거부.
     */
    g_mcmcan.canNodeConfig.filterConfig.standardFilterForNonMatchingFrames = IfxCan_NonMatchingFrame_reject;
    g_mcmcan.canNodeConfig.filterConfig.extendedFilterForNonMatchingFrames = IfxCan_NonMatchingFrame_reject;
    g_mcmcan.canNodeConfig.filterConfig.rejectRemoteFramesWithStandardId = TRUE;
    g_mcmcan.canNodeConfig.filterConfig.rejectRemoteFramesWithExtendedId = TRUE;

    /*
     * RX Dedicated Buffer interrupt
     */
    g_mcmcan.canNodeConfig.interruptConfig.messageStoredToDedicatedRxBufferEnabled = TRUE;
    g_mcmcan.canNodeConfig.interruptConfig.reint.priority = ISR_PRIORITY_CAN_RX;
    g_mcmcan.canNodeConfig.interruptConfig.reint.interruptLine = IfxCan_InterruptLine_1;
    g_mcmcan.canNodeConfig.interruptConfig.reint.typeOfService = IfxSrc_Tos_cpu0;

    IfxCan_Can_initNode(&g_mcmcan.canNode,
                        &g_mcmcan.canNodeConfig);
}


/**
 * @brief Motor ECU RX Filter 설정
 *
 * 0x100 VehicleControlCmd -> RxBuffer0
 */
static void initMotorRxFilters(void)
{
    setStandardFilter(0U, CAN_ID_VEHICLE_CONTROL_CMD, IfxCan_RxBufferId_0);
}


/**
 * @brief Standard ID filter 설정
 */
static void setStandardFilter(uint8 filterNumber,
                              uint32 canId,
                              IfxCan_RxBufferId rxBufferId)
{
    g_mcmcan.canFilter.number = filterNumber;
    g_mcmcan.canFilter.elementConfiguration =
        IfxCan_FilterElementConfiguration_storeInRxBuffer;

    g_mcmcan.canFilter.type = IfxCan_FilterType_none;
    g_mcmcan.canFilter.id1 = canId;
    g_mcmcan.canFilter.id2 = 0U;
    g_mcmcan.canFilter.rxBufferOffset = rxBufferId;

    IfxCan_Can_setStandardFilter(&g_mcmcan.canNode,
                                 &g_mcmcan.canFilter);
}


/**
 * @brief 새 데이터가 들어온 RX Buffer들을 모두 읽음
 */
static void readUpdatedRxBuffers(void)
{
    if(IfxCan_Node_isRxBufferNewDataUpdated(g_mcmcan.canNode.node,
                                            IfxCan_RxBufferId_0))
    {
        readRxBuffer(IfxCan_RxBufferId_0);
    }
}


/**
 * @brief 특정 Dedicated RX Buffer에서 메시지 읽기
 */
static void readRxBuffer(IfxCan_RxBufferId rxBufferId)
{
    uint8_t rxBytes[CANFD_MAX_DLC];
    uint8_t length;

    IfxCan_Can_initMessage(&g_mcmcan.rxMsg);
    memset(g_mcmcan.rxData, 0, sizeof(g_mcmcan.rxData));
    memset(rxBytes, 0, sizeof(rxBytes));

    /*
     * Dedicated RX Buffer에서 읽기.
     */
    g_mcmcan.rxMsg.readFromRxFifo0 = FALSE;
    g_mcmcan.rxMsg.readFromRxFifo1 = FALSE;
    g_mcmcan.rxMsg.bufferNumber = (uint8)rxBufferId;

    IfxCan_Can_readMessage(&g_mcmcan.canNode,
                           &g_mcmcan.rxMsg,
                           g_mcmcan.rxData);

    length = getLengthFromDlc(g_mcmcan.rxMsg.dataLengthCode);

    if(length > CANFD_MAX_DLC)
    {
        length = CANFD_MAX_DLC;
    }

    copyWordsToBytes(rxBytes, g_mcmcan.rxData, length);

    CanIf_onReceive(g_mcmcan.rxMsg.messageId,
                    rxBytes,
                    length);
}


/**
 * @brief DLC -> 실제 payload length 변환
 */
static uint8 getLengthFromDlc(IfxCan_DataLengthCode dlc)
{
    switch(dlc)
    {
        case IfxCan_DataLengthCode_0:  return 0U;
        case IfxCan_DataLengthCode_1:  return 1U;
        case IfxCan_DataLengthCode_2:  return 2U;
        case IfxCan_DataLengthCode_3:  return 3U;
        case IfxCan_DataLengthCode_4:  return 4U;
        case IfxCan_DataLengthCode_5:  return 5U;
        case IfxCan_DataLengthCode_6:  return 6U;
        case IfxCan_DataLengthCode_7:  return 7U;
        case IfxCan_DataLengthCode_8:  return 8U;
        case IfxCan_DataLengthCode_12: return 12U;
        case IfxCan_DataLengthCode_16: return 16U;
        case IfxCan_DataLengthCode_20: return 20U;
        case IfxCan_DataLengthCode_24: return 24U;
        case IfxCan_DataLengthCode_32: return 32U;
        case IfxCan_DataLengthCode_48: return 48U;
        case IfxCan_DataLengthCode_64: return 64U;
        default:                       return 0U;
    }
}


/**
 * @brief uint32 word buffer -> byte buffer
 */
static void copyWordsToBytes(uint8_t *byteBuffer,
                             const uint32 *wordBuffer,
                             uint16_t length)
{
    memcpy(byteBuffer, (const uint8_t *)wordBuffer, length);
}
