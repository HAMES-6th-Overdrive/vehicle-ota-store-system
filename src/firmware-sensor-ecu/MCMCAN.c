/**********************************************************************************************************************
 * \file MCMCAN.c
 * \brief Sensor ECU MCMCAN driver - Node0(ZCU) + Node2(TOF CAN), separated Message RAM version
 *
 * Sensor ECU 역할:
 *  - Node0 RX:
 *      0x080 VehicleState  -> RxBuffer0
 *      0x600 OtaRequest    -> RxBuffer1
 *
 *  - Node0 TX:
 *      0x200 ImuData
 *      0x201 TofDistanceData
 *      0x202 SpeedData
 *      0x601 OtaResponse
 *
 *  - Node2 RX:
 *      TOF Sensor CAN frame -> RX FIFO0
 *      수신 frame은 TofSensor_onCanFrame(id, data, length)로 전달
 *
 * 중요:
 *  - MODULE_CAN0은 Node0과 Node2가 공유한다.
 *  - 따라서 Node0/Node2의 Message RAM 영역을 반드시 분리해야 한다.
 *********************************************************************************************************************/

#include "MCMCAN.h"
#include "IfxCpu.h"
#include "TofSensor.h"

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

McmcanType g_mcmcan;


/* ============================================================
   Node0 RX 디버깅용 변수
   ============================================================ */

volatile uint32_t testLastRxId = 0;

volatile uint8_t testGearState = GEAR_STATE_P;

/*
 * Sensor ECU: Gear P -> 센서값 읽기 X
 * Gear D일 때만 센서 송신 허용
 */
volatile boolean testSensorReadEnabled = FALSE;

/* OTA Request 확인용 */
volatile uint8_t  testOtaServiceId      = 0;
volatile uint32_t testOtaFirmwareSize   = 0;
volatile uint32_t testOtaFirmwareCrc32  = 0;
volatile uint16_t testOtaBlockSequence  = 0;
volatile uint16_t testOtaDataLength     = 0;
volatile uint32_t testOtaTotalCrc32     = 0;
volatile uint8_t  testOtaApplyRequest   = 0;
volatile uint8_t  testOtaResetType      = 0;


/* ============================================================
   Node0 RX ISR 확인용
   ============================================================ */

volatile uint32_t testNode0RxIsrCount = 0U;


/* ============================================================
   CAN TX Queue
   - Node0 송신용
   - busy-wait 없이 queue에 저장
   - TX complete ISR에서 다음 메시지 송신
   ============================================================ */

#define CAN_TX_QUEUE_SIZE      8U

typedef struct
{
    uint32  id;
    uint8   length;
    boolean isFd;
    uint8_t data[CANFD_MAX_DLC];
} CanTxQueueItem_t;

static CanTxQueueItem_t g_txQueue[CAN_TX_QUEUE_SIZE];

static volatile uint8   g_txQueueHead  = 0U;
static volatile uint8   g_txQueueTail  = 0U;
static volatile uint8   g_txQueueCount = 0U;
static volatile boolean g_txInProgress = FALSE;

/* TX Queue 디버깅용 */
volatile uint32_t testTxQueuedCount = 0U;
volatile uint32_t testTxSentCount   = 0U;
volatile uint32_t testTxBusyCount   = 0U;
volatile uint32_t testTxDropCount   = 0U;


/* ============================================================
   Node2 TOF CAN 설정
   ============================================================ */

#define ISR_PRIORITY_TOF_CAN_RX      4U
#define TOF_CAN_BAUDRATE             500000U
#define TOF_CAN_RX_FIFO0_SIZE        15U

static IfxCan_Can_Node       g_tofCanNode2;
static IfxCan_Can_NodeConfig g_tofCanNode2Config;
static IfxCan_Filter         g_tofCanFilter;
static IfxCan_Message        g_tofRxMsg;
static uint32                g_tofRxData[CAN_CLASSIC_DATA_WORDS];

/* TOF Node2 디버깅용 */
volatile uint32_t testTofCanRxId     = 0U;
volatile uint8_t  testTofCanRxLength = 0U;
volatile uint32_t testTofCanRxCount  = 0U;
volatile uint8_t  testTofRawBytes[8] = {0};


/* ============================================================
   Node0 pin mapping
   ZCU 통신용
   TX: P20.8
   RX: P20.7
   ============================================================ */

static const IfxCan_Can_Pins g_mcmcanNode0Pins =
{
    &IfxCan_TXD00_P20_8_OUT, IfxPort_OutputMode_pushPull,
    &IfxCan_RXD00B_P20_7_IN, IfxPort_InputMode_noPullDevice,
    IfxPort_PadDriver_cmosAutomotiveSpeed1
};


/* ============================================================
   Node2 pin mapping
   TOF 센서 CAN 수신용
   TX: P15.0
   RX: P15.1
   ============================================================ */

static const IfxCan_Can_Pins g_tofCanNode2Pins =
{
    &IfxCan_TXD02_P15_0_OUT, IfxPort_OutputMode_pushPull,
    &IfxCan_RXD02A_P15_1_IN, IfxPort_InputMode_pullUp,
    IfxPort_PadDriver_cmosAutomotiveSpeed1
};


/*********************************************************************************************************************/
/*------------------------------------------------Private prototypes-------------------------------------------------*/
/*********************************************************************************************************************/

static void initCanTransceiver(void);
static void initCanModule(void);

static void initCanNode0(void);
static void initSensorNode0RxFilters(void);

static void initTofCanNode2(void);
static void initTofCanNode2Filter(void);

static void setNode0MessageRamLayout(void);
static void setNode2MessageRamLayout(void);

static void setNode0StandardFilter(uint8 filterNumber, uint32 canId, IfxCan_RxBufferId rxBufferId);

static void readNode0UpdatedRxBuffers(void);
static void readNode0RxBuffer(IfxCan_RxBufferId rxBufferId);

static void readTofCanNode2Fifo0(void);

static CanTxResult_t enqueueTxMessage(uint32 id,
                                      const uint8_t *data,
                                      uint8 length,
                                      boolean isFd);

static void tryStartNextTx(void);
static void prepareTxMessage(const CanTxQueueItem_t *item);

static IfxCan_DataLengthCode getClassicDlc(uint8 dlc);
static IfxCan_DataLengthCode getCanFdDlc(uint8 length);
static uint8 getLengthFromDlc(IfxCan_DataLengthCode dlc);

static void copyBytesToWords(uint32 *wordBuffer, const uint8_t *byteBuffer, uint16_t length);
static void copyWordsToBytes(uint8_t *byteBuffer, const uint32 *wordBuffer, uint16_t length);


/*********************************************************************************************************************/
/*------------------------------------------------Interrupt handlers-------------------------------------------------*/
/*********************************************************************************************************************/

IFX_INTERRUPT(canIsrTxHandler, 0, ISR_PRIORITY_CAN_TX);
IFX_INTERRUPT(canIsrRxHandler, 0, ISR_PRIORITY_CAN_RX);
IFX_INTERRUPT(tofCanNode2RxIsrHandler, 0, ISR_PRIORITY_TOF_CAN_RX);


/**
 * @brief Node0 CAN TX interrupt handler
 */
void canIsrTxHandler(void)
{
    IfxCan_Node_clearInterruptFlag(g_mcmcan.canNode.node,
                                   IfxCan_Interrupt_transmissionCompleted);

    g_txInProgress = FALSE;
    testTxSentCount++;

    tryStartNextTx();
}


/**
 * @brief Node0 CAN RX interrupt handler
 *
 * Dedicated RX Buffer 사용:
 *  - RxBuffer0: 0x080 VehicleState
 *  - RxBuffer1: 0x600 OtaRequest
 */
void canIsrRxHandler(void)
{
    testNode0RxIsrCount++;

    IfxCan_Node_clearInterruptFlag(g_mcmcan.canNode.node,
                                   IfxCan_Interrupt_messageStoredToDedicatedRxBuffer);

    readNode0UpdatedRxBuffers();
}


/**
 * @brief Node2 TOF CAN RX interrupt handler
 *
 * TOF 센서 CAN frame을 Node2 RX FIFO0에서 읽고,
 * TofSensor_onCanFrame()으로 전달한다.
 */
void tofCanNode2RxIsrHandler(void)
{
    IfxCan_Node_clearInterruptFlag(g_tofCanNode2.node,
                                   IfxCan_Interrupt_rxFifo0NewMessage);

    readTofCanNode2Fifo0();
}


/*********************************************************************************************************************/
/*------------------------------------------------Public functions---------------------------------------------------*/
/*********************************************************************************************************************/

/**
 * @brief MCMCAN 초기화
 *
 * 순서:
 *  1. CAN transceiver 활성화
 *  2. CAN module 초기화
 *  3. TOF sensor parser 초기화
 *  4. Node0 초기화 + Node0 filter 설정
 *  5. Node2 초기화 + Node2 filter 설정
 */
void initMcmcan(void)
{
    initCanTransceiver();
    initCanModule();

    TofSensor_init();

    /*
     * Node0: ZCU 통신용
     */
    initCanNode0();
    initSensorNode0RxFilters();

    /*
     * Node2: TOF 센서 CAN 수신용
     */
    initTofCanNode2();
    initTofCanNode2Filter();

    /*
     * 중요:
     * 만약 여기까지 했는데 Node0 0x080 수신이 또 죽으면,
     * 아래 줄을 임시로 한 번 더 호출해서 필터 충돌 여부를 확인한다.
     *
     * initSensorNode0RxFilters();
     */
}


/**
 * @brief Classical CAN 메시지 송신 요청
 */
CanTxResult_t CanIf_sendClassic(uint32 id, const uint8_t *data, uint8_t dlc)
{
    if((data == NULL_PTR) || (dlc > CAN_CLASSIC_MAX_DLC))
    {
        return CAN_TX_ERROR;
    }

    return enqueueTxMessage(id, data, dlc, FALSE);
}


/**
 * @brief CAN FD 메시지 송신 요청
 */
CanTxResult_t CanIf_sendFd(uint32 id, const uint8_t *data, uint8_t length)
{
    if((data == NULL_PTR) || (length > CANFD_MAX_DLC))
    {
        return CAN_TX_ERROR;
    }

    return enqueueTxMessage(id, data, length, TRUE);
}


/**
 * @brief Sensor ECU 송신: 0x200 ImuData
 */
CanTxResult_t CanIf_sendImuData(const ImuData_t *msg)
{
    ImuData_Frame_t frame;

    if(msg == NULL_PTR)
    {
        return CAN_TX_ERROR;
    }

    frame.fields = *msg;

    return CanIf_sendClassic(CAN_ID_IMU_DATA,
                             frame.raw,
                             CAN_DLC_IMU_DATA);
}


/**
 * @brief Sensor ECU 송신: 0x201 TofDistanceData
 */
CanTxResult_t CanIf_sendTofDistanceData(const TofDistanceData_t *msg)
{
    TofDistanceData_Frame_t frame;

    if(msg == NULL_PTR)
    {
        return CAN_TX_ERROR;
    }

    frame.fields = *msg;

    return CanIf_sendClassic(CAN_ID_TOF_DISTANCE_DATA,
                             frame.raw,
                             CAN_DLC_TOF_DISTANCE_DATA);
}


/**
 * @brief Sensor ECU 송신: 0x202 SpeedData
 */
CanTxResult_t CanIf_sendSpeedData(const SpeedData_t *msg)
{
    SpeedData_Frame_t frame;

    if(msg == NULL_PTR)
    {
        return CAN_TX_ERROR;
    }

    frame.fields = *msg;

    return CanIf_sendClassic(CAN_ID_SPEED_DATA,
                             frame.raw,
                             CAN_DLC_SPEED_DATA);
}


/**
 * @brief Sensor ECU 송신: 0x601 OtaResponse
 */
CanTxResult_t CanIf_sendOtaResponse(const uint8_t *payload, uint16_t length)
{
    if((payload == NULL_PTR) || (length > CANFD_MAX_DLC))
    {
        return CAN_TX_ERROR;
    }

    return CanIf_sendFd(CAN_ID_OTA_RESPONSE, payload, (uint8_t)length);
}


/**
 * @brief Sensor ECU RX 메시지 처리
 *
 * Node0에서 ZCU가 보낸 메시지를 처리한다.
 */
void CanIf_onReceive(uint32 id, const uint8_t *data, uint8_t length)
{
    testLastRxId = id;

    switch(id)
    {
        case CAN_ID_VEHICLE_STATE:
        {
            VehicleState_Frame_t frame;

            if(length < CAN_DLC_VEHICLE_STATE)
            {
                break;
            }

            memcpy(frame.raw, data, CAN_DLC_VEHICLE_STATE);

            testGearState = frame.fields.gearState;

            if(frame.fields.gearState == GEAR_STATE_D)
            {
                testSensorReadEnabled = TRUE;
            }
            else
            {
                testSensorReadEnabled = FALSE;
            }

            break;
        }

        case CAN_ID_OTA_REQUEST:
        {
            if(length < 1U)
            {
                break;
            }

            testOtaServiceId = data[0];

            switch(testOtaServiceId)
            {
                case OTA_SERVICE_START:
                {
                    OtaStartRequest_t req;

                    if(length < OTA_START_REQUEST_SIZE)
                    {
                        break;
                    }

                    memcpy(&req, data, OTA_START_REQUEST_SIZE);

                    testOtaFirmwareSize  = req.firmwareSize;
                    testOtaFirmwareCrc32 = req.firmwareCrc32;

                    break;
                }

                case OTA_SERVICE_TRANSFER_DATA:
                {
                    OtaTransferDataRequest_t reqHeader;

                    if(length < OTA_TRANSFER_DATA_HEADER_SIZE)
                    {
                        break;
                    }

                    memcpy(&reqHeader, data, OTA_TRANSFER_DATA_HEADER_SIZE);

                    testOtaBlockSequence = reqHeader.blockSequence;
                    testOtaDataLength    = reqHeader.dataLength;

                    break;
                }

                case OTA_SERVICE_TRANSFER_EXIT:
                {
                    OtaTransferExitRequest_t req;

                    if(length < OTA_TRANSFER_EXIT_REQUEST_SIZE)
                    {
                        break;
                    }

                    memcpy(&req, data, OTA_TRANSFER_EXIT_REQUEST_SIZE);

                    testOtaTotalCrc32   = req.totalCrc32;
                    testOtaApplyRequest = req.applyRequest;

                    break;
                }

                case OTA_SERVICE_RESET:
                {
                    OtaResetRequest_t req;

                    if(length < OTA_RESET_REQUEST_SIZE)
                    {
                        break;
                    }

                    memcpy(&req, data, OTA_RESET_REQUEST_SIZE);

                    testOtaResetType = req.resetType;

                    break;
                }

                default:
                    break;
            }

            break;
        }

        default:
            break;
    }
}


/*********************************************************************************************************************/
/*-----------------------------------------Unused common helper function stubs----------------------------------------*/
/*********************************************************************************************************************/

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

CanTxResult_t CanIf_sendOtaRequest(const uint8_t *payload, uint16_t length)
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
 * Node0 CAN_STB = P20.6
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
 *
 * MODULE_CAN0은 Node0과 Node2가 공유하므로 한 번만 초기화한다.
 */
static void initCanModule(void)
{
    IfxCan_Can_initModuleConfig(&g_mcmcan.canConfig, &MODULE_CAN0);
    IfxCan_Can_initModule(&g_mcmcan.canModule, &g_mcmcan.canConfig);
}


/**
 * @brief Node0 초기화
 *
 * Node0:
 *  - ZCU 통신용
 *  - Classical CAN + CAN FD
 */
static void initCanNode0(void)
{
    IfxCan_Can_initNodeConfig(&g_mcmcan.canNodeConfig,
                              &g_mcmcan.canModule);

    g_mcmcan.canNodeConfig.nodeId = MCMCAN_USED_NODE_ID;
    g_mcmcan.canNodeConfig.busLoopbackEnabled = FALSE;

    g_mcmcan.canNodeConfig.frame.type = IfxCan_FrameType_transmitAndReceive;
    g_mcmcan.canNodeConfig.frame.mode = IfxCan_FrameMode_fdLong;

    g_mcmcan.canNodeConfig.pins = &g_mcmcanNode0Pins;

    /*
     * TX/RX buffer size
     */
    g_mcmcan.canNodeConfig.txConfig.txMode = IfxCan_TxMode_dedicatedBuffers;
    g_mcmcan.canNodeConfig.txConfig.txBufferDataFieldSize = IfxCan_DataFieldSize_64;

    g_mcmcan.canNodeConfig.rxConfig.rxMode = IfxCan_RxMode_dedicatedBuffers;
    g_mcmcan.canNodeConfig.rxConfig.rxBufferDataFieldSize = IfxCan_DataFieldSize_64;

    /*
     * Standard ID filter 2개:
     * 0x080, 0x600
     */
    g_mcmcan.canNodeConfig.filterConfig.messageIdLength = IfxCan_MessageIdLength_standard;
    g_mcmcan.canNodeConfig.filterConfig.standardListSize = 2U;
    g_mcmcan.canNodeConfig.filterConfig.extendedListSize = 0U;

    g_mcmcan.canNodeConfig.filterConfig.standardFilterForNonMatchingFrames =
        IfxCan_NonMatchingFrame_reject;
    g_mcmcan.canNodeConfig.filterConfig.extendedFilterForNonMatchingFrames =
        IfxCan_NonMatchingFrame_reject;

    g_mcmcan.canNodeConfig.filterConfig.rejectRemoteFramesWithStandardId = TRUE;
    g_mcmcan.canNodeConfig.filterConfig.rejectRemoteFramesWithExtendedId = TRUE;

    /*
     * Node0 Message RAM 분리
     */
    setNode0MessageRamLayout();

    /*
     * TX interrupt
     */
    g_mcmcan.canNodeConfig.interruptConfig.transmissionCompletedEnabled = TRUE;
    g_mcmcan.canNodeConfig.interruptConfig.traco.priority = ISR_PRIORITY_CAN_TX;
    g_mcmcan.canNodeConfig.interruptConfig.traco.interruptLine = IfxCan_InterruptLine_0;
    g_mcmcan.canNodeConfig.interruptConfig.traco.typeOfService = IfxSrc_Tos_cpu0;

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
 * @brief Sensor ECU Node0 RX Filter 설정
 *
 * 0x080 VehicleState -> RxBuffer0
 * 0x600 OtaRequest   -> RxBuffer1
 */
static void initSensorNode0RxFilters(void)
{
    setNode0StandardFilter(0U, CAN_ID_VEHICLE_STATE, IfxCan_RxBufferId_0);
    setNode0StandardFilter(1U, CAN_ID_OTA_REQUEST,   IfxCan_RxBufferId_1);
}


/**
 * @brief Node0 Standard ID filter 설정
 */
static void setNode0StandardFilter(uint8 filterNumber, uint32 canId, IfxCan_RxBufferId rxBufferId)
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
 * @brief TOF 센서용 CAN Node2 초기화
 *
 * Node2:
 *  - Classical CAN
 *  - TX: P15.0
 *  - RX: P15.1
 *  - RX FIFO0 사용
 */
static void initTofCanNode2(void)
{
    IfxCan_Can_initNodeConfig(&g_tofCanNode2Config,
                              &g_mcmcan.canModule);

    g_tofCanNode2Config.nodeId = IfxCan_NodeId_2;
    g_tofCanNode2Config.busLoopbackEnabled = FALSE;

    /*
     * TOF 센서 CAN 속도
     */
    g_tofCanNode2Config.baudRate.baudrate = TOF_CAN_BAUDRATE;

    g_tofCanNode2Config.frame.type = IfxCan_FrameType_receive;
    g_tofCanNode2Config.frame.mode = IfxCan_FrameMode_standard;

    g_tofCanNode2Config.pins = &g_tofCanNode2Pins;

    /*
     * RX FIFO0 설정
     */
    g_tofCanNode2Config.rxConfig.rxMode = IfxCan_RxMode_sharedFifo0;
    g_tofCanNode2Config.rxConfig.rxBufferDataFieldSize = IfxCan_DataFieldSize_8;
    g_tofCanNode2Config.rxConfig.rxFifo0DataFieldSize = IfxCan_DataFieldSize_8;
    g_tofCanNode2Config.rxConfig.rxFifo0Size = TOF_CAN_RX_FIFO0_SIZE;

    /*
     * Standard ID filter 1개:
     * 0x000~0x7FF range filter
     */
    g_tofCanNode2Config.filterConfig.messageIdLength = IfxCan_MessageIdLength_standard;
    g_tofCanNode2Config.filterConfig.standardListSize = 1U;
    g_tofCanNode2Config.filterConfig.extendedListSize = 0U;

    g_tofCanNode2Config.filterConfig.standardFilterForNonMatchingFrames =
        IfxCan_NonMatchingFrame_reject;
    g_tofCanNode2Config.filterConfig.extendedFilterForNonMatchingFrames =
        IfxCan_NonMatchingFrame_reject;

    g_tofCanNode2Config.filterConfig.rejectRemoteFramesWithStandardId = TRUE;
    g_tofCanNode2Config.filterConfig.rejectRemoteFramesWithExtendedId = TRUE;

    /*
     * Node2 Message RAM 분리
     */
    setNode2MessageRamLayout();

    /*
     * Node2 RX FIFO0 interrupt
     */
    g_tofCanNode2Config.interruptConfig.rxFifo0NewMessageEnabled = TRUE;
    g_tofCanNode2Config.interruptConfig.rxf0n.priority = ISR_PRIORITY_TOF_CAN_RX;
    g_tofCanNode2Config.interruptConfig.rxf0n.interruptLine = IfxCan_InterruptLine_2;
    g_tofCanNode2Config.interruptConfig.rxf0n.typeOfService = IfxSrc_Tos_cpu0;

    IfxCan_Can_initNode(&g_tofCanNode2,
                        &g_tofCanNode2Config);
}


/**
 * @brief TOF Node2 RX filter 설정
 *
 * 우선 전체 Standard ID 0x000~0x7FF 수신.
 */
static void initTofCanNode2Filter(void)
{
    g_tofCanFilter.number = 0U;
    g_tofCanFilter.type = IfxCan_FilterType_range;
    g_tofCanFilter.elementConfiguration =
        IfxCan_FilterElementConfiguration_storeInRxFifo0;

    g_tofCanFilter.id1 = 0x000U;
    g_tofCanFilter.id2 = 0x7FFU;

    IfxCan_Can_setStandardFilter(&g_tofCanNode2,
                                 &g_tofCanFilter);
}


/**
 * @brief Node0 Message RAM layout
 *
 * Node0:
 *  - ZCU 통신용
 *  - Dedicated RX Buffer
 *  - TX Buffer
 *  - CAN FD 64 byte까지 고려
 */
static void setNode0MessageRamLayout(void)
{
    g_mcmcan.canNodeConfig.messageRAM.standardFilterListStartAddress = 0x000U;
    g_mcmcan.canNodeConfig.messageRAM.extendedFilterListStartAddress = 0x040U;

    /*
     * Node0은 RX FIFO를 쓰지 않지만 영역 충돌 방지용으로 분리
     */
    g_mcmcan.canNodeConfig.messageRAM.rxFifo0StartAddress = 0x080U;
    g_mcmcan.canNodeConfig.messageRAM.rxFifo1StartAddress = 0x100U;

    /*
     * Node0 Dedicated RxBuffer0~1
     */
    g_mcmcan.canNodeConfig.messageRAM.rxBuffersStartAddress = 0x180U;

    /*
     * Node0 TX 영역
     */
    g_mcmcan.canNodeConfig.messageRAM.txEventFifoStartAddress = 0x300U;
    g_mcmcan.canNodeConfig.messageRAM.txBuffersStartAddress   = 0x340U;
}


/**
 * @brief Node2 Message RAM layout
 *
 * Node2:
 *  - TOF 센서 CAN 수신용
 *  - RX FIFO0 사용
 *  - Classical CAN 8 byte
 */
static void setNode2MessageRamLayout(void)
{
    g_tofCanNode2Config.messageRAM.standardFilterListStartAddress = 0x400U;
    g_tofCanNode2Config.messageRAM.extendedFilterListStartAddress = 0x440U;

    /*
     * Node2 RX FIFO0
     */
    g_tofCanNode2Config.messageRAM.rxFifo0StartAddress = 0x480U;
    g_tofCanNode2Config.messageRAM.rxFifo1StartAddress = 0x600U;

    /*
     * Node2는 Dedicated RxBuffer를 쓰지 않지만 영역 충돌 방지용으로 분리
     */
    g_tofCanNode2Config.messageRAM.rxBuffersStartAddress = 0x640U;

    /*
     * Node2 TX 영역은 현재 사용하지 않지만 충돌 방지용으로 분리
     */
    g_tofCanNode2Config.messageRAM.txEventFifoStartAddress = 0x680U;
    g_tofCanNode2Config.messageRAM.txBuffersStartAddress   = 0x6C0U;
}


/**
 * @brief Node0 새 데이터가 들어온 RX Buffer들을 모두 읽음
 */
static void readNode0UpdatedRxBuffers(void)
{
    if(IfxCan_Node_isRxBufferNewDataUpdated(g_mcmcan.canNode.node, IfxCan_RxBufferId_0))
    {
        readNode0RxBuffer(IfxCan_RxBufferId_0);
    }

    if(IfxCan_Node_isRxBufferNewDataUpdated(g_mcmcan.canNode.node, IfxCan_RxBufferId_1))
    {
        readNode0RxBuffer(IfxCan_RxBufferId_1);
    }
}


/**
 * @brief Node0 특정 Dedicated RX Buffer에서 메시지 읽기
 */
static void readNode0RxBuffer(IfxCan_RxBufferId rxBufferId)
{
    uint8_t rxBytes[CANFD_MAX_DLC];
    uint8_t length;

    IfxCan_Can_initMessage(&g_mcmcan.rxMsg);
    memset(g_mcmcan.rxData, 0, sizeof(g_mcmcan.rxData));
    memset(rxBytes, 0, sizeof(rxBytes));

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
 * @brief TOF Node2 RX FIFO0에서 메시지 1개 읽기
 */
static void readTofCanNode2Fifo0(void)
{
    uint8_t rxBytes[CAN_CLASSIC_MAX_DLC];
    uint8 length;
    uint8 i;

    IfxCan_Can_initMessage(&g_tofRxMsg);
    memset(g_tofRxData, 0, sizeof(g_tofRxData));
    memset(rxBytes, 0, sizeof(rxBytes));

    g_tofRxMsg.readFromRxFifo0 = TRUE;
    g_tofRxMsg.readFromRxFifo1 = FALSE;

    IfxCan_Can_readMessage(&g_tofCanNode2,
                           &g_tofRxMsg,
                           g_tofRxData);

    length = getLengthFromDlc(g_tofRxMsg.dataLengthCode);

    if(length > CAN_CLASSIC_MAX_DLC)
    {
        length = CAN_CLASSIC_MAX_DLC;
    }

    copyWordsToBytes(rxBytes, g_tofRxData, length);

    testTofCanRxId = g_tofRxMsg.messageId;
    testTofCanRxLength = length;
    testTofCanRxCount++;

    for(i = 0U; i < 8U; i++)
    {
        if(i < length)
        {
            testTofRawBytes[i] = rxBytes[i];
        }
        else
        {
            testTofRawBytes[i] = 0U;
        }
    }

    /*
     * 거리 계산/센서 payload 해석은 MCMCAN.c에서 하지 않는다.
     * TOF Sensor 모듈로 raw CAN frame을 넘긴다.
     */
    TofSensor_onCanFrame(g_tofRxMsg.messageId,
                         rxBytes,
                         length);
}


/**
 * @brief TX Queue에 메시지 저장
 */
static CanTxResult_t enqueueTxMessage(uint32 id,
                                      const uint8_t *data,
                                      uint8 length,
                                      boolean isFd)
{
    CanTxQueueItem_t *item;
    boolean interruptState;

    interruptState = IfxCpu_disableInterrupts();

    if(g_txQueueCount >= CAN_TX_QUEUE_SIZE)
    {
        testTxDropCount++;
        IfxCpu_restoreInterrupts(interruptState);
        return CAN_TX_BUSY;
    }

    item = &g_txQueue[g_txQueueTail];

    item->id     = id;
    item->length = length;
    item->isFd   = isFd;

    memset(item->data, 0, CANFD_MAX_DLC);
    memcpy(item->data, data, length);

    g_txQueueTail++;

    if(g_txQueueTail >= CAN_TX_QUEUE_SIZE)
    {
        g_txQueueTail = 0U;
    }

    g_txQueueCount++;
    testTxQueuedCount++;

    tryStartNextTx();

    IfxCpu_restoreInterrupts(interruptState);

    return CAN_TX_OK;
}


/**
 * @brief TX Queue에서 다음 메시지 송신 시도
 */
static void tryStartNextTx(void)
{
    CanTxQueueItem_t *item;
    IfxCan_Status status;

    if(g_txInProgress == TRUE)
    {
        return;
    }

    if(g_txQueueCount == 0U)
    {
        return;
    }

    item = &g_txQueue[g_txQueueHead];

    prepareTxMessage(item);

    status = IfxCan_Can_sendMessage(&g_mcmcan.canNode,
                                    &g_mcmcan.txMsg,
                                    g_mcmcan.txData);

    if(status == IfxCan_Status_notSentBusy)
    {
        testTxBusyCount++;
        return;
    }

    g_txQueueHead++;

    if(g_txQueueHead >= CAN_TX_QUEUE_SIZE)
    {
        g_txQueueHead = 0U;
    }

    g_txQueueCount--;
    g_txInProgress = TRUE;
}


/**
 * @brief Queue item을 IfxCan_Message + txData로 변환
 */
static void prepareTxMessage(const CanTxQueueItem_t *item)
{
    IfxCan_Can_initMessage(&g_mcmcan.txMsg);
    memset(g_mcmcan.txData, 0, sizeof(g_mcmcan.txData));

    copyBytesToWords(g_mcmcan.txData, item->data, item->length);

    g_mcmcan.txMsg.messageId       = item->id;
    g_mcmcan.txMsg.messageIdLength = IfxCan_MessageIdLength_standard;

    if(item->isFd == TRUE)
    {
        g_mcmcan.txMsg.frameMode      = IfxCan_FrameMode_fdLong;
        g_mcmcan.txMsg.dataLengthCode = getCanFdDlc(item->length);
    }
    else
    {
        g_mcmcan.txMsg.frameMode      = IfxCan_FrameMode_standard;
        g_mcmcan.txMsg.dataLengthCode = getClassicDlc(item->length);
    }
}


/**
 * @brief Classical CAN DLC 변환
 */
static IfxCan_DataLengthCode getClassicDlc(uint8 dlc)
{
    switch(dlc)
    {
        case 0U: return IfxCan_DataLengthCode_0;
        case 1U: return IfxCan_DataLengthCode_1;
        case 2U: return IfxCan_DataLengthCode_2;
        case 3U: return IfxCan_DataLengthCode_3;
        case 4U: return IfxCan_DataLengthCode_4;
        case 5U: return IfxCan_DataLengthCode_5;
        case 6U: return IfxCan_DataLengthCode_6;
        case 7U: return IfxCan_DataLengthCode_7;
        case 8U: return IfxCan_DataLengthCode_8;
        default: return IfxCan_DataLengthCode_0;
    }
}


/**
 * @brief CAN FD length -> DLC 변환
 */
static IfxCan_DataLengthCode getCanFdDlc(uint8 length)
{
    if(length <= 8U)
    {
        return getClassicDlc(length);
    }
    else if(length <= 12U)
    {
        return IfxCan_DataLengthCode_12;
    }
    else if(length <= 16U)
    {
        return IfxCan_DataLengthCode_16;
    }
    else if(length <= 20U)
    {
        return IfxCan_DataLengthCode_20;
    }
    else if(length <= 24U)
    {
        return IfxCan_DataLengthCode_24;
    }
    else if(length <= 32U)
    {
        return IfxCan_DataLengthCode_32;
    }
    else if(length <= 48U)
    {
        return IfxCan_DataLengthCode_48;
    }
    else
    {
        return IfxCan_DataLengthCode_64;
    }
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
 * @brief byte buffer -> uint32 word buffer
 */
static void copyBytesToWords(uint32 *wordBuffer, const uint8_t *byteBuffer, uint16_t length)
{
    memset(wordBuffer, 0, CANFD_DATA_WORDS * sizeof(uint32));
    memcpy((uint8_t *)wordBuffer, byteBuffer, length);
}


/**
 * @brief uint32 word buffer -> byte buffer
 */
static void copyWordsToBytes(uint8_t *byteBuffer, const uint32 *wordBuffer, uint16_t length)
{
    memcpy(byteBuffer, (const uint8_t *)wordBuffer, length);
}
