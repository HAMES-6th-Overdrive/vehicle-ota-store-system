/**********************************************************************************************************************
 * \file MCMCAN.c
 * \brief Sensor ECU MCMCAN driver - Node0(ZCU) + optional Node2(TOF CAN), separated Message RAM version
 *
 * Sensor ECU 역할:
 *  - Node0 RX:
 *      0x600 UDS OTA Request -> RxBuffer0
 *
 *  - Node0 TX:
 *      0x201 TofDistanceData, FEATURE_TOF_SENSOR == 1U일 때만 Scheduler에서 송신
 *      0x202 SpeedData
 *      0x601 UDS OTA Response
 *
 *  - Node2 RX:
 *      FEATURE_TOF_SENSOR == 1U일 때만 사용
 *      TOF Sensor CAN frame -> RX FIFO0
 *      수신 frame은 TofSensor_onCanFrame(id, data, length)로 전달
 *
 * 중요:
 *  - MODULE_CAN0은 Node0과 Node2가 공유한다.
 *  - 따라서 Node0/Node2의 Message RAM 영역을 반드시 분리해야 한다.
 *  - FEATURE_TOF_SENSOR == 0U이면 Node2 TOF CAN은 초기화하지 않는다.
 *
 * 변경 사항:
 *  - IMU 센서를 사용하지 않으므로 0x200 ImuData 송신 기능 제거.
 *  - Sensor ECU 센서값 송신은 VehicleState/Gear와 무관하게 항상 수행한다.
 *  - OTA 허용 조건도 Gear P/D와 무관하게 항상 허용한다.
 *  - Sensor ECU는 더 이상 0x080 VehicleState를 수신/처리하지 않는다.
 *  - 0x600 UDS OTA Request는 RX ISR에서 직접 처리하지 않고 pending buffer에 복사한다.
 *  - 실제 UdsOta_onRequest() 호출은 CanIf_ProcessPendingOtaRequest()에서 수행한다.
 *********************************************************************************************************************/

#include "MCMCAN.h"
#include "IfxCpu.h"
#include "FeatureConfig.h"
#include "UdsOta.h"
#include "AppVersion.h"

#if (FEATURE_TOF_SENSOR == 1U)
#include "TofSensor.h"
#endif

#include <string.h>

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/

McmcanType g_mcmcan;

#define CAN_TX_STUCK_LIMIT_1MS   1U

static volatile uint32_t g_txInProgressTicks = 0U;

volatile uint32_t testTxServiceCount = 0U;
volatile uint32_t testTxStuckRecoverCount = 0U;

/* ============================================================
   Node0 RX 디버깅용 변수
   ============================================================ */

volatile uint32_t testLastRxId = 0U;

/*
 * Legacy debug only.
 *
 * 현재 구조에서는 ZCU가 Gear 정보를 보내지 않고,
 * Sensor ECU도 Gear 상태를 OTA 허용 조건으로 사용하지 않는다.
 *
 * 혹시 다른 파일/Watch 설정에서 참조 중일 수 있어 변수만 남겨둔다.
 * 로직에서는 사용하지 않는다.
 */
volatile uint8_t testGearState = 0U;

/*
 * 기존 Scheduler.c 호환용 변수.
 *
 * 현재 구조:
 *   센서값은 VehicleState/Gear와 무관하게 항상 송신한다.
 *   따라서 항상 TRUE로 유지한다.
 */
volatile boolean testSensorReadEnabled = TRUE;

/*
 * OTA 허용 여부.
 *
 * 현재 구조:
 *   ZCU는 Gear 정보를 알지 않는다.
 *   사용자가 HPC에서 Store 구매/다운로드를 승인하면 OTA를 수행한다.
 *   따라서 Sensor ECU는 Gear 조건 없이 OTA를 항상 허용한다.
 */
volatile boolean testOtaAllowed = TRUE;


/* OTA Request 확인용 */
volatile uint8_t  testOtaServiceId      = 0U;
volatile uint32_t testOtaFirmwareSize   = 0U;
volatile uint32_t testOtaFirmwareCrc32  = 0U;
volatile uint16_t testOtaBlockSequence  = 0U;
volatile uint16_t testOtaDataLength     = 0U;
volatile uint32_t testOtaTotalCrc32     = 0U;
volatile uint8_t  testOtaApplyRequest   = 0U;
volatile uint8_t  testOtaResetType      = 0U;


/* ============================================================
   OTA Request Pending Buffer
   - RX ISR 경로에서는 0x600 payload를 복사만 한다.
   - 실제 UdsOta_onRequest()는 main/scheduler context에서 처리한다.
   ============================================================ */

static volatile boolean g_otaRxPending = FALSE;
static volatile uint8_t g_otaRxPendingLength = 0U;
static uint8_t g_otaRxPendingData[CANFD_MAX_DLC];

volatile uint32_t testOtaRxPendingSetCount = 0U;
volatile uint32_t testOtaRxPendingProcessCount = 0U;
volatile uint32_t testOtaRxPendingDropCount = 0U;
volatile uint8_t  testOtaRxPendingLastSid = 0U;
volatile uint8_t  testOtaRxPendingLastLength = 0U;


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


#if (FEATURE_TOF_SENSOR == 1U)

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

#endif /* FEATURE_TOF_SENSOR */


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


#if (FEATURE_TOF_SENSOR == 1U)

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

#endif /* FEATURE_TOF_SENSOR */


/*********************************************************************************************************************/
/*------------------------------------------------Private prototypes-------------------------------------------------*/
/*********************************************************************************************************************/

static void initCanTransceiver(void);
static void initCanModule(void);

static void initCanNode0(void);
static void initSensorNode0RxFilters(void);

#if (FEATURE_TOF_SENSOR == 1U)
static void initTofCanNode2(void);
static void initTofCanNode2Filter(void);
#endif

static void setNode0MessageRamLayout(void);

#if (FEATURE_TOF_SENSOR == 1U)
static void setNode2MessageRamLayout(void);
#endif

static void setNode0StandardFilter(uint8 filterNumber, uint32 canId, IfxCan_RxBufferId rxBufferId);

static void readNode0UpdatedRxBuffers(void);
static void readNode0RxBuffer(IfxCan_RxBufferId rxBufferId);

#if (FEATURE_TOF_SENSOR == 1U)
static void readTofCanNode2Fifo0(void);
#endif

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

#if (FEATURE_TOF_SENSOR == 1U)
IFX_INTERRUPT(tofCanNode2RxIsrHandler, 0, ISR_PRIORITY_TOF_CAN_RX);
#endif


/**
 * @brief Node0 CAN TX interrupt handler
 */
void canIsrTxHandler(void)
{
    IfxCan_Node_clearInterruptFlag(g_mcmcan.canNode.node,
                                   IfxCan_Interrupt_transmissionCompleted);

    g_txInProgress = FALSE;
    g_txInProgressTicks = 0U;

    testTxSentCount++;

    tryStartNextTx();
}


/**
 * @brief Node0 CAN RX interrupt handler
 *
 * Dedicated RX Buffer 사용:
 *  - RxBuffer0: 0x600 UDS OTA Request
 */
void canIsrRxHandler(void)
{
    testNode0RxIsrCount++;

    IfxCan_Node_clearInterruptFlag(g_mcmcan.canNode.node,
                                   IfxCan_Interrupt_messageStoredToDedicatedRxBuffer);

    readNode0UpdatedRxBuffers();
}


#if (FEATURE_TOF_SENSOR == 1U)

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

#endif /* FEATURE_TOF_SENSOR */


/*********************************************************************************************************************/
/*------------------------------------------------Public functions---------------------------------------------------*/
/*********************************************************************************************************************/

/**
 * @brief MCMCAN 초기화
 *
 * 순서:
 *  1. CAN transceiver 활성화
 *  2. CAN module 초기화
 *  3. Node0 초기화 + Node0 filter 설정
 *  4. FEATURE_TOF_SENSOR == 1U이면 TOF parser 초기화 + Node2 초기화
 */
void initMcmcan(void)
{
    /*
     * 현재 구조:
     *  - Sensor ECU는 Gear/VehicleState를 사용하지 않는다.
     *  - 센서값 송신은 항상 허용한다.
     *  - OTA도 HPC 사용자 승인 기반으로 수행되므로 Sensor ECU 내부에서는 항상 허용한다.
     */
    testGearState = 0U;
    testSensorReadEnabled = TRUE;
    testOtaAllowed = TRUE;

    g_otaRxPending = FALSE;
    g_otaRxPendingLength = 0U;
    memset(g_otaRxPendingData, 0, sizeof(g_otaRxPendingData));

    UdsOta_init();
    UdsOta_setProgrammingAllowed(TRUE);

    initCanTransceiver();
    initCanModule();

    /*
     * Node0: ZCU 통신용
     *  - RX: 0x600 UDS OTA Request
     *  - TX: 0x201 / 0x202 / 0x601
     */
    initCanNode0();
    initSensorNode0RxFilters();

#if (FEATURE_TOF_SENSOR == 1U)
    /*
     * TOF 기능이 있는 App에서만 TOF Sensor parser와 Node2 CAN을 초기화한다.
     */
    TofSensor_init();

    /*
     * Node2: TOF 센서 CAN 수신용
     */
    initTofCanNode2();
    initTofCanNode2Filter();
#endif
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
 * @brief Sensor ECU 송신: 0x201 TofDistanceData
 *
 * FEATURE_TOF_SENSOR == 1U인 Application에서만 Scheduler가 호출한다.
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
 * @brief Sensor ECU 송신: 0x601 UDS OTA Response
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
 *
 * 주의:
 *  - 이 함수는 RX ISR 경로에서 호출될 수 있다.
 *  - 따라서 긴 작업, Flash erase/write/CRC 계산은 여기서 직접 수행하지 않는다.
 *  - OTA Request는 pending buffer에 복사만 하고,
 *    실제 UdsOta_onRequest()는 CanIf_ProcessPendingOtaRequest()에서 수행한다.
 */
void CanIf_onReceive(uint32 id, const uint8_t *data, uint8_t length)
{
    testLastRxId = id;

    switch(id)
    {
        case CAN_ID_OTA_REQUEST:
        {
            if((data != NULL_PTR) && (length > 0U) && (length <= CANFD_MAX_DLC))
            {
                testOtaServiceId = data[0];

                /*
                 * 아직 이전 OTA request가 처리되지 않았는데 새 request가 오면
                 * overwrite하고 drop count를 증가시킨다.
                 *
                 * 정상 UDS 흐름에서는 ZCU가 응답을 기다린 뒤 다음 요청을 보내므로
                 * 이 count가 증가하면 비정상 속도/처리 지연을 의심하면 된다.
                 */
                if(g_otaRxPending == TRUE)
                {
                    testOtaRxPendingDropCount++;
                }

                memset(g_otaRxPendingData, 0, sizeof(g_otaRxPendingData));
                memcpy(g_otaRxPendingData, data, length);

                g_otaRxPendingLength = length;
                g_otaRxPending = TRUE;

                testOtaRxPendingLastSid = data[0];
                testOtaRxPendingLastLength = length;
                testOtaRxPendingSetCount++;
            }

            break;
        }
        case CAN_ID_VERSION_REQUEST:
        {
            /* ZCU에서 버전 요청이 오면 0x703 Sensor Version을 송신한다. */
            const char *versionStr = APP_SENSOR_VERSION;
            uint8_t versionData[CAN_DLC_ECU_VERSION] = {0};
            strncpy((char *)versionData, versionStr, CAN_DLC_ECU_VERSION);

            CanIf_sendClassic(CAN_ID_SENSOR_VERSION, versionData, CAN_DLC_ECU_VERSION);
            break;
        }

        default:
        {
            break;
        }
    }
}


/**
 * @brief Pending OTA Request 처리
 *
 * RX ISR에서 복사해둔 0x600 UDS Request를 main/scheduler context에서 처리한다.
 *
 * 호출 위치:
 *  - Scheduler 1ms task
 *  - 또는 main while loop
 *
 * 목적:
 *  - UdsOta_onRequest() 내부에서 Flash erase/write/CRC 같은 긴 작업이 수행되더라도
 *    RX ISR 안에서 실행되지 않도록 분리한다.
 */
void CanIf_ProcessPendingOtaRequest(void)
{
    uint8_t localData[CANFD_MAX_DLC];
    uint8_t localLength;
    boolean interruptState;

    interruptState = IfxCpu_disableInterrupts();

    if(g_otaRxPending == FALSE)
    {
        IfxCpu_restoreInterrupts(interruptState);
        return;
    }

    localLength = g_otaRxPendingLength;

    if(localLength > CANFD_MAX_DLC)
    {
        localLength = CANFD_MAX_DLC;
    }

    memset(localData, 0, sizeof(localData));
    memcpy(localData, g_otaRxPendingData, localLength);

    g_otaRxPending = FALSE;
    g_otaRxPendingLength = 0U;

    IfxCpu_restoreInterrupts(interruptState);

    if(localLength > 0U)
    {
        UdsOta_onRequest(localData, localLength);
        testOtaRxPendingProcessCount++;
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
     * 0x600 UDS OTA Request
     * 0x700 Version Request
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
 * 0x600 UDS OTA Request -> RxBuffer0
 * 0x700 Version Request -> RxBuffer1
 */
static void initSensorNode0RxFilters(void)
{
    setNode0StandardFilter(0U, CAN_ID_OTA_REQUEST, IfxCan_RxBufferId_0);
    setNode0StandardFilter(1U, CAN_ID_VERSION_REQUEST, IfxCan_RxBufferId_1);
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


#if (FEATURE_TOF_SENSOR == 1U)

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

#endif /* FEATURE_TOF_SENSOR */


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
     * Node0 Dedicated RxBuffer0
     */
    g_mcmcan.canNodeConfig.messageRAM.rxBuffersStartAddress = 0x180U;

    /*
     * Node0 TX 영역
     */
    g_mcmcan.canNodeConfig.messageRAM.txEventFifoStartAddress = 0x300U;
    g_mcmcan.canNodeConfig.messageRAM.txBuffersStartAddress   = 0x340U;
}


#if (FEATURE_TOF_SENSOR == 1U)

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

#endif /* FEATURE_TOF_SENSOR */


/**
 * @brief Node0 새 데이터가 들어온 RX Buffer를 읽음
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


#if (FEATURE_TOF_SENSOR == 1U)

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

#endif /* FEATURE_TOF_SENSOR */


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
    g_txInProgressTicks = 0U;
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


void CanIf_TxService1ms(void)
{
    boolean interruptState;

    interruptState = IfxCpu_disableInterrupts();

    testTxServiceCount++;

    if(g_txInProgress == TRUE)
    {
        g_txInProgressTicks++;

        /*
         * Classical/CAN FD 송신이 20ms 동안 완료 안 되는 경우는 비정상.
         * TX complete ISR 누락 또는 CAN HW stuck 진단/회복용.
         *
         * 주의:
         *  - 현재는 SW flag만 회복한다.
         *  - 실제 HW TX cancel/flush는 수행하지 않는다.
         */
        if(g_txInProgressTicks >= CAN_TX_STUCK_LIMIT_1MS)
        {
            g_txInProgress = FALSE;
            g_txInProgressTicks = 0U;
            testTxStuckRecoverCount++;
        }
    }
    else
    {
        g_txInProgressTicks = 0U;
    }

    tryStartNextTx();

    IfxCpu_restoreInterrupts(interruptState);
}
