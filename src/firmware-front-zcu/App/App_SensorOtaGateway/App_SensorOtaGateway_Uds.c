/**********************************************************************************************************************
 * \file App_SensorOtaGateway_Uds.c
 * \brief Sensor ECU OTA Gateway UDS adapter for DoIP
 *
 * 역할:
 *  - Pi/HPC가 DoIP로 보낸 UDS payload를 해석한다.
 *  - ZCU 자기 Flash에 write하지 않는다.
 *  - App_OtaReceiver를 통해 Sensor ECU OTA Gateway로 전달한다.
 *
 * 구조:
 *  Pi/HPC
 *      ↓ DoIP
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
 * 지원 UDS 서비스:
 *  - 0x10 DiagnosticSessionControl
 *  - 0x34 RequestDownload
 *  - 0x36 TransferData
 *  - 0x37 RequestTransferExit + CRC32
 *
 * 중요:
 *  - 이 모듈은 ZCU Local OTA용이 아니다.
 *  - Sensor ECU OTA 중계용이다.
 *********************************************************************************************************************/

#include "App_SensorOtaGateway_Uds.h"

#include "App_OtaReceiver/App_OtaReceiver.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* ============================================================
   Timeout config
   ============================================================ */

/*
 * DoIP 요청 처리 함수 안에서 App_OtaReceiver 상태를 기다릴 때 사용한다.
 *
 * 주의:
 *  - vTaskDelay(1ms)를 사용하므로 이 함수는 FreeRTOS task context에서 호출되어야 한다.
 *  - 너무 짧게 잡으면 Sensor ECU flash erase/write 응답 대기 중 timeout이 날 수 있다.
 */
#define APP_SENSOR_OTA_GATEWAY_WAIT_BLOCK_TIMEOUT_MS      5000U
#define APP_SENSOR_OTA_GATEWAY_WAIT_NEXT_TIMEOUT_MS       5000U 
#define APP_SENSOR_OTA_GATEWAY_WAIT_DONE_TIMEOUT_MS       60000U

#define APP_SENSOR_OTA_GATEWAY_UDS_WORKER_STACK_SIZE      (configMINIMAL_STACK_SIZE + 128U)
#define APP_SENSOR_OTA_GATEWAY_UDS_WORKER_PRIORITY        (tskIDLE_PRIORITY + 1U)
#define APP_SENSOR_OTA_GATEWAY_UDS_WORKER_PERIOD_MS       1U

/* ============================================================
   Internal state
   ============================================================ */

typedef enum
{
    APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_IDLE = 0,
    APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_PENDING,
    APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_PROCESSING,
    APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_DONE,
    APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_ERROR
} AppSensorOtaGatewayUds_AsyncState_t;

static uint8  g_session = APP_SENSOR_OTA_GATEWAY_UDS_SESSION_DEFAULT;

static uint32 g_downloadSize = 0U;
static uint32 g_receivedBytes = 0U;
static uint8  g_blockSeq = 0x01U;

/* 마지막 0x37에서 받은 CRC 확인용 */
static uint32 g_expectedCrc32 = 0U;

static volatile AppSensorOtaGatewayUds_AsyncState_t g_asyncState =
    APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_IDLE;
static boolean g_asyncWorkerCreated = FALSE;
static uint16 g_asyncRxLen = 0U;
static uint16 g_asyncTxLen = 0U;
static uint8 g_asyncRx[APP_SENSOR_OTA_GATEWAY_UDS_RX_BUF_SIZE];
static uint8 g_asyncTx[APP_SENSOR_OTA_GATEWAY_UDS_TX_BUF_SIZE];

/* ============================================================
   Debug variables
   ============================================================ */

volatile uint32 g_sensorOtaUdsSessionReqCount = 0U;
volatile uint32 g_sensorOtaUdsRequestDownloadCount = 0U;
volatile uint32 g_sensorOtaUdsTransferDataCount = 0U;
volatile uint32 g_sensorOtaUdsTransferExitCount = 0U;

volatile uint32 g_sensorOtaUdsNegResponseCount = 0U;

volatile uint32 g_sensorOtaUdsLastSid = 0U;
volatile uint32 g_sensorOtaUdsLastNrc = 0U;

volatile uint32 g_sensorOtaUdsDownloadSize = 0U;
volatile uint32 g_sensorOtaUdsReceivedBytes = 0U;
volatile uint32 g_sensorOtaUdsExpectedCrc32 = 0U;

volatile uint32 g_sensorOtaUdsLastBlockIndex = 0U;
volatile uint32 g_sensorOtaUdsLastBlockLength = 0U;

volatile uint32 g_sensorOtaUdsStartNoCrcOkCount = 0U;
volatile uint32 g_sensorOtaUdsStartNoCrcFailCount = 0U;

volatile uint32 g_sensorOtaUdsProvideBlockOkCount = 0U;
volatile uint32 g_sensorOtaUdsProvideBlockFailCount = 0U;

volatile uint32 g_sensorOtaUdsFinalCrcOkCount = 0U;
volatile uint32 g_sensorOtaUdsFinalCrcFailCount = 0U;

volatile uint32 g_sensorOtaUdsWaitBlockTimeoutCount = 0U;
volatile uint32 g_sensorOtaUdsWaitNextTimeoutCount = 0U;
volatile uint32 g_sensorOtaUdsWaitDoneTimeoutCount = 0U;
volatile uint32 g_sensorOtaUdsAsyncStartOkCount = 0U;
volatile uint32 g_sensorOtaUdsAsyncStartFailCount = 0U;
volatile uint32 g_sensorOtaUdsAsyncDoneCount = 0U;
volatile uint32 g_sensorOtaUdsAsyncBusyCount = 0U;
volatile uint32 g_sensorOtaUdsAsyncErrorCount = 0U;
//debug
volatile uint32 g_dbgWaitNextFailBlockIndex = 0U;
volatile uint32 g_dbgWaitNextFailReqIndex = 0U;
volatile uint32 g_dbgWaitNextFailIsWaitingBlock = 0U;
volatile uint32 g_dbgWaitNextFailIsWaitingFinalCrc = 0U;
volatile uint32 g_dbgWaitNextFailIsError = 0U;

/* ============================================================
   Private prototypes
   ============================================================ */

static uint16 makeNegativeResponse(uint8 *tx, uint8 sid, uint8 nrc);

static uint16 handleSessionControl(uint8 *rx, uint16 rxLen, uint8 *tx);
static uint16 handleRequestDownload(uint8 *rx, uint16 rxLen, uint8 *tx);
static uint16 handleTransferData(uint8 *rx, uint16 rxLen, uint8 *tx);
static uint16 handleTransferExit(uint8 *rx, uint16 rxLen, uint8 *tx);
static uint16 handleEcuReset(uint8 *rx, uint16 rxLen, uint8 *tx);

static uint32 readU32Be(const uint8 *p);

static boolean waitUntilWaitingBlock(uint32 timeoutMs);
static boolean waitUntilBlockConsumedOrFinalCrc(uint32 blockIndex, uint32 timeoutMs);
static boolean waitUntilDoneOrError(uint32 timeoutMs);

static void resetDownloadState(void);
static uint8 nextBlockSeq(uint8 currentSeq);
static void AppSensorOtaGatewayUds_WorkerTask(void *arg);


/* ============================================================
   Public functions
   ============================================================ */

void AppSensorOtaGatewayUds_Init(void)
{
    g_session = APP_SENSOR_OTA_GATEWAY_UDS_SESSION_DEFAULT;
    resetDownloadState();

    g_sensorOtaUdsSessionReqCount = 0U;
    g_sensorOtaUdsRequestDownloadCount = 0U;
    g_sensorOtaUdsTransferDataCount = 0U;
    g_sensorOtaUdsTransferExitCount = 0U;

    g_sensorOtaUdsNegResponseCount = 0U;

    g_sensorOtaUdsLastSid = 0U;
    g_sensorOtaUdsLastNrc = 0U;

    g_sensorOtaUdsDownloadSize = 0U;
    g_sensorOtaUdsReceivedBytes = 0U;
    g_sensorOtaUdsExpectedCrc32 = 0U;

    g_sensorOtaUdsLastBlockIndex = 0U;
    g_sensorOtaUdsLastBlockLength = 0U;

    g_sensorOtaUdsStartNoCrcOkCount = 0U;
    g_sensorOtaUdsStartNoCrcFailCount = 0U;

    g_sensorOtaUdsProvideBlockOkCount = 0U;
    g_sensorOtaUdsProvideBlockFailCount = 0U;

    g_sensorOtaUdsFinalCrcOkCount = 0U;
    g_sensorOtaUdsFinalCrcFailCount = 0U;

    g_sensorOtaUdsWaitBlockTimeoutCount = 0U;
    g_sensorOtaUdsWaitNextTimeoutCount = 0U;
    g_sensorOtaUdsWaitDoneTimeoutCount = 0U;

    g_sensorOtaUdsAsyncStartOkCount = 0U;
    g_sensorOtaUdsAsyncStartFailCount = 0U;
    g_sensorOtaUdsAsyncDoneCount = 0U;
    g_sensorOtaUdsAsyncBusyCount = 0U;
    g_sensorOtaUdsAsyncErrorCount = 0U;

    taskENTER_CRITICAL();
    g_asyncState = APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_IDLE;
    g_asyncRxLen = 0U;
    g_asyncTxLen = 0U;
    memset(g_asyncRx, 0, sizeof(g_asyncRx));
    memset(g_asyncTx, 0, sizeof(g_asyncTx));
    taskEXIT_CRITICAL();

    if(g_asyncWorkerCreated == FALSE)
    {
        BaseType_t taskResult;

        taskResult = xTaskCreate(AppSensorOtaGatewayUds_WorkerTask,
                                 "SENSOR OTA UDS",
                                 APP_SENSOR_OTA_GATEWAY_UDS_WORKER_STACK_SIZE,
                                 NULL,
                                 APP_SENSOR_OTA_GATEWAY_UDS_WORKER_PRIORITY,
                                 NULL);

        if(taskResult == pdPASS)
        {
            g_asyncWorkerCreated = TRUE;
        }
        else
        {
            g_sensorOtaUdsAsyncErrorCount++;
        }
    }
}


void AppSensorOtaGatewayUds_Task(void)
{
    /*
     * 현재는 별도 주기 동작 없음.
     *
     * 나중에 DoIP level timeout, session timeout, transfer timeout 등을 넣을 경우
     * 이 함수에서 처리한다.
    */
}

boolean AppSensorOtaGatewayUds_TryStartRequest(const uint8 *rxData,
                                               uint16 rxLen)
{
    boolean result = FALSE;

    if((g_asyncWorkerCreated == FALSE) ||
       (rxData == NULL_PTR) ||
       (rxLen == 0U) ||
       (rxLen > APP_SENSOR_OTA_GATEWAY_UDS_RX_BUF_SIZE))
    {
        g_sensorOtaUdsAsyncStartFailCount++;
        return FALSE;
    }

    taskENTER_CRITICAL();

    if(g_asyncState == APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_IDLE)
    {
        memcpy(g_asyncRx, rxData, rxLen);
        memset(g_asyncTx, 0, sizeof(g_asyncTx));
        g_asyncRxLen = rxLen;
        g_asyncTxLen = 0U;
        g_asyncState = APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_PENDING;
        result = TRUE;
    }

    taskEXIT_CRITICAL();

    if(result == TRUE)
    {
        g_sensorOtaUdsAsyncStartOkCount++;
    }
    else
    {
        g_sensorOtaUdsAsyncBusyCount++;
        g_sensorOtaUdsAsyncStartFailCount++;
    }

    return result;
}


AppSensorOtaGatewayUds_ResponseStatus_t AppSensorOtaGatewayUds_TryReadResponse(uint8 *txData,
                                                                               uint16 *txLen)
{
    AppSensorOtaGatewayUds_ResponseStatus_t status =
        APP_SENSOR_OTA_GATEWAY_UDS_RESPONSE_NOT_READY;

    if((txData == NULL_PTR) || (txLen == NULL_PTR))
    {
        g_sensorOtaUdsAsyncErrorCount++;
        return APP_SENSOR_OTA_GATEWAY_UDS_RESPONSE_ERROR;
    }

    taskENTER_CRITICAL();

    if(g_asyncState == APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_DONE)
    {
        *txLen = g_asyncTxLen;
        if(g_asyncTxLen > 0U)
        {
            memcpy(txData, g_asyncTx, g_asyncTxLen);
        }

        status = APP_SENSOR_OTA_GATEWAY_UDS_RESPONSE_READY;
    }
    else if(g_asyncState == APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_ERROR)
    {
        *txLen = 0U;
        status = APP_SENSOR_OTA_GATEWAY_UDS_RESPONSE_ERROR;
    }

    taskEXIT_CRITICAL();

    return status;
}


void AppSensorOtaGatewayUds_ReleaseResponse(void)
{
    taskENTER_CRITICAL();

    if((g_asyncState == APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_DONE) ||
       (g_asyncState == APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_ERROR))
    {
        g_asyncRxLen = 0U;
        g_asyncTxLen = 0U;
        memset(g_asyncRx, 0, sizeof(g_asyncRx));
        memset(g_asyncTx, 0, sizeof(g_asyncTx));
        g_asyncState = APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_IDLE;
    }

    taskEXIT_CRITICAL();
}


void AppSensorOtaGatewayUds_HandleService(uint8  *rxData,
                                          uint16  rxLen,
                                          uint8  *txData,
                                          uint16 *txLen)
{
    uint8 sid;

    if((txData == NULL_PTR) || (txLen == NULL_PTR))
    {
        return;
    }

    if((rxData == NULL_PTR) || (rxLen == 0U))
    {
        *txLen = 0U;
        return;
    }

    sid = rxData[0];
    g_sensorOtaUdsLastSid = sid;

    switch(sid)
    {
        case APP_SENSOR_OTA_GATEWAY_UDS_SID_SESSION_CONTROL:
        {
            *txLen = handleSessionControl(rxData, rxLen, txData);
            break;
        }

        case APP_SENSOR_OTA_GATEWAY_UDS_SID_REQUEST_DOWNLOAD:
        {
            *txLen = handleRequestDownload(rxData, rxLen, txData);
            break;
        }

        case APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_DATA:
        {
            *txLen = handleTransferData(rxData, rxLen, txData);
            break;
        }

        case APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_EXIT:
        {
            *txLen = handleTransferExit(rxData, rxLen, txData);
            break;
        }
        case APP_SENSOR_OTA_GATEWAY_UDS_SID_ECU_RESET:
        {
            *txLen = handleEcuReset(rxData, rxLen, txData);
            break;
        }
        default:
        {
            *txLen = makeNegativeResponse(txData,
                                          sid,
                                          APP_SENSOR_OTA_GATEWAY_UDS_NRC_SERVICE_NOT_SUPPORTED);
            break;
        }
    }
}

/* ============================================================
   UDS service handlers
   ============================================================ */

static uint16 handleSessionControl(uint8 *rx, uint16 rxLen, uint8 *tx)
{
    uint8 reqSession;

    g_sensorOtaUdsSessionReqCount++;

    if(rxLen < 2U)
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_SESSION_CONTROL,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_GENERAL_REJECT);
    }

    reqSession = rx[1];

    if((reqSession == APP_SENSOR_OTA_GATEWAY_UDS_SESSION_DEFAULT) ||
       (reqSession == APP_SENSOR_OTA_GATEWAY_UDS_SESSION_EXTENDED))
    {
        g_session = reqSession;

        tx[0] = APP_SENSOR_OTA_GATEWAY_UDS_SID_SESSION_CONTROL +
                APP_SENSOR_OTA_GATEWAY_UDS_POS_RESPONSE_OFFSET;
        tx[1] = reqSession;

        /*
         * P2 / P2* timing.
         * 기존 팀원 코드와 호환되도록 6바이트 응답 유지.
         */
        tx[2] = 0x00U;
        tx[3] = 0x19U;    /* P2 = 25ms */
        tx[4] = 0x01U;
        tx[5] = 0xF4U;    /* P2* = 500ms */

        return 6U;
    }

    return makeNegativeResponse(tx,
                                APP_SENSOR_OTA_GATEWAY_UDS_SID_SESSION_CONTROL,
                                APP_SENSOR_OTA_GATEWAY_UDS_NRC_SUBFUNC_NOT_SUPPORTED);
}


static uint16 handleRequestDownload(uint8 *rx, uint16 rxLen, uint8 *tx)
{
    uint8 addrLen;
    uint8 sizeLen;
    uint8 i;
    uint32 firmwareSize;
    BaseType_t startResult;

    g_sensorOtaUdsRequestDownloadCount++;

    if(g_session != APP_SENSOR_OTA_GATEWAY_UDS_SESSION_EXTENDED)
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_REQUEST_DOWNLOAD,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_CONDITIONS_NOT_CORRECT);
    }

    /*
     * 요청 형식:
     *   [0] 0x34
     *   [1] dataFormatId
     *   [2] addrAndLengthFormatIdentifier
     *   [3..] address
     *   [...] size
     */
    if(rxLen < 4U)
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_REQUEST_DOWNLOAD,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_GENERAL_REJECT);
    }

    addrLen = (uint8)(rx[2] & 0x0FU);
    sizeLen = (uint8)((rx[2] >> 4) & 0x0FU);

    if((addrLen == 0U) || (sizeLen == 0U))
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_REQUEST_DOWNLOAD,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_REQUEST_OUT_OF_RANGE);
    }

    if(rxLen < (uint16)(3U + addrLen + sizeLen))
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_REQUEST_DOWNLOAD,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_GENERAL_REJECT);
    }

    /*
     * Pi/HPC가 보낸 address는 ZCU local flash write에는 사용하지 않는다.
     * 이 모듈은 Sensor ECU Gateway이므로 size만 사용한다.
     */
    firmwareSize = 0U;
    for(i = 0U; i < sizeLen; i++)
    {
        firmwareSize = (firmwareSize << 8) | rx[3U + addrLen + i];
    }

    if(firmwareSize == 0U)
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_REQUEST_DOWNLOAD,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_REQUEST_OUT_OF_RANGE);
    }

    resetDownloadState();

    g_downloadSize = firmwareSize;
    g_sensorOtaUdsDownloadSize = firmwareSize;

    /*
     * Late CRC mode:
     *  - Pi/HPC는 CRC를 마지막 0x37에서 준다.
     *  - 따라서 여기서는 size만으로 Sensor ECU OTA를 시작한다.
     */
    startResult = AppOtaReceiver_StartDownloadWithoutCrc(firmwareSize, 0U);

    if(startResult != pdPASS)
    {
        g_sensorOtaUdsStartNoCrcFailCount++;

        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_REQUEST_DOWNLOAD,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_CONDITIONS_NOT_CORRECT);
    }

    g_sensorOtaUdsStartNoCrcOkCount++;

    /*
     * Sensor ECU 쪽 0x10/0x34가 끝나고 첫 block 요청 상태가 될 때까지 기다린다.
     * 이 대기를 하지 않으면 HPC가 바로 0x36을 보냈을 때 Gateway가 아직 block 대기 상태가 아닐 수 있다.
     */
    if(waitUntilWaitingBlock(APP_SENSOR_OTA_GATEWAY_WAIT_BLOCK_TIMEOUT_MS) == FALSE)
    {
        g_sensorOtaUdsWaitBlockTimeoutCount++;
        if(AppOtaReceiver_IsError() == TRUE)
        {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_REQUEST_DOWNLOAD,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_GENERAL_PROG_FAILURE);
        }
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_REQUEST_DOWNLOAD,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_GENERAL_PROG_FAILURE);
    }

    /*
     * Positive response:
     *   0x74 0x20 maxBlockSizeHi maxBlockSizeLo
     *
     * 여기서 maxBlockSize = 34로 응답한다.
     * 그러면 HPC는 data payload를 34 - 2 = 32 bytes 단위로 보낸다.
     */
    tx[0] = APP_SENSOR_OTA_GATEWAY_UDS_SID_REQUEST_DOWNLOAD +
            APP_SENSOR_OTA_GATEWAY_UDS_POS_RESPONSE_OFFSET;
    tx[1] = 0x20U;
    tx[2] = (uint8)((APP_SENSOR_OTA_GATEWAY_UDS_MAX_BLOCK_SIZE >> 8) & 0xFFU);
    tx[3] = (uint8)(APP_SENSOR_OTA_GATEWAY_UDS_MAX_BLOCK_SIZE & 0xFFU);

    return 4U;
}


static uint16 handleTransferData(uint8 *rx, uint16 rxLen, uint8 *tx)
{
    uint8 seqCounter;
    uint8 *chunk;
    uint16 chunkLen;
    uint32 requestedBlockIndex;
    uint8 requestedLength;
    BaseType_t provideResult;

    g_sensorOtaUdsTransferDataCount++;

    if(g_downloadSize == 0U)
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_DATA,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_CONDITIONS_NOT_CORRECT);
    }

    if(rxLen < 2U)
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_DATA,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_GENERAL_REJECT);
    }

    seqCounter = rx[1];

    if(seqCounter != g_blockSeq)
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_DATA,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_WRONG_BLOCK_SEQ_COUNTER);
    }

    chunk = &rx[2];
    chunkLen = (uint16)(rxLen - 2U);

    if((chunkLen == 0U) ||
       (chunkLen > APP_SENSOR_OTA_GATEWAY_UDS_DATA_SIZE))
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_DATA,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_REQUEST_OUT_OF_RANGE);
    }

    if(AppOtaReceiver_IsWaitingBlock() == FALSE)
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_DATA,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_CONDITIONS_NOT_CORRECT);
    }

    requestedBlockIndex = AppOtaReceiver_GetRequestedBlockIndex();
    requestedLength = AppOtaReceiver_GetRequestedLength();

    /*
     * 마지막 block은 32보다 작을 수 있다.
     * 따라서 App_OtaReceiver가 요구하는 length와 정확히 맞아야 한다.
     */
    if(chunkLen != requestedLength)
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_DATA,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_REQUEST_OUT_OF_RANGE);
    }

    provideResult = AppOtaReceiver_ProvideBlock(requestedBlockIndex,
                                                chunk,
                                                (uint8)chunkLen,
                                                0U);

    if(provideResult != pdPASS)
    {
        g_sensorOtaUdsProvideBlockFailCount++;

        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_DATA,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_GENERAL_PROG_FAILURE);
    }

    g_sensorOtaUdsProvideBlockOkCount++;

    g_sensorOtaUdsLastBlockIndex = requestedBlockIndex;
    g_sensorOtaUdsLastBlockLength = chunkLen;

    /*
     * Sensor ECU 쪽 0x36 응답까지 처리되어 다음 block 대기 상태 또는 WAIT_FINAL_CRC 상태가 될 때까지 기다린다.
     */
    if(waitUntilBlockConsumedOrFinalCrc(requestedBlockIndex,
                                    APP_SENSOR_OTA_GATEWAY_WAIT_NEXT_TIMEOUT_MS) == FALSE)
    {
        g_sensorOtaUdsWaitNextTimeoutCount++;

        g_dbgWaitNextFailBlockIndex = requestedBlockIndex;
        g_dbgWaitNextFailReqIndex = AppOtaReceiver_GetRequestedBlockIndex();
        g_dbgWaitNextFailIsWaitingBlock = AppOtaReceiver_IsWaitingBlock();
        g_dbgWaitNextFailIsWaitingFinalCrc = AppOtaReceiver_IsWaitingFinalCrc();
        g_dbgWaitNextFailIsError = AppOtaReceiver_IsError();

        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_DATA,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_GENERAL_PROG_FAILURE);
    }

    g_receivedBytes += chunkLen;
    g_sensorOtaUdsReceivedBytes = g_receivedBytes;

    g_blockSeq = nextBlockSeq(g_blockSeq);

    tx[0] = APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_DATA +
            APP_SENSOR_OTA_GATEWAY_UDS_POS_RESPONSE_OFFSET;
    tx[1] = seqCounter;

    return 2U;
}

static uint16 handleTransferExit(uint8 *rx, uint16 rxLen, uint8 *tx)
{
    uint32 expectedCrc;
    BaseType_t crcResult;

    g_sensorOtaUdsTransferExitCount++;

    /*
     * 현재 HPC server.py 흐름 기준:
     *   0x37 + CRC32(4 bytes)
     */
    if(rxLen < 5U)
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_EXIT,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_GENERAL_REJECT);
    }

    if(g_downloadSize == 0U)
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_EXIT,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_CONDITIONS_NOT_CORRECT);
    }

    if(g_receivedBytes != g_downloadSize)
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_EXIT,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_CONDITIONS_NOT_CORRECT);
    }

    if(AppOtaReceiver_IsWaitingFinalCrc() == FALSE)
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_EXIT,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_CONDITIONS_NOT_CORRECT);
    }

    expectedCrc = readU32Be(&rx[1]);
    //expectedCrc = 0x12345678;  //debug
    g_expectedCrc32 = expectedCrc;
    g_sensorOtaUdsExpectedCrc32 = expectedCrc;

    /*
     * Late CRC mode의 마지막 단계.
     * 이 호출 이후 Sensor ECU 쪽으로:
     *   0x37 RequestTransferExit
     *   0x31 RoutineControl CRC
     * 가 진행된다.
     */
    crcResult = AppOtaReceiver_SetFinalCrc(expectedCrc, 0U);

    if(crcResult != pdPASS)
    {
        g_sensorOtaUdsFinalCrcFailCount++;

        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_EXIT,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_GENERAL_PROG_FAILURE);
    }

    if(waitUntilDoneOrError(APP_SENSOR_OTA_GATEWAY_WAIT_DONE_TIMEOUT_MS) == FALSE)
    {
        g_sensorOtaUdsWaitDoneTimeoutCount++;

        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_EXIT,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_GENERAL_PROG_FAILURE);
    }

    if(AppOtaReceiver_IsError() == TRUE)
    {
        g_sensorOtaUdsFinalCrcFailCount++;

        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_EXIT,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_GENERAL_PROG_FAILURE);
    }

    g_sensorOtaUdsFinalCrcOkCount++;

    tx[0] = APP_SENSOR_OTA_GATEWAY_UDS_SID_TRANSFER_EXIT +
            APP_SENSOR_OTA_GATEWAY_UDS_POS_RESPONSE_OFFSET;

    /*
     * Download/Verify phase 완료.
     * Activation/SOTA switch는 아직 여기서 하지 않는다.
     */
    g_session = APP_SENSOR_OTA_GATEWAY_UDS_SESSION_DEFAULT;
    resetDownloadState();

    return 1U;
}

static uint16 handleEcuReset(uint8 *rx, uint16 rxLen, uint8 *tx)
{
    BaseType_t resetResult;

    if (rxLen < 2U)
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_ECU_RESET,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_GENERAL_REJECT);
    }

    if (rx[1] != APP_SENSOR_OTA_GATEWAY_UDS_RESET_HARD_RESET)
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_ECU_RESET,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_REQUEST_OUT_OF_RANGE);
    }

    /*
     * Sensor ECU OTA download/verify가 끝난 뒤에만 reset 가능해야 한다.
     * UdsOtaClient_RequestEcuReset() 내부에서 state == DONE인지 검사한다.
     */
    resetResult = AppOtaReceiver_RequestSensorEcuReset(0U);

    if (resetResult != pdPASS)
    {
        return makeNegativeResponse(tx,
                                    APP_SENSOR_OTA_GATEWAY_UDS_SID_ECU_RESET,
                                    APP_SENSOR_OTA_GATEWAY_UDS_NRC_CONDITIONS_NOT_CORRECT);
    }

    /*
     * 여기서의 0x51 응답은 "ZCU가 reset 요청을 접수했다"는 응답이다.
     * 실제 Sensor ECU의 0x51은 UdsOtaClient 상태머신이 CAN 0x601로 받는다.
     *
     * 엄밀히는 Sensor ECU 0x51을 기다린 뒤 DoIP 0x51을 주는 게 맞지만,
     * 테스트 단계에서는 접수 응답으로 먼저 확인해도 된다.
     */
    tx[0] = APP_SENSOR_OTA_GATEWAY_UDS_SID_ECU_RESET
            + APP_SENSOR_OTA_GATEWAY_UDS_POS_RESPONSE_OFFSET;
    tx[1] = rx[1];

    return 2U;
}
/* ============================================================
   Helper functions
   ============================================================ */

static uint16 makeNegativeResponse(uint8 *tx, uint8 sid, uint8 nrc)
{
    g_sensorOtaUdsNegResponseCount++;
    g_sensorOtaUdsLastNrc = nrc;

    tx[0] = APP_SENSOR_OTA_GATEWAY_UDS_NEGATIVE_RSP;
    tx[1] = sid;
    tx[2] = nrc;

    return 3U;
}


static uint32 readU32Be(const uint8 *p)
{
    return ((uint32)p[0] << 24) |
           ((uint32)p[1] << 16) |
           ((uint32)p[2] << 8)  |
           ((uint32)p[3]);
}


static boolean waitUntilWaitingBlock(uint32 timeoutMs)
{
    uint32 elapsedMs = 0U;

    while(elapsedMs < timeoutMs)
    {
        /*
         * 새 StartDownload가 정상 처리되면 결국 WAIT_BLOCK으로 진입한다.
         */
        if(AppOtaReceiver_IsWaitingBlock() == TRUE)
        {
            return TRUE;
        }

        /*
         * ERROR는 실제 실패로 본다.
         */
        if(AppOtaReceiver_IsError() == TRUE)
        {
            return FALSE;
        }

        /*
         * 중요:
         * 이전 OTA가 성공한 직후에는 AppOtaReceiver_IsDone()이 잠깐 TRUE로 남아 있을 수 있다.
         *
         * 기존 코드는 여기서 DONE을 실패로 처리했기 때문에,
         * 새 0x34가 Sensor ECU까지 정상 전달되고 Sensor ECU가 0x74를 줬는데도
         * ZCU가 PC에는 7F 34 72를 반환하는 문제가 발생했다.
         *
         * 따라서 WAIT_BLOCK 대기 중에는 DONE을 실패 조건으로 보지 않는다.
         * 새 StartDownload가 실패해서 WAIT_BLOCK으로 못 가면 timeout으로 빠진다.
         */

        vTaskDelay(pdMS_TO_TICKS(1U));
        elapsedMs++;
    }

    return FALSE;
}


static boolean waitUntilBlockConsumedOrFinalCrc(uint32 blockIndex, uint32 timeoutMs)
{
    uint32 elapsedMs = 0U;
    uint32 currentBlockIndex;

    while(elapsedMs < timeoutMs)
    {
        if(AppOtaReceiver_IsError() == TRUE)
        {
            return FALSE;
        }

        if(AppOtaReceiver_IsWaitingFinalCrc() == TRUE)
        {
            return TRUE;
        }

        if(AppOtaReceiver_IsWaitingBlock() == TRUE)
        {
            currentBlockIndex = AppOtaReceiver_GetRequestedBlockIndex();

            if(currentBlockIndex != blockIndex)
            {
                return TRUE;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
        elapsedMs++;
    }

    return FALSE;
}


static boolean waitUntilDoneOrError(uint32 timeoutMs)
{
    uint32 elapsedMs = 0U;

    while(elapsedMs < timeoutMs)
    {
        if(AppOtaReceiver_IsDone() == TRUE)
        {
            return TRUE;
        }

        if(AppOtaReceiver_IsError() == TRUE)
        {
            return TRUE;
        }

        vTaskDelay(pdMS_TO_TICKS(1U));
        elapsedMs++;
    }

    return FALSE;
}


static void resetDownloadState(void)
{
    g_downloadSize = 0U;
    g_receivedBytes = 0U;
    g_blockSeq = 0x01U;
    g_expectedCrc32 = 0U;
}


static uint8 nextBlockSeq(uint8 currentSeq)
{
    if(currentSeq >= 0xFFU)
    {
        return 0x00U;
    }

    return (uint8)(currentSeq + 1U);
}


static void AppSensorOtaGatewayUds_WorkerTask(void *arg)
{
    uint16 localTxLen;

    (void)arg;

    for(;;)
    {
        if(g_asyncState == APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_PENDING)
        {
            taskENTER_CRITICAL();
            if(g_asyncState == APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_PENDING)
            {
                g_asyncState = APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_PROCESSING;
            }
            taskEXIT_CRITICAL();

            if(g_asyncState == APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_PROCESSING)
            {
                localTxLen = 0U;

                AppSensorOtaGatewayUds_HandleService(g_asyncRx,
                                                     g_asyncRxLen,
                                                     g_asyncTx,
                                                     &localTxLen);

                taskENTER_CRITICAL();

                if(localTxLen <= APP_SENSOR_OTA_GATEWAY_UDS_TX_BUF_SIZE)
                {
                    g_asyncTxLen = localTxLen;
                    g_asyncState = APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_DONE;
                    g_sensorOtaUdsAsyncDoneCount++;
                }
                else
                {
                    g_asyncTxLen = 0U;
                    g_asyncState = APP_SENSOR_OTA_GATEWAY_UDS_ASYNC_ERROR;
                    g_sensorOtaUdsAsyncErrorCount++;
                }

                taskEXIT_CRITICAL();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(APP_SENSOR_OTA_GATEWAY_UDS_WORKER_PERIOD_MS));
    }
}
