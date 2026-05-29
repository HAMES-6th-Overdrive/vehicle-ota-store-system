/**********************************************************************************************************************
 * \file App_OtaGateway.c
 * \brief FreeRTOS wrapper for ZCU OTA Gateway - command queue version with debug/auto-test variables
 *
 * 역할:
 *  - OtaGateway_MainFunction()을 1ms 주기로 호출한다.
 *  - AppCan_RecvById(0x601)로 Sensor ECU UDS Response를 수신한다.
 *  - 수신한 0x601 payload를 UdsOtaClient_OnResponse()로 전달한다.
 *  - Pi/HPC 계층에서 들어온 OTA_START / OTA_BLOCK / OTA_FINAL_CRC 요청을 command queue로 받아 처리한다.
 *
 * 현재 단계:
 *  - Download/Verify phase만 담당한다.
 *  - ZCU는 전체 firmware binary를 저장하지 않는다.
 *  - OTA_START에서는 firmwareSize만 받을 수도 있고, firmwareSize/firmwareCrc32를 같이 받을 수도 있다.
 *  - OTA_BLOCK에서는 현재 block 하나만 받는다.
 *  - OTA_FINAL_CRC에서는 마지막 0x37 단계에서 받은 CRC32를 Gateway에 전달한다.
 *  - SOTA/UCB_SWAP activation은 아직 만들지 않는다.
 *********************************************************************************************************************/

#include "App_OtaGateway.h"

#include "OtaGateway.h"
#include "UdsOtaClient.h"
#include "App_Can/App_Can.h"

#include "task.h"
#include "queue.h"

#include <string.h>

/* ============================================================
   Task config
   ============================================================ */

#define APP_OTA_GATEWAY_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE + 128u)
#define APP_OTA_GATEWAY_TASK_PRIORITY         (tskIDLE_PRIORITY + 2u)
#define APP_OTA_GATEWAY_TASK_PERIOD_MS        (1u)

#define APP_OTA_GATEWAY_RX_DRAIN_LIMIT        (8u)
#define APP_OTA_GATEWAY_CMD_QUEUE_LENGTH      (4u)


/* ============================================================
   Auto test config
   ============================================================ */

/*
 * 디버거 Watch에서 변수 값을 바꾸기 어려울 때 사용하는 자동 테스트 모드.
 *
 * 1U:
 *  - 부팅 후 일정 loop가 지나면 OTA_START를 자동으로 1회 요청한다.
 *  - WAIT_BLOCK 상태가 되면 현재 요청 block을 자동으로 제공한다.
 *
 * 테스트 끝나면 반드시 0U로 바꿔두는 것을 추천.
 */
#define APP_OTA_GATEWAY_AUTO_TEST_ENABLE          (0u)
#define APP_OTA_GATEWAY_AUTO_START_DELAY_LOOPS    (1000u)

#define APP_OTA_GATEWAY_AUTO_TEST_SIZE            (64u)
#define APP_OTA_GATEWAY_AUTO_TEST_CRC32           (0x778EB6E5U)


/* ============================================================
   Command types
   ============================================================ */

typedef enum
{
    APP_OTA_GATEWAY_CMD_START_DOWNLOAD = 0,
    APP_OTA_GATEWAY_CMD_START_DOWNLOAD_NO_CRC,
    APP_OTA_GATEWAY_CMD_PROVIDE_BLOCK,
    APP_OTA_GATEWAY_CMD_SET_FINAL_CRC,
    APP_OTA_GATEWAY_CMD_CANCEL,
    APP_OTA_GATEWAY_CMD_SENSOR_ECU_RESET
} AppOtaGateway_CommandType_t;


typedef struct
{
    AppOtaGateway_CommandType_t type;

    uint32_t firmwareSize;
    uint32_t firmwareCrc32;

    uint32_t blockIndex;
    uint8_t  blockLength;
    uint8_t  blockData[UDS_OTA_CLIENT_TRANSFER_DATA_SIZE];
} AppOtaGateway_Command_t;


/* ============================================================
   Private variables
   ============================================================ */

static QueueHandle_t g_appOtaCmdQueue = NULL;


/* ============================================================
   Debug variables for Watch
   ============================================================ */

/*
 * Watch에서 바로 보기 쉽게 static으로 숨기지 않는다.
 * 디버그용 변수이므로 외부 로직에서 사용하지 않는다.
 */

volatile uint32_t g_appOtaStartCallCount       = 0U;
volatile uint32_t g_appOtaStartOkCount         = 0U;
volatile uint32_t g_appOtaStartFailCount       = 0U;
volatile BaseType_t g_appOtaLastStartResult    = pdFAIL;

volatile uint32_t g_appOtaTaskLoopCount        = 0U;
volatile uint32_t g_appOtaProcessRxCallCount   = 0U;
volatile uint32_t g_appOtaMainCallCount        = 0U;

volatile uint32_t g_appOtaRx601Count           = 0U;
volatile uint32_t g_appOtaRxEmptyCount         = 0U;

volatile uint32_t g_appOtaLastRxId             = 0U;
volatile uint8_t  g_appOtaLastRxLength         = 0U;
volatile uint8_t  g_appOtaLastRxIsFd           = 0U;
volatile uint8_t  g_appOtaLastRxSid            = 0U;
volatile uint8_t  g_appOtaLastRxData[8]        = {0U};

volatile uint32_t g_appOtaOnResponseCallCount  = 0U;

/* Command queue debug */
volatile uint32_t g_appOtaCmdQueueCreateOkCount   = 0U;
volatile uint32_t g_appOtaCmdQueueCreateFailCount = 0U;

volatile uint32_t g_appOtaReqDownloadCallCount    = 0U;
volatile uint32_t g_appOtaReqDownloadQueuedCount  = 0U;
volatile uint32_t g_appOtaReqDownloadFailCount    = 0U;

volatile uint32_t g_appOtaReqDownloadNoCrcCallCount    = 0U;
volatile uint32_t g_appOtaReqDownloadNoCrcQueuedCount  = 0U;
volatile uint32_t g_appOtaReqDownloadNoCrcFailCount    = 0U;

volatile uint32_t g_appOtaProvideBlockCallCount   = 0U;
volatile uint32_t g_appOtaProvideBlockQueuedCount = 0U;
volatile uint32_t g_appOtaProvideBlockFailCount   = 0U;

volatile uint32_t g_appOtaSetFinalCrcCallCount    = 0U;
volatile uint32_t g_appOtaSetFinalCrcQueuedCount  = 0U;
volatile uint32_t g_appOtaSetFinalCrcFailCount    = 0U;

volatile uint32_t g_appOtaCancelCallCount         = 0U;
volatile uint32_t g_appOtaCancelQueuedCount       = 0U;
volatile uint32_t g_appOtaCancelFailCount         = 0U;

volatile uint32_t g_appOtaCmdProcessCallCount     = 0U;
volatile uint32_t g_appOtaCmdProcessedCount       = 0U;
volatile uint32_t g_appOtaCmdStartProcessedCount  = 0U;
volatile uint32_t g_appOtaCmdStartNoCrcProcessedCount = 0U;
volatile uint32_t g_appOtaCmdBlockProcessedCount  = 0U;
volatile uint32_t g_appOtaCmdSetFinalCrcProcessedCount = 0U;
volatile uint32_t g_appOtaCmdCancelProcessedCount = 0U;

volatile uint32_t g_appOtaLastCmdType             = 0U;
volatile uint32_t g_appOtaLastCmdBlockIndex       = 0U;
volatile uint8_t  g_appOtaLastCmdBlockLength      = 0U;
volatile uint32_t g_appOtaLastCmdFirmwareSize     = 0U;
volatile uint32_t g_appOtaLastCmdFirmwareCrc32    = 0U;

volatile uint32_t g_appOtaGatewayStartOkCount     = 0U;
volatile uint32_t g_appOtaGatewayStartFailCount   = 0U;

volatile uint32_t g_appOtaGatewayStartNoCrcOkCount   = 0U;
volatile uint32_t g_appOtaGatewayStartNoCrcFailCount = 0U;

volatile uint32_t g_appOtaGatewayBlockOkCount     = 0U;
volatile uint32_t g_appOtaGatewayBlockFailCount   = 0U;

volatile uint32_t g_appOtaGatewaySetFinalCrcOkCount   = 0U;
volatile uint32_t g_appOtaGatewaySetFinalCrcFailCount = 0U;


/* ============================================================
   Debug trigger variables
   ============================================================ */

/*
 * 사용법:
 *  - Watch에서 g_appOtaDebugStartRequest = 1 로 변경
 *    → App_OtaGateway task 문맥에서 AppOtaGateway_RequestDownload() 호출
 *
 *  - Watch에서 g_appOtaDebugStartNoCrcRequest = 1 로 변경
 *    → App_OtaGateway task 문맥에서 AppOtaGateway_RequestDownloadWithoutCrc() 호출
 *
 *  - WAIT_BLOCK 상태 진입 후 Watch에서 g_appOtaDebugBlock0Request = 1 로 변경
 *    → App_OtaGateway task 문맥에서 AppOtaGateway_ProvideBlock() 호출
 *
 *  - WAIT_FINAL_CRC 상태 진입 후 Watch에서 g_appOtaDebugFinalCrcRequest = 1 로 변경
 *    → App_OtaGateway task 문맥에서 AppOtaGateway_SetFinalCrc() 호출
 *
 * Watch에서 변수 변경이 어려우면 AUTO_TEST를 사용한다.
 */

volatile uint32_t g_appOtaDebugStartRequest    = 0U;
volatile uint32_t g_appOtaDebugStartSize       = 64U;
volatile uint32_t g_appOtaDebugStartCrc32      = 0x778EB6E5U;
volatile uint32_t g_appOtaDebugStartDoneCount  = 0U;
volatile BaseType_t g_appOtaDebugStartResult   = pdFAIL;

volatile uint32_t g_appOtaDebugStartNoCrcRequest   = 0U;
volatile uint32_t g_appOtaDebugStartNoCrcDoneCount = 0U;
volatile BaseType_t g_appOtaDebugStartNoCrcResult  = pdFAIL;

volatile uint32_t g_appOtaDebugBlock0Request   = 0U;
volatile uint32_t g_appOtaDebugBlock0DoneCount = 0U;
volatile BaseType_t g_appOtaDebugBlock0Result  = pdFAIL;
volatile uint32_t g_appOtaDebugBlockIndex      = 0U;
volatile uint8_t  g_appOtaDebugBlockLength     = UDS_OTA_CLIENT_TRANSFER_DATA_SIZE;

volatile uint32_t g_appOtaDebugFinalCrcRequest   = 0U;
volatile uint32_t g_appOtaDebugFinalCrcDoneCount = 0U;
volatile BaseType_t g_appOtaDebugFinalCrcResult  = pdFAIL;
volatile uint32_t g_appOtaDebugFinalCrc32        = 0x778EB6E5U;

volatile uint32_t g_appOtaDebugCancelRequest   = 0U;
volatile uint32_t g_appOtaDebugCancelDoneCount = 0U;
volatile BaseType_t g_appOtaDebugCancelResult  = pdFAIL;

/*
 * 테스트용 32-byte block.
 * ZCU는 전체 bin을 저장하지 않고, 이 block 하나만 queue에 복사해서 넘긴다.
 */
static uint8_t g_appOtaDebugBlockData[UDS_OTA_CLIENT_TRANSFER_DATA_SIZE] =
{
    0x00U, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U,
    0x08U, 0x09U, 0x0AU, 0x0BU, 0x0CU, 0x0DU, 0x0EU, 0x0FU,
    0x10U, 0x11U, 0x12U, 0x13U, 0x14U, 0x15U, 0x16U, 0x17U,
    0x18U, 0x19U, 0x1AU, 0x1BU, 0x1CU, 0x1DU, 0x1EU, 0x1FU
};


/* ============================================================
   Auto test debug variables
   ============================================================ */

volatile uint32_t g_appOtaAutoTestLoopCount          = 0U;
volatile uint32_t g_appOtaAutoStartRequestCount      = 0U;
volatile BaseType_t g_appOtaAutoStartResult          = pdFAIL;

volatile uint32_t g_appOtaAutoBlockRequestCount      = 0U;
volatile BaseType_t g_appOtaAutoBlockResult          = pdFAIL;
volatile uint32_t g_appOtaAutoLastProvidedBlockIndex = 0xFFFFFFFFU;


/* ============================================================
   Private prototypes
   ============================================================ */

static void AppOtaGateway_Task(void *arg);
static void AppOtaGateway_ProcessCanResponses(void);
static void AppOtaGateway_ProcessCommands(void);
static void AppOtaGateway_ProcessDebugTrigger(void);
static void AppOtaGateway_ProcessAutoTest(void);


/* ============================================================
   Public API
   ============================================================ */

BaseType_t AppOtaGateway_Start(void)
{
    BaseType_t result;

    g_appOtaStartCallCount++;

    /*
     * Gateway가 내부적으로 UdsOtaClient까지 초기화한다.
     *
     * 주의:
     *  - AppCan_Start() 이후에 호출해야 한다.
     *  - App_OtaGateway는 AppCan_RecvById()를 사용하기 때문이다.
     */
    OtaGateway_Init();

    g_appOtaCmdQueue = xQueueCreate(APP_OTA_GATEWAY_CMD_QUEUE_LENGTH,
                                    sizeof(AppOtaGateway_Command_t));

    if(g_appOtaCmdQueue == NULL)
    {
        g_appOtaCmdQueueCreateFailCount++;
        g_appOtaLastStartResult = pdFAIL;
        g_appOtaStartFailCount++;
        return pdFAIL;
    }

    g_appOtaCmdQueueCreateOkCount++;

    result = xTaskCreate(AppOtaGateway_Task,
                         "APP OTA",
                         APP_OTA_GATEWAY_TASK_STACK_SIZE,
                         NULL,
                         APP_OTA_GATEWAY_TASK_PRIORITY,
                         NULL);

    g_appOtaLastStartResult = result;

    if(result == pdPASS)
    {
        g_appOtaStartOkCount++;
    }
    else
    {
        g_appOtaStartFailCount++;
    }

    return result;
}


BaseType_t AppOtaGateway_RequestDownload(uint32_t firmwareSize,
                                         uint32_t firmwareCrc32,
                                         TickType_t waitTicks)
{
    AppOtaGateway_Command_t cmd;
    BaseType_t result;

    g_appOtaReqDownloadCallCount++;

    if((g_appOtaCmdQueue == NULL) || (firmwareSize == 0U))
    {
        g_appOtaReqDownloadFailCount++;
        return pdFAIL;
    }

    memset(&cmd, 0, sizeof(cmd));

    cmd.type = APP_OTA_GATEWAY_CMD_START_DOWNLOAD;
    cmd.firmwareSize = firmwareSize;
    cmd.firmwareCrc32 = firmwareCrc32;

    result = xQueueSend(g_appOtaCmdQueue, &cmd, waitTicks);

    if(result == pdPASS)
    {
        g_appOtaReqDownloadQueuedCount++;
    }
    else
    {
        g_appOtaReqDownloadFailCount++;
    }

    return result;
}


BaseType_t AppOtaGateway_RequestDownloadWithoutCrc(uint32_t firmwareSize,
                                                   TickType_t waitTicks)
{
    AppOtaGateway_Command_t cmd;
    BaseType_t result;

    g_appOtaReqDownloadNoCrcCallCount++;

    if((g_appOtaCmdQueue == NULL) || (firmwareSize == 0U))
    {
        g_appOtaReqDownloadNoCrcFailCount++;
        return pdFAIL;
    }

    memset(&cmd, 0, sizeof(cmd));

    cmd.type = APP_OTA_GATEWAY_CMD_START_DOWNLOAD_NO_CRC;
    cmd.firmwareSize = firmwareSize;
    cmd.firmwareCrc32 = 0U;

    result = xQueueSend(g_appOtaCmdQueue, &cmd, waitTicks);

    if(result == pdPASS)
    {
        g_appOtaReqDownloadNoCrcQueuedCount++;
    }
    else
    {
        g_appOtaReqDownloadNoCrcFailCount++;
    }

    return result;
}


BaseType_t AppOtaGateway_SetFinalCrc(uint32_t firmwareCrc32,
                                     TickType_t waitTicks)
{
    AppOtaGateway_Command_t cmd;
    BaseType_t result;

    g_appOtaSetFinalCrcCallCount++;

    if(g_appOtaCmdQueue == NULL)
    {
        g_appOtaSetFinalCrcFailCount++;
        return pdFAIL;
    }

    memset(&cmd, 0, sizeof(cmd));

    cmd.type = APP_OTA_GATEWAY_CMD_SET_FINAL_CRC;
    cmd.firmwareCrc32 = firmwareCrc32;

    result = xQueueSend(g_appOtaCmdQueue, &cmd, waitTicks);

    if(result == pdPASS)
    {
        g_appOtaSetFinalCrcQueuedCount++;
    }
    else
    {
        g_appOtaSetFinalCrcFailCount++;
    }

    return result;
}


BaseType_t AppOtaGateway_ProvideBlock(uint32_t blockIndex,
                                      const uint8_t *data,
                                      uint8_t length,
                                      TickType_t waitTicks)
{
    AppOtaGateway_Command_t cmd;
    BaseType_t result;

    g_appOtaProvideBlockCallCount++;

    if((g_appOtaCmdQueue == NULL) ||
       (data == NULL_PTR) ||
       (length == 0U) ||
       (length > UDS_OTA_CLIENT_TRANSFER_DATA_SIZE))
    {
        g_appOtaProvideBlockFailCount++;
        return pdFAIL;
    }

    memset(&cmd, 0, sizeof(cmd));

    cmd.type = APP_OTA_GATEWAY_CMD_PROVIDE_BLOCK;
    cmd.blockIndex = blockIndex;
    cmd.blockLength = length;
    memcpy(cmd.blockData, data, length);

    result = xQueueSend(g_appOtaCmdQueue, &cmd, waitTicks);

    if(result == pdPASS)
    {
        g_appOtaProvideBlockQueuedCount++;
    }
    else
    {
        g_appOtaProvideBlockFailCount++;
    }

    return result;
}


BaseType_t AppOtaGateway_Cancel(TickType_t waitTicks)
{
    AppOtaGateway_Command_t cmd;
    BaseType_t result;

    g_appOtaCancelCallCount++;

    if(g_appOtaCmdQueue == NULL)
    {
        g_appOtaCancelFailCount++;
        return pdFAIL;
    }

    memset(&cmd, 0, sizeof(cmd));

    cmd.type = APP_OTA_GATEWAY_CMD_CANCEL;

    result = xQueueSend(g_appOtaCmdQueue, &cmd, waitTicks);

    if(result == pdPASS)
    {
        g_appOtaCancelQueuedCount++;
    }
    else
    {
        g_appOtaCancelFailCount++;
    }

    return result;
}

BaseType_t AppOtaGateway_RequestSensorEcuReset(TickType_t waitTicks)
{
    AppOtaGateway_Command_t cmd;
    BaseType_t result;

    if (g_appOtaCmdQueue == NULL)
    {
        return pdFAIL;
    }

    memset(&cmd, 0, sizeof(cmd));
    cmd.type = APP_OTA_GATEWAY_CMD_SENSOR_ECU_RESET;

    result = xQueueSend(g_appOtaCmdQueue, &cmd, waitTicks);

    return result;
}

boolean AppOtaGateway_IsBusy(void)
{
    return OtaGateway_IsBusy();
}


boolean AppOtaGateway_IsWaitingBlock(void)
{
    return OtaGateway_IsWaitingBlock();
}


boolean AppOtaGateway_IsWaitingFinalCrc(void)
{
    return OtaGateway_IsWaitingFinalCrc();
}


boolean AppOtaGateway_IsDone(void)
{
    return OtaGateway_IsDone();
}


boolean AppOtaGateway_IsError(void)
{
    return OtaGateway_IsError();
}


uint32_t AppOtaGateway_GetRequestedBlockIndex(void)
{
    return OtaGateway_GetRequestedBlockIndex();
}


uint32_t AppOtaGateway_GetRequestedOffset(void)
{
    return OtaGateway_GetRequestedOffset();
}


uint8_t AppOtaGateway_GetRequestedLength(void)
{
    return OtaGateway_GetRequestedLength();
}


uint8_t AppOtaGateway_GetProgress(void)
{
    return OtaGateway_GetProgress();
}


/* ============================================================
   Private functions
   ============================================================ */

static void AppOtaGateway_Task(void *arg)
{
    (void)arg;

    for(;;)
    {
        g_appOtaTaskLoopCount++;

        /*
         * 1. Sensor ECU에서 온 0x601 UDS Response 처리
         * 2. Debug trigger 처리
         * 3. Auto test 처리
         * 4. Pi/HPC 계층에서 들어온 OTA command 처리
         * 5. OtaGateway/UdsOtaClient 상태머신 진행
         */
        AppOtaGateway_ProcessCanResponses();
        AppOtaGateway_ProcessDebugTrigger();
        AppOtaGateway_ProcessAutoTest();
        AppOtaGateway_ProcessCommands();

        OtaGateway_MainFunction();
        g_appOtaMainCallCount++;

        vTaskDelay(pdMS_TO_TICKS(APP_OTA_GATEWAY_TASK_PERIOD_MS));
    }
}


static void AppOtaGateway_ProcessCanResponses(void)
{
    AppCanFrame frame;
    uint32_t i;
    uint8_t j;

    g_appOtaProcessRxCallCount++;

    for(i = 0U; i < APP_OTA_GATEWAY_RX_DRAIN_LIMIT; i++)
    {
        if(AppCan_RecvById(CAN_ID_OTA_RESPONSE, &frame, 0U) != pdPASS)
        {
            /*
             * queue가 비어있으면 여기로 들어온다.
             * 1ms마다 자주 증가하는 것이 정상이다.
             */
            g_appOtaRxEmptyCount++;
            break;
        }

        g_appOtaRx601Count++;

        g_appOtaLastRxId = frame.id;
        g_appOtaLastRxLength = frame.length;
        g_appOtaLastRxIsFd = frame.is_fd;

        if(frame.length > 0U)
        {
            g_appOtaLastRxSid = frame.data[0];
        }
        else
        {
            g_appOtaLastRxSid = 0U;
        }

        for(j = 0U; j < 8U; j++)
        {
            if(j < frame.length)
            {
                g_appOtaLastRxData[j] = frame.data[j];
            }
            else
            {
                g_appOtaLastRxData[j] = 0U;
            }
        }

        /*
         * App_Can은 CAN ID 의미를 해석하지 않는다.
         * 여기서 0x601 payload를 UdsOtaClient에 넘긴다.
         */
        UdsOtaClient_OnResponse(frame.data, frame.length);
        g_appOtaOnResponseCallCount++;
    }
}


static void AppOtaGateway_ProcessDebugTrigger(void)
{
    if(g_appOtaDebugStartRequest != 0U)
    {
        /*
         * 한 번만 실행되도록 먼저 0으로 내린다.
         */
        g_appOtaDebugStartRequest = 0U;

        g_appOtaDebugStartResult =
            AppOtaGateway_RequestDownload(g_appOtaDebugStartSize,
                                          g_appOtaDebugStartCrc32,
                                          0U);

        g_appOtaDebugStartDoneCount++;
    }

    if(g_appOtaDebugStartNoCrcRequest != 0U)
    {
        /*
         * 한 번만 실행되도록 먼저 0으로 내린다.
         */
        g_appOtaDebugStartNoCrcRequest = 0U;

        g_appOtaDebugStartNoCrcResult =
            AppOtaGateway_RequestDownloadWithoutCrc(g_appOtaDebugStartSize,
                                                    0U);

        g_appOtaDebugStartNoCrcDoneCount++;
    }

    if(g_appOtaDebugBlock0Request != 0U)
    {
        /*
         * 한 번만 실행되도록 먼저 0으로 내린다.
         */
        g_appOtaDebugBlock0Request = 0U;

        g_appOtaDebugBlock0Result =
            AppOtaGateway_ProvideBlock(g_appOtaDebugBlockIndex,
                                       g_appOtaDebugBlockData,
                                       g_appOtaDebugBlockLength,
                                       0U);

        g_appOtaDebugBlock0DoneCount++;
    }

    if(g_appOtaDebugFinalCrcRequest != 0U)
    {
        /*
         * 한 번만 실행되도록 먼저 0으로 내린다.
         */
        g_appOtaDebugFinalCrcRequest = 0U;

        g_appOtaDebugFinalCrcResult =
            AppOtaGateway_SetFinalCrc(g_appOtaDebugFinalCrc32,
                                      0U);

        g_appOtaDebugFinalCrcDoneCount++;
    }

    if(g_appOtaDebugCancelRequest != 0U)
    {
        /*
         * 한 번만 실행되도록 먼저 0으로 내린다.
         */
        g_appOtaDebugCancelRequest = 0U;

        g_appOtaDebugCancelResult = AppOtaGateway_Cancel(0U);
        g_appOtaDebugCancelDoneCount++;
    }
}


static void AppOtaGateway_ProcessAutoTest(void)
{
#if (APP_OTA_GATEWAY_AUTO_TEST_ENABLE == 1u)
    uint32_t requestedBlockIndex;
    uint8_t requestedLength;

    g_appOtaAutoTestLoopCount++;

    /*
     * 1단계:
     * 부팅 후 일정 loop가 지나면 OTA_START를 자동으로 1회 요청한다.
     *
     * 기대 결과:
     *  - g_appOtaAutoStartRequestCount = 1
     *  - g_appOtaAutoStartResult = pdPASS
     *  - PCAN에서 0x600 / 10 02 확인
     */
    if((g_appOtaAutoStartRequestCount == 0U) &&
       (g_appOtaAutoTestLoopCount >= APP_OTA_GATEWAY_AUTO_START_DELAY_LOOPS))
    {
        g_appOtaAutoStartResult =
            AppOtaGateway_RequestDownload(APP_OTA_GATEWAY_AUTO_TEST_SIZE,
                                          APP_OTA_GATEWAY_AUTO_TEST_CRC32,
                                          0U);

        g_appOtaAutoStartRequestCount++;
    }

    /*
     * 2단계:
     * UdsOtaClient가 WAIT_BLOCK 상태가 되면 현재 요청 block을 자동 제공한다.
     *
     * firmwareSize=64이면 block 0, block 1 두 번 제공된다.
     */
    if(OtaGateway_IsWaitingBlock() == TRUE)
    {
        requestedBlockIndex = OtaGateway_GetRequestedBlockIndex();
        requestedLength = OtaGateway_GetRequestedLength();

        if((requestedLength > 0U) &&
           (requestedLength <= UDS_OTA_CLIENT_TRANSFER_DATA_SIZE) &&
           (g_appOtaAutoLastProvidedBlockIndex != requestedBlockIndex))
        {
            g_appOtaAutoBlockResult =
                AppOtaGateway_ProvideBlock(requestedBlockIndex,
                                           g_appOtaDebugBlockData,
                                           requestedLength,
                                           0U);

            g_appOtaAutoLastProvidedBlockIndex = requestedBlockIndex;
            g_appOtaAutoBlockRequestCount++;
        }
    }
#endif
}


static void AppOtaGateway_ProcessCommands(void)
{
    AppOtaGateway_Command_t cmd;
    uint32_t i;
    OtaGateway_Result_t gatewayResult;

    g_appOtaCmdProcessCallCount++;

    if(g_appOtaCmdQueue == NULL)
    {
        return;
    }

    for(i = 0U; i < APP_OTA_GATEWAY_CMD_QUEUE_LENGTH; i++)
    {
        if(xQueueReceive(g_appOtaCmdQueue, &cmd, 0U) != pdPASS)
        {
            break;
        }

        g_appOtaCmdProcessedCount++;
        g_appOtaLastCmdType = (uint32_t)cmd.type;

        switch(cmd.type)
        {
            case APP_OTA_GATEWAY_CMD_START_DOWNLOAD:
            {
                g_appOtaCmdStartProcessedCount++;
                g_appOtaLastCmdFirmwareSize = cmd.firmwareSize;
                g_appOtaLastCmdFirmwareCrc32 = cmd.firmwareCrc32;

                gatewayResult = OtaGateway_Start(cmd.firmwareSize,
                                                 cmd.firmwareCrc32);

                if(gatewayResult == OTA_GATEWAY_RESULT_OK)
                {
                    g_appOtaGatewayStartOkCount++;
                }
                else
                {
                    g_appOtaGatewayStartFailCount++;
                }

                break;
            }

            case APP_OTA_GATEWAY_CMD_START_DOWNLOAD_NO_CRC:
            {
                g_appOtaCmdStartNoCrcProcessedCount++;
                g_appOtaLastCmdFirmwareSize = cmd.firmwareSize;
                g_appOtaLastCmdFirmwareCrc32 = 0U;

                gatewayResult = OtaGateway_StartWithoutCrc(cmd.firmwareSize);

                if(gatewayResult == OTA_GATEWAY_RESULT_OK)
                {
                    g_appOtaGatewayStartNoCrcOkCount++;
                }
                else
                {
                    g_appOtaGatewayStartNoCrcFailCount++;
                }

                break;
            }

            case APP_OTA_GATEWAY_CMD_PROVIDE_BLOCK:
            {
                g_appOtaCmdBlockProcessedCount++;
                g_appOtaLastCmdBlockIndex = cmd.blockIndex;
                g_appOtaLastCmdBlockLength = cmd.blockLength;

                gatewayResult = OtaGateway_ProvideBlock(cmd.blockIndex,
                                                        cmd.blockData,
                                                        cmd.blockLength);

                if(gatewayResult == OTA_GATEWAY_RESULT_OK)
                {
                    g_appOtaGatewayBlockOkCount++;
                }
                else
                {
                    g_appOtaGatewayBlockFailCount++;
                }

                break;
            }

            case APP_OTA_GATEWAY_CMD_SET_FINAL_CRC:
            {
                g_appOtaCmdSetFinalCrcProcessedCount++;
                g_appOtaLastCmdFirmwareCrc32 = cmd.firmwareCrc32;

                gatewayResult = OtaGateway_SetFinalCrc(cmd.firmwareCrc32);

                if(gatewayResult == OTA_GATEWAY_RESULT_OK)
                {
                    g_appOtaGatewaySetFinalCrcOkCount++;
                }
                else
                {
                    g_appOtaGatewaySetFinalCrcFailCount++;
                }

                break;
            }

            case APP_OTA_GATEWAY_CMD_CANCEL:
            {
                g_appOtaCmdCancelProcessedCount++;
                (void)OtaGateway_Cancel();
                break;
            }
            case APP_OTA_GATEWAY_CMD_SENSOR_ECU_RESET:
            {
                UdsOtaClient_Result_t resetResult;

                resetResult = UdsOtaClient_RequestEcuReset();

                if (resetResult != UDS_OTA_CLIENT_RESULT_OK)
                {
                }

                break;
            }

            default:
            {
                break;
            }
        }
    }
}
