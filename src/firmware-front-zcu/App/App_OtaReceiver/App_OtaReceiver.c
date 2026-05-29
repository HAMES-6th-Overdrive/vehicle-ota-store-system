/**********************************************************************************************************************
 * \file App_OtaReceiver.c
 * \brief ZCU OTA input adapter layer with temporary self-test
 *
 * 역할:
 *  - 외부 입력 계층에서 받은 OTA_START / OTA_BLOCK / OTA_FINAL_CRC 요청을 App_OtaGateway로 전달한다.
 *  - 현재 단계에서는 UART/SOMEIP/Ethernet/DoIP 파싱은 하지 않는다.
 *
 * 현재 단계:
 *  - App_OtaReceiver -> App_OtaGateway -> OtaGateway -> UdsOtaClient 경로 제공
 *  - SelfTest는 기본 OFF
 *
 * 주의:
 *  - App_OtaGateway 내부 AutoTest는 반드시 꺼야 한다.
 *      APP_OTA_GATEWAY_AUTO_TEST_ENABLE = 0
 *
 *  - 이 파일의 SelfTest는 테스트 완료 후 반드시 꺼두는 것을 권장한다.
 *      APP_OTA_RECEIVER_SELF_TEST_ENABLE = 0
 *********************************************************************************************************************/

#include "App_OtaReceiver.h"

#include "App_OtaGateway/App_OtaGateway.h"

#include "task.h"

#include <string.h>

/* ============================================================
   Self test config
   ============================================================ */

/*
 * App_OtaReceiver 경로 검증용 임시 self-test.
 *
 * 1U:
 *  - AppOtaReceiver_Init()에서 self-test task 생성
 *  - 일정 delay 후 AppOtaReceiver_StartDownload() 호출
 *  - Gateway가 WAIT_BLOCK이면 AppOtaReceiver_ProvideBlock() 호출
 *
 * 0U:
 *  - 실제 Pi/HPC, UART, SOME/IP, DoIP 입력 계층을 사용할 때는 반드시 꺼둔다.
 */
#define APP_OTA_RECEIVER_SELF_TEST_ENABLE          (0u)
#define APP_OTA_RECEIVER_SELF_TEST_DELAY_MS        (1000u)
#define APP_OTA_RECEIVER_SELF_TEST_PERIOD_MS       (5u)

#define APP_OTA_RECEIVER_SELF_TEST_SIZE            (64u)
#define APP_OTA_RECEIVER_SELF_TEST_CRC32           (0x778EB6E5u)

#define APP_OTA_RECEIVER_SELF_TEST_STACK_SIZE      (configMINIMAL_STACK_SIZE + 128u)
#define APP_OTA_RECEIVER_SELF_TEST_PRIORITY        (tskIDLE_PRIORITY + 1u)

#define APP_OTA_RECEIVER_TEST_BLOCK_SIZE           (32u)


/* ============================================================
   Private variables
   ============================================================ */

static boolean g_appOtaReceiverInitialized = FALSE;

#if (APP_OTA_RECEIVER_SELF_TEST_ENABLE == 1u)
static boolean g_appOtaReceiverSelfTestTaskCreated = FALSE;
#endif


/* ============================================================
   Debug variables
   ============================================================ */

volatile uint32_t g_appOtaReceiverInitCount = 0U;

/* CRC known start */
volatile uint32_t g_appOtaReceiverStartCallCount = 0U;
volatile uint32_t g_appOtaReceiverStartOkCount = 0U;
volatile uint32_t g_appOtaReceiverStartFailCount = 0U;

/* Late CRC start */
volatile uint32_t g_appOtaReceiverStartNoCrcCallCount = 0U;
volatile uint32_t g_appOtaReceiverStartNoCrcOkCount = 0U;
volatile uint32_t g_appOtaReceiverStartNoCrcFailCount = 0U;

/* Final CRC */
volatile uint32_t g_appOtaReceiverFinalCrcCallCount = 0U;
volatile uint32_t g_appOtaReceiverFinalCrcOkCount = 0U;
volatile uint32_t g_appOtaReceiverFinalCrcFailCount = 0U;

volatile uint32_t g_appOtaReceiverLastFirmwareSize = 0U;
volatile uint32_t g_appOtaReceiverLastFirmwareCrc32 = 0U;

volatile uint32_t g_appOtaReceiverBlockCallCount = 0U;
volatile uint32_t g_appOtaReceiverBlockOkCount = 0U;
volatile uint32_t g_appOtaReceiverBlockFailCount = 0U;
volatile uint32_t g_appOtaReceiverLastBlockIndex = 0U;
volatile uint8_t  g_appOtaReceiverLastBlockLength = 0U;

volatile uint32_t g_appOtaReceiverCancelCallCount = 0U;
volatile uint32_t g_appOtaReceiverCancelOkCount = 0U;
volatile uint32_t g_appOtaReceiverCancelFailCount = 0U;

volatile BaseType_t g_appOtaReceiverLastResult = pdFAIL;


/* ============================================================
   Self test debug variables
   ============================================================ */

volatile uint32_t g_appOtaReceiverSelfTestTaskCreateOkCount = 0U;
volatile uint32_t g_appOtaReceiverSelfTestTaskCreateFailCount = 0U;

volatile uint32_t g_appOtaReceiverSelfTestLoopCount = 0U;

volatile uint32_t g_appOtaReceiverSelfTestStartRequestCount = 0U;
volatile uint32_t g_appOtaReceiverSelfTestStartOkCount = 0U;
volatile uint32_t g_appOtaReceiverSelfTestStartFailCount = 0U;
volatile BaseType_t g_appOtaReceiverSelfTestStartResult = pdFAIL;

volatile uint32_t g_appOtaReceiverSelfTestBlockAttemptCount = 0U;
volatile uint32_t g_appOtaReceiverSelfTestBlockOkCount = 0U;
volatile uint32_t g_appOtaReceiverSelfTestBlockFailCount = 0U;
volatile BaseType_t g_appOtaReceiverSelfTestBlockResult = pdFAIL;
volatile uint32_t g_appOtaReceiverSelfTestLastBlockIndex = 0xFFFFFFFFU;
volatile uint8_t  g_appOtaReceiverSelfTestLastBlockLength = 0U;


/*
 * 테스트용 32-byte block.
 *
 * SelfTest가 꺼져 있을 때 warning을 피하기 위해 #if로 감싼다.
 */
#if (APP_OTA_RECEIVER_SELF_TEST_ENABLE == 1u)

static uint8_t g_appOtaReceiverSelfTestBlockData[APP_OTA_RECEIVER_TEST_BLOCK_SIZE] =
{
    0x00U, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U, 0x06U, 0x07U,
    0x08U, 0x09U, 0x0AU, 0x0BU, 0x0CU, 0x0DU, 0x0EU, 0x0FU,
    0x10U, 0x11U, 0x12U, 0x13U, 0x14U, 0x15U, 0x16U, 0x17U,
    0x18U, 0x19U, 0x1AU, 0x1BU, 0x1CU, 0x1DU, 0x1EU, 0x1FU
};

#endif


/* ============================================================
   Private prototypes
   ============================================================ */

#if (APP_OTA_RECEIVER_SELF_TEST_ENABLE == 1u)
static void AppOtaReceiver_SelfTestTask(void *arg);
#endif


/* ============================================================
   Public API
   ============================================================ */

void AppOtaReceiver_Init(void)
{
    g_appOtaReceiverInitCount++;

    g_appOtaReceiverStartCallCount = 0U;
    g_appOtaReceiverStartOkCount = 0U;
    g_appOtaReceiverStartFailCount = 0U;

    g_appOtaReceiverStartNoCrcCallCount = 0U;
    g_appOtaReceiverStartNoCrcOkCount = 0U;
    g_appOtaReceiverStartNoCrcFailCount = 0U;

    g_appOtaReceiverFinalCrcCallCount = 0U;
    g_appOtaReceiverFinalCrcOkCount = 0U;
    g_appOtaReceiverFinalCrcFailCount = 0U;

    g_appOtaReceiverLastFirmwareSize = 0U;
    g_appOtaReceiverLastFirmwareCrc32 = 0U;

    g_appOtaReceiverBlockCallCount = 0U;
    g_appOtaReceiverBlockOkCount = 0U;
    g_appOtaReceiverBlockFailCount = 0U;
    g_appOtaReceiverLastBlockIndex = 0U;
    g_appOtaReceiverLastBlockLength = 0U;

    g_appOtaReceiverCancelCallCount = 0U;
    g_appOtaReceiverCancelOkCount = 0U;
    g_appOtaReceiverCancelFailCount = 0U;

    g_appOtaReceiverLastResult = pdFAIL;

    g_appOtaReceiverInitialized = TRUE;

#if (APP_OTA_RECEIVER_SELF_TEST_ENABLE == 1u)
    if(g_appOtaReceiverSelfTestTaskCreated == FALSE)
    {
        BaseType_t taskResult;

        taskResult = xTaskCreate(AppOtaReceiver_SelfTestTask,
                                 "OTA RX TEST",
                                 APP_OTA_RECEIVER_SELF_TEST_STACK_SIZE,
                                 NULL,
                                 APP_OTA_RECEIVER_SELF_TEST_PRIORITY,
                                 NULL);

        if(taskResult == pdPASS)
        {
            g_appOtaReceiverSelfTestTaskCreateOkCount++;
            g_appOtaReceiverSelfTestTaskCreated = TRUE;
        }
        else
        {
            g_appOtaReceiverSelfTestTaskCreateFailCount++;
        }
    }
#endif
}


BaseType_t AppOtaReceiver_StartDownload(uint32_t firmwareSize,
                                        uint32_t firmwareCrc32,
                                        TickType_t waitTicks)
{
    BaseType_t result;

    g_appOtaReceiverStartCallCount++;
    g_appOtaReceiverLastFirmwareSize = firmwareSize;
    g_appOtaReceiverLastFirmwareCrc32 = firmwareCrc32;

    if((g_appOtaReceiverInitialized == FALSE) || (firmwareSize == 0U))
    {
        g_appOtaReceiverStartFailCount++;
        g_appOtaReceiverLastResult = pdFAIL;
        return pdFAIL;
    }

    result = AppOtaGateway_RequestDownload(firmwareSize,
                                           firmwareCrc32,
                                           waitTicks);

    g_appOtaReceiverLastResult = result;

    if(result == pdPASS)
    {
        g_appOtaReceiverStartOkCount++;
    }
    else
    {
        g_appOtaReceiverStartFailCount++;
    }

    return result;
}


BaseType_t AppOtaReceiver_StartDownloadWithoutCrc(uint32_t firmwareSize,
                                                  TickType_t waitTicks)
{
    BaseType_t result;

    g_appOtaReceiverStartNoCrcCallCount++;
    g_appOtaReceiverLastFirmwareSize = firmwareSize;
    g_appOtaReceiverLastFirmwareCrc32 = 0U;

    if((g_appOtaReceiverInitialized == FALSE) || (firmwareSize == 0U))
    {
        g_appOtaReceiverStartNoCrcFailCount++;
        g_appOtaReceiverLastResult = pdFAIL;
        return pdFAIL;
    }

    result = AppOtaGateway_RequestDownloadWithoutCrc(firmwareSize,
                                                     waitTicks);

    g_appOtaReceiverLastResult = result;

    if(result == pdPASS)
    {
        g_appOtaReceiverStartNoCrcOkCount++;
    }
    else
    {
        g_appOtaReceiverStartNoCrcFailCount++;
    }

    return result;
}


BaseType_t AppOtaReceiver_SetFinalCrc(uint32_t firmwareCrc32,
                                      TickType_t waitTicks)
{
    BaseType_t result;

    g_appOtaReceiverFinalCrcCallCount++;
    g_appOtaReceiverLastFirmwareCrc32 = firmwareCrc32;

    if(g_appOtaReceiverInitialized == FALSE)
    {
        g_appOtaReceiverFinalCrcFailCount++;
        g_appOtaReceiverLastResult = pdFAIL;
        return pdFAIL;
    }

    result = AppOtaGateway_SetFinalCrc(firmwareCrc32,
                                       waitTicks);

    g_appOtaReceiverLastResult = result;

    if(result == pdPASS)
    {
        g_appOtaReceiverFinalCrcOkCount++;
    }
    else
    {
        g_appOtaReceiverFinalCrcFailCount++;
    }

    return result;
}


BaseType_t AppOtaReceiver_ProvideBlock(uint32_t blockIndex,
                                       const uint8_t *data,
                                       uint8_t length,
                                       TickType_t waitTicks)
{
    BaseType_t result;

    g_appOtaReceiverBlockCallCount++;
    g_appOtaReceiverLastBlockIndex = blockIndex;
    g_appOtaReceiverLastBlockLength = length;

    if((g_appOtaReceiverInitialized == FALSE) ||
       (data == NULL_PTR) ||
       (length == 0U))
    {
        g_appOtaReceiverBlockFailCount++;
        g_appOtaReceiverLastResult = pdFAIL;
        return pdFAIL;
    }

    /*
     * 여기서는 blockIndex/length의 정합성을 강하게 판단하지 않는다.
     *
     * 최종 검증은 아래 계층에서 수행한다.
     *  - App_OtaGateway
     *  - OtaGateway
     *  - UdsOtaClient
     */
    result = AppOtaGateway_ProvideBlock(blockIndex,
                                        data,
                                        length,
                                        waitTicks);

    g_appOtaReceiverLastResult = result;

    if(result == pdPASS)
    {
        g_appOtaReceiverBlockOkCount++;
    }
    else
    {
        g_appOtaReceiverBlockFailCount++;
    }

    return result;
}


BaseType_t AppOtaReceiver_Cancel(TickType_t waitTicks)
{
    BaseType_t result;

    g_appOtaReceiverCancelCallCount++;

    if(g_appOtaReceiverInitialized == FALSE)
    {
        g_appOtaReceiverCancelFailCount++;
        g_appOtaReceiverLastResult = pdFAIL;
        return pdFAIL;
    }

    result = AppOtaGateway_Cancel(waitTicks);

    g_appOtaReceiverLastResult = result;

    if(result == pdPASS)
    {
        g_appOtaReceiverCancelOkCount++;
    }
    else
    {
        g_appOtaReceiverCancelFailCount++;
    }

    return result;
}

BaseType_t AppOtaReceiver_RequestSensorEcuReset(TickType_t waitTicks)
{
    if (g_appOtaReceiverInitialized == FALSE)
    {
        return pdFAIL;
    }
    return AppOtaGateway_RequestSensorEcuReset(waitTicks);
}

/* ============================================================
   Gateway state helper
   ============================================================ */

boolean AppOtaReceiver_IsBusy(void)
{
    return AppOtaGateway_IsBusy();
}


boolean AppOtaReceiver_IsWaitingBlock(void)
{
    return AppOtaGateway_IsWaitingBlock();
}


boolean AppOtaReceiver_IsWaitingFinalCrc(void)
{
    return AppOtaGateway_IsWaitingFinalCrc();
}


boolean AppOtaReceiver_IsDone(void)
{
    return AppOtaGateway_IsDone();
}


boolean AppOtaReceiver_IsError(void)
{
    return AppOtaGateway_IsError();
}


uint32_t AppOtaReceiver_GetRequestedBlockIndex(void)
{
    return AppOtaGateway_GetRequestedBlockIndex();
}


uint32_t AppOtaReceiver_GetRequestedOffset(void)
{
    return AppOtaGateway_GetRequestedOffset();
}


uint8_t AppOtaReceiver_GetRequestedLength(void)
{
    return AppOtaGateway_GetRequestedLength();
}


uint8_t AppOtaReceiver_GetProgress(void)
{
    return AppOtaGateway_GetProgress();
}


/* ============================================================
   Self test task
   ============================================================ */

#if (APP_OTA_RECEIVER_SELF_TEST_ENABLE == 1u)

static void AppOtaReceiver_SelfTestTask(void *arg)
{
    uint32_t requestedBlockIndex;
    uint8_t requestedLength;

    (void)arg;

    /*
     * AppCan / AppOtaGateway task가 먼저 안정적으로 시작되도록 대기.
     */
    vTaskDelay(pdMS_TO_TICKS(APP_OTA_RECEIVER_SELF_TEST_DELAY_MS));

    for(;;)
    {
        g_appOtaReceiverSelfTestLoopCount++;

        /*
         * 1단계:
         * Receiver API를 통해 OTA_START를 1회 요청한다.
         *
         * 기대:
         *  - g_appOtaReceiverSelfTestStartRequestCount = 1
         *  - g_appOtaReceiverSelfTestStartResult = pdPASS
         *  - g_appOtaReceiverStartOkCount = 1
         *  - PCAN에서 0x600 / 10 02 송신 확인
         */
        if(g_appOtaReceiverSelfTestStartRequestCount == 0U)
        {
            g_appOtaReceiverSelfTestStartResult =
                AppOtaReceiver_StartDownload(APP_OTA_RECEIVER_SELF_TEST_SIZE,
                                             APP_OTA_RECEIVER_SELF_TEST_CRC32,
                                             0U);

            g_appOtaReceiverSelfTestStartRequestCount++;

            if(g_appOtaReceiverSelfTestStartResult == pdPASS)
            {
                g_appOtaReceiverSelfTestStartOkCount++;
            }
            else
            {
                g_appOtaReceiverSelfTestStartFailCount++;
            }
        }

        /*
         * 2단계:
         * Gateway가 block을 요청하면 Receiver API를 통해 block 제공.
         *
         * 64 bytes 테스트이므로 block 0, block 1 두 번 제공된다.
         */
        if(AppOtaReceiver_IsWaitingBlock() == TRUE)
        {
            requestedBlockIndex = AppOtaReceiver_GetRequestedBlockIndex();
            requestedLength = AppOtaReceiver_GetRequestedLength();

            if((requestedLength > 0U) &&
               (requestedLength <= APP_OTA_RECEIVER_TEST_BLOCK_SIZE) &&
               (g_appOtaReceiverSelfTestLastBlockIndex != requestedBlockIndex))
            {
                g_appOtaReceiverSelfTestBlockAttemptCount++;

                g_appOtaReceiverSelfTestBlockResult =
                    AppOtaReceiver_ProvideBlock(requestedBlockIndex,
                                                g_appOtaReceiverSelfTestBlockData,
                                                requestedLength,
                                                0U);

                if(g_appOtaReceiverSelfTestBlockResult == pdPASS)
                {
                    g_appOtaReceiverSelfTestLastBlockIndex = requestedBlockIndex;
                    g_appOtaReceiverSelfTestLastBlockLength = requestedLength;
                    g_appOtaReceiverSelfTestBlockOkCount++;
                }
                else
                {
                    g_appOtaReceiverSelfTestBlockFailCount++;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(APP_OTA_RECEIVER_SELF_TEST_PERIOD_MS));
    }
}

#endif
