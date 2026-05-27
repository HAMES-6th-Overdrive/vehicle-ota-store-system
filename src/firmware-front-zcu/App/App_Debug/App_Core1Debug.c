#include "App_Core1Debug.h"
#include "App_Debug/App_Debug.h"
#include "IfxCpu.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define APP_CORE1DEBUG_TASK_STACK_SIZE     (512u)
#define APP_CORE1DEBUG_TASK_PRIORITY       (tskIDLE_PRIORITY + 1u)
#define APP_CORE1DEBUG_TASK_PERIOD_MS      (10u)

typedef struct
{
    char msg[APP_CORE1DEBUG_MSG_MAX_LEN];
} AppCore1DebugItem;

typedef struct
{
    volatile uint32 head;       /* Core1 write index */
    volatile uint32 tail;       /* Core0 read index */
    volatile uint32 dropped;    /* overflow count */
    volatile uint32 initialized;

    AppCore1DebugItem items[APP_CORE1DEBUG_QUEUE_DEPTH];
} AppCore1DebugBuffer;

/*
 * 가능하면 LMU RAM 섹션에 배치하는 것을 추천.
 * 링커 스크립트에 .lmu_data가 없다면 일단 일반 global로 둬도 빌드는 가능하지만,
 * Core0/Core1 공유 목적이면 LMU 배치가 가장 안전하다.
 */
#if defined(__TASKING__)
#pragma section farbss "lmu_data"
#endif

AppCore1DebugBuffer g_Core1Debug;

#if defined(__TASKING__)
#pragma section farbss restore
#endif

static void AppCore1Debug_Task(void *arg);
static void AppCore1Debug_CopyString(char *dst, const char *src, uint32 maxLen);
static void AppCore1Debug_U32ToDec(char *dst, uint32 maxLen, uint32 value);

void AppCore1Debug_Init(void)
{
    uint32 i;

    g_Core1Debug.head = 0u;
    g_Core1Debug.tail = 0u;
    g_Core1Debug.dropped = 0u;

    for (i = 0u; i < APP_CORE1DEBUG_QUEUE_DEPTH; i++)
    {
        g_Core1Debug.items[i].msg[0] = '\0';
    }

    __dsync();

    g_Core1Debug.initialized = 1u;

    __dsync();
}

void AppCore1Debug_StartTask(void)
{
    (void)xTaskCreate(AppCore1Debug_Task,
                      "C1 LOG",
                      APP_CORE1DEBUG_TASK_STACK_SIZE,
                      NULL,
                      APP_CORE1DEBUG_TASK_PRIORITY,
                      NULL);
}

void AppCore1Debug_Push(const char *msg)
{
    uint32 head;
    uint32 nextHead;

    if ((msg == NULL) || (g_Core1Debug.initialized != 1u))
    {
        return;
    }

    head = g_Core1Debug.head;
    nextHead = (head + 1u) % APP_CORE1DEBUG_QUEUE_DEPTH;

    /*
     * Queue full.
     * Core1에서는 절대 block 하지 않는다.
     * Flash/UCB 작업 중 로그 때문에 멈추면 안 되므로 drop만 카운트.
     */
    if (nextHead == g_Core1Debug.tail)
    {
        g_Core1Debug.dropped++;
        return;
    }

    AppCore1Debug_CopyString(g_Core1Debug.items[head].msg,
                           msg,
                           APP_CORE1DEBUG_MSG_MAX_LEN);

    /*
     * 문자열을 먼저 다 쓴 뒤 head를 갱신해야 Core0이 반쯤 쓴 문자열을 읽지 않는다.
     */
    __dsync();

    g_Core1Debug.head = nextHead;

    __dsync();
}

void AppCore1Debug_PushU32(const char *prefix, uint32 value)
{
    char buf[APP_CORE1DEBUG_MSG_MAX_LEN];
    char num[16];
    uint32 i = 0u;
    uint32 j = 0u;

    if (prefix == NULL)
    {
        return;
    }

    while ((prefix[i] != '\0') && (i < (APP_CORE1DEBUG_MSG_MAX_LEN - 1u)))
    {
        buf[i] = prefix[i];
        i++;
    }

    AppCore1Debug_U32ToDec(num, sizeof(num), value);

    while ((num[j] != '\0') && (i < (APP_CORE1DEBUG_MSG_MAX_LEN - 1u)))
    {
        buf[i] = num[j];
        i++;
        j++;
    }

    buf[i] = '\0';

    AppCore1Debug_Push(buf);
}

void AppCore1Debug_DrainOnce(void)
{
    char localMsg[APP_CORE1DEBUG_MSG_MAX_LEN];
    uint32 tail;
    uint32 dropped;

    if (g_Core1Debug.initialized != 1u)
    {
        return;
    }

    while (g_Core1Debug.tail != g_Core1Debug.head)
    {
        tail = g_Core1Debug.tail;

        /*
         * Core0 로컬 버퍼로 먼저 복사.
         * 그 다음 tail 이동.
         */
        AppCore1Debug_CopyString(localMsg,
                               g_Core1Debug.items[tail].msg,
                               APP_CORE1DEBUG_MSG_MAX_LEN);

        __dsync();

        g_Core1Debug.tail = (tail + 1u) % APP_CORE1DEBUG_QUEUE_DEPTH;

        __dsync();

        AppDebug_Print_C1("[C1] %s", localMsg);
    }

    dropped = g_Core1Debug.dropped;
    if (dropped != 0u)
    {
        g_Core1Debug.dropped = 0u;
        AppDebug_Print_C1("[C1] log dropped=%lu", dropped);
    }
}

static void AppCore1Debug_Task(void *arg)
{
    (void)arg;

    for (;;)
    {
        AppCore1Debug_DrainOnce();
        vTaskDelay(pdMS_TO_TICKS(APP_CORE1DEBUG_TASK_PERIOD_MS));
    }
}

static void AppCore1Debug_CopyString(char *dst, const char *src, uint32 maxLen)
{
    uint32 i;

    if ((dst == NULL) || (src == NULL) || (maxLen == 0u))
    {
        return;
    }

    for (i = 0u; i < (maxLen - 1u); i++)
    {
        dst[i] = src[i];

        if (src[i] == '\0')
        {
            return;
        }
    }

    dst[maxLen - 1u] = '\0';
}

static void AppCore1Debug_U32ToDec(char *dst, uint32 maxLen, uint32 value)
{
    char temp[16];
    uint32 i = 0u;
    uint32 j = 0u;

    if ((dst == NULL) || (maxLen == 0u))
    {
        return;
    }

    if (value == 0u)
    {
        if (maxLen >= 2u)
        {
            dst[0] = '0';
            dst[1] = '\0';
        }
        return;
    }

    while ((value > 0u) && (i < sizeof(temp)))
    {
        temp[i] = (char)('0' + (value % 10u));
        value /= 10u;
        i++;
    }

    while ((i > 0u) && (j < (maxLen - 1u)))
    {
        i--;
        dst[j] = temp[i];
        j++;
    }

    dst[j] = '\0';
}
