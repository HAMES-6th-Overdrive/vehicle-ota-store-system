#include "App_Debug.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <UART_Logging.h>
#include "IfxCpu.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "App_Sota/App_Sota.h"

static SemaphoreHandle_t g_print_mutex = NULL;

void AppDebug_Init(void)
{
    initUART();
    g_print_mutex = xSemaphoreCreateMutex();

    SOTA_IsGroupBActive() ? AppDebug_Print("FreeRTOS Bank B Start!") : AppDebug_Print("FreeRTOS Bank A Start!");
    AppDebug_Print("Slow Mode!");
    // AppDebug_Print("Fast Mode!");
}

void AppDebug_Print(const char *format, ...)
{
    char str[MAXCHARS + 1];
    char out[MAXCHARS + 1];

    va_list args;
    int len;
    uint16 cnt;
    const char prefix[] = "[C0] ";

    uint16 prefixLen = 5u;
    uint16 copyLen;
    uint16 i;

    if (format == NULL) return;

    va_start(args, format);
    len = vsnprintf(str, sizeof(str), format, args);
    va_end(args);

    if (len < 0) return;

    if (len > MAXCHARS) copyLen = MAXCHARS;
    else copyLen = (uint16)len;

    if (copyLen > (MAXCHARS - prefixLen)) copyLen = (uint16)(MAXCHARS - prefixLen);

    for (i = 0u; i < prefixLen; i++)
    {
        out[i] = prefix[i];
    }

    for (i = 0u; i < copyLen; i++)
    {
        out[prefixLen + i] = str[i];
    }

    cnt = (uint16)(prefixLen + copyLen);

    if (g_print_mutex == NULL) return;

    if (xSemaphoreTake(g_print_mutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        sendUARTMessage(out, cnt);
        sendUARTMessage("\r\n", 2);

        xSemaphoreGive(g_print_mutex);
    }
}


void AppDebug_Print_C1(const char *format, ...)
{
    char str[MAXCHARS + 1];
    va_list args;
    int len;
    uint16 cnt;

    if (format == NULL) return;

    va_start(args, format);
    len = vsnprintf(str, sizeof(str), format, args);
    va_end(args);

    if (len < 0) return;
    if (len > MAXCHARS) cnt = MAXCHARS;
    else cnt = (uint16)len;

    if (g_print_mutex == NULL) return;

    if (xSemaphoreTake(g_print_mutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        sendUARTMessage(str, cnt);
        sendUARTMessage("\r\n", 2);

        xSemaphoreGive(g_print_mutex);
    }
}