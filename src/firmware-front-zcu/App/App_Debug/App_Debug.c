#include "App_Debug.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <UART_Logging.h>
#include "IfxCpu.h"
#include "FreeRTOS.h"
#include "semphr.h"

static SemaphoreHandle_t g_print_mutex = NULL;

void AppDebug_Init(void)
{
    initUART();
    g_print_mutex = xSemaphoreCreateMutex();

    SOTA_IsGroupBActive() ? AppDebug_Print("FreeRTOS Bank B Start!\r\n") : AppDebug_Print("FreeRTOS Bank A Start!\r\n");
    AppDebug_Print("Slow Mode!");
    // AppDebug_Print("Fast Mode!");
}

void AppDebug_Print(const char *format, ...)
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