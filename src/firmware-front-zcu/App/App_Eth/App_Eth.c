#include "App_Eth.h"

#include "Ifx_Lwip.h"

#define APP_ETH_TASK_STACK_SIZE (configMINIMAL_STACK_SIZE)
#define APP_ETH_TASK_PRIORITY   (2u)
#define APP_ETH_TASK_PERIOD_MS (1u)

static BaseType_t g_app_eth_ready = pdFALSE;

static BaseType_t AppEth_Init(void);
static void AppEth_Task(void *arg);

BaseType_t AppEth_Start(void)
{
    if (AppEth_Init() != pdPASS)
    {
        return pdFAIL;
    }

    return xTaskCreate(AppEth_Task, "APP ETH", APP_ETH_TASK_STACK_SIZE, NULL, APP_ETH_TASK_PRIORITY, NULL);
}

static BaseType_t AppEth_Init(void)
{
    eth_addr_t mac_addr;

    MAC_ADDR(&mac_addr, 0x02u, 0x00u, 0x00u, 0x00u, 0x00u, 0x01u);
    Ifx_Lwip_init(mac_addr);

    g_app_eth_ready = pdTRUE;

    return pdPASS;
}

static void AppEth_Task(void *arg)
{
    (void)arg;

    for (;;)
    {
        g_TickCount_1ms++;
        Ifx_Lwip_onTimerTick();
        Ifx_Lwip_pollTimerFlags();
        Ifx_Lwip_pollReceiveFlags();

        vTaskDelay(pdMS_TO_TICKS(APP_ETH_TASK_PERIOD_MS));
    }
}

BaseType_t AppEth_IsReady(void)
{
    return g_app_eth_ready;
}
