#include "App_Eth.h"
#include "App_DoIP.h"
#include "Ifx_Lwip.h"

#define APP_ETH_TASK_STACK_SIZE (4096u)
#define APP_ETH_TASK_PRIORITY   (2u)
#define APP_ETH_TASK_PERIOD_MS (1u)
#define APP_ETH_POWER_ON_DELAY_MS (1000u)
#define APP_ETH_INIT_RETRY_MS (100u)
#define APP_ETH_LINK_STABLE_MS (500u)

static BaseType_t g_app_eth_ready = pdFALSE;
static BaseType_t g_app_eth_initialized = pdFALSE;

static BaseType_t AppEth_Init(void);
static void AppEth_Task(void *arg);
static void AppEth_UpdateReadyState(void);

BaseType_t AppEth_Start(void)
{
    return xTaskCreate(AppEth_Task, "APP ETH", APP_ETH_TASK_STACK_SIZE, NULL, APP_ETH_TASK_PRIORITY, NULL);
}

static BaseType_t AppEth_Init(void)
{
    eth_addr_t mac_addr;

    MAC_ADDR(&mac_addr, 0x02u, 0x00u, 0x00u, 0x00u, 0x00u, 0x01u);
    Ifx_Lwip_init(mac_addr);
    DoIP_Init();

    g_app_eth_ready = pdFALSE;
    g_app_eth_initialized = pdTRUE;

    return pdPASS;
}

static void AppEth_Task(void *arg)
{
    (void)arg;

    vTaskDelay(pdMS_TO_TICKS(APP_ETH_POWER_ON_DELAY_MS));

    for (;;)
    {
        if(g_app_eth_initialized != pdTRUE)
        {
            if(AppEth_Init() != pdPASS)
            {
                vTaskDelay(pdMS_TO_TICKS(APP_ETH_INIT_RETRY_MS));
                continue;
            }
        }

        g_TickCount_1ms++;
        Ifx_Lwip_onTimerTick();
        Ifx_Lwip_pollTimerFlags();
        Ifx_Lwip_pollReceiveFlags();
        AppEth_UpdateReadyState();

        vTaskDelay(pdMS_TO_TICKS(APP_ETH_TASK_PERIOD_MS));
    }
}

BaseType_t AppEth_IsReady(void)
{
    return g_app_eth_ready;
}

static void AppEth_UpdateReadyState(void)
{
    static BaseType_t link_timer_started = pdFALSE;
    static TickType_t link_up_start_tick = 0u;
    TickType_t now;

    if((g_Lwip.netif.flags & NETIF_FLAG_LINK_UP) != 0u)
    {
        now = xTaskGetTickCount();

        if(link_timer_started != pdTRUE)
        {
            link_timer_started = pdTRUE;
            link_up_start_tick = now;
            g_app_eth_ready = pdFALSE;
        }

        if((TickType_t)(now - link_up_start_tick) >= pdMS_TO_TICKS(APP_ETH_LINK_STABLE_MS))
        {
            g_app_eth_ready = pdTRUE;
        }
    }
    else
    {
        link_timer_started = pdFALSE;
        link_up_start_tick = 0u;
        g_app_eth_ready = pdFALSE;
    }
}
