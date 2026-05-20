#include "App_CanTest.h"

#include "App_Can/App_Can.h"
#include "task.h"

#define APP_CANTEST_TASK_STACK_SIZE       (configMINIMAL_STACK_SIZE)
#define APP_CANTEST_TASK_PRIORITY         (tskIDLE_PRIORITY + 1u)
#define APP_CANTEST_POLL_PERIOD_MS        (10u)

#define APP_CANTEST_PERIODIC_TX_ID        (0x100u)
#define APP_CANTEST_TRIGGER_RX_ID         (0x200u)
#define APP_CANTEST_RESPONSE_TX_ID        (0x101u)
#define APP_CANTEST_PERIODIC_TX_MS        (1000u)

static void AppCanTest_Task(void *arg);
static void AppCanTest_SendPeriodicMessage(uint8_t counter);
static void AppCanTest_SendResponseMessage(const AppCanFrame *request, uint8_t counter);

BaseType_t AppCanTest_Start(void)
{
    return xTaskCreate(AppCanTest_Task,
                       "APP CAN TEST",
                       APP_CANTEST_TASK_STACK_SIZE,
                       NULL,
                       APP_CANTEST_TASK_PRIORITY,
                       NULL);
}

static void AppCanTest_Task(void *arg)
{
    AppCanFrame rx_frame;
    TickType_t last_periodic_tick;
    TickType_t now;
    uint8_t periodic_counter = 0u;
    uint8_t response_counter = 0u;

    (void)arg;

    last_periodic_tick = xTaskGetTickCount();

    for (;;)
    {
        now = xTaskGetTickCount();

        if ((TickType_t)(now - last_periodic_tick) >= pdMS_TO_TICKS(APP_CANTEST_PERIODIC_TX_MS))
        {
            last_periodic_tick = now;
            AppCanTest_SendPeriodicMessage(periodic_counter);
            periodic_counter++;
        }

        while (AppCan_RecvById(APP_CANTEST_TRIGGER_RX_ID, &rx_frame, 0u) == pdPASS)
        {
            AppCanTest_SendResponseMessage(&rx_frame, response_counter);
            response_counter++;
        }

        vTaskDelay(pdMS_TO_TICKS(APP_CANTEST_POLL_PERIOD_MS));
    }
}

static void AppCanTest_SendPeriodicMessage(uint8_t counter)
{
    uint8_t payload[8] = {
        0xCAu, 0x4Eu, 0x54u, 0x58u,
        counter, 0x00u, 0x00u, 0x00u
    };

    (void)AppCan_SendClassic(APP_CANTEST_PERIODIC_TX_ID, payload, sizeof(payload));
}

static void AppCanTest_SendResponseMessage(const AppCanFrame *request, uint8_t counter)
{
    uint8_t payload[8] = {
        0xACu, 0x4Bu, counter, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u
    };

    if (request != NULL)
    {
        payload[3] = request->length;
        payload[4] = (request->length > 0u) ? request->data[0] : 0u;
        payload[5] = (request->length > 1u) ? request->data[1] : 0u;
    }

    (void)AppCan_SendClassic(APP_CANTEST_RESPONSE_TX_ID, payload, sizeof(payload));
}
