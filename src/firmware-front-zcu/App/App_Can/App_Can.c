/**
 * @file App_Can.c
 * @brief FreeRTOS App-level CAN interface for TC375 MCMCAN.
 *
 * 책임:
 *  - TC375 MCMCAN 하드웨어 초기화
 *  - raw Classical CAN / CAN FD 송신
 *  - RX ISR에서 CAN frame을 내부 Queue로 복사
 *  - CAN ID별 raw mailbox 저장
 *  - Classical CAN ISO-TP normal addressing 송수신
 *
 * 비책임:
 *  - CAN ID 의미 해석
 *  - UDS SID 해석
 *  - 차량 제어 정책
 *  - App별 callback 라우팅
 */

#include "App_Can.h"

#include "Ifx_Types.h"
#include "IfxCan.h"
#include "IfxCan_Can.h"
#include "IfxCpu.h"
#include "IfxCpu_Irq.h"
#include "IfxPort.h"

#include "queue.h"
#include "task.h"

#include <stddef.h>
#include <string.h>

#define APP_CAN_TASK_STACK_SIZE              (768u)
#define APP_CAN_TASK_PRIORITY                (tskIDLE_PRIORITY + 3u)
#define APP_CAN_TASK_PERIOD_MS               (1u)
#define APP_CAN_RX_DRAIN_LIMIT               (8u)

#define APP_CAN_ISR_PRIORITY_TX              (10u)
#define APP_CAN_ISR_PRIORITY_RX              (11u)

#define APP_CAN_USED_NODE_ID                 IfxCan_NodeId_0
#define APP_CAN_CLASSIC_DATA_WORDS           (2u)
#define APP_CAN_FD_DATA_WORDS                (16u)

#define APP_CAN_DEFAULT_N_BS_MS              (1000u)
#define APP_CAN_DEFAULT_N_CR_MS              (1000u)
#define APP_CAN_TP_DEFAULT_RX_QUEUE_LENGTH   (2u)

/* ============================================================
 * Project static CAN receive objects
 * ============================================================
 *
 * CAN에서는 ID가 프레임 안에 들어 있고, TC375 MCMCAN 하드웨어 필터가
 * 이 ECU가 받을 ID를 먼저 걸러준다. 따라서 AppCan은 App별 callback route를
 * 만들지 않고, CAN ID별 mailbox만 만든다.
 *
 * queue_length > 0 : AppCan_RecvById(can_id, ...)로 순서대로 꺼낼 수 있음.
 * keep_latest = 1  : AppCan_ReadLatestById(can_id, ...)로 최신값을 읽을 수 있음.
 *
 * 예시:
 *   #define APP_CAN_RAW_RX_OBJECT_COUNT (2u)
 *   static const AppCanRawRxObjectConfig g_appCanRawRxObjectConfigs[APP_CAN_MAX_RAW_RX_OBJECTS] = {
 *       { 0x201u, 4u, 1u },  // queue 4개 + 최신값 보관
 *       { 0x202u, 4u, 1u }
 *   };
 */

typedef struct AppCanRawRxObjectConfig {
    uint32_t can_id;
    uint8_t  queue_length;
    uint8_t  keep_latest;
} AppCanRawRxObjectConfig;

typedef struct AppCanTpChannelConfig {
    uint8_t  channel_id;
    uint32_t tx_can_id;                       /* Local ECU -> Remote ECU */
    uint32_t rx_can_id;                       /* Remote ECU -> Local ECU */
    uint8_t  rx_queue_length;                 /* 0이면 APP_CAN_TP_DEFAULT_RX_QUEUE_LENGTH */

    uint8_t  padding_enabled;                 /* UDS에서는 보통 1 권장 */
    uint8_t  padding_byte;
    uint8_t  local_block_size;                /* 0 = block size 제한 없음 */
    uint8_t  local_st_min_ms;                 /* 0..127 ms */

    TickType_t n_bs_timeout_ticks;             /* 0이면 기본값 사용 */
    TickType_t n_cr_timeout_ticks;             /* 0이면 기본값 사용 */
} AppCanTpChannelConfig;

#define APP_CAN_RAW_RX_OBJECT_COUNT           (5u)
static const AppCanRawRxObjectConfig g_appCanRawRxObjectConfigs[APP_CAN_MAX_RAW_RX_OBJECTS] = {
    { 0x201u, 4u, 1u },
    { 0x202u, 4u, 1u },
    { 0x702u, 2u, 1u },
    { 0x703u, 2u, 1u },
    { 0x601u, 8u, 0u }
};
static const uint8_t g_appCanRawRxObjectConfigCount = APP_CAN_RAW_RX_OBJECT_COUNT;

#define APP_CAN_TP_CHANNEL_COUNT              (0u)
static const AppCanTpChannelConfig g_appCanTpChannels[APP_CAN_MAX_TP_CHANNELS] = {
    { 0 }
};
static const uint8_t g_appCanTpChannelCount = APP_CAN_TP_CHANNEL_COUNT;

/* ============================================================
 * ISO-TP private constants
 * ============================================================ */

#define APP_CAN_TP_PCI_TYPE_MASK             (0xF0u)
#define APP_CAN_TP_PCI_LENGTH_MASK           (0x0Fu)
#define APP_CAN_TP_SN_MASK                   (0x0Fu)

#define APP_CAN_TP_PCI_SF                    (0x00u)
#define APP_CAN_TP_PCI_FF                    (0x10u)
#define APP_CAN_TP_PCI_CF                    (0x20u)
#define APP_CAN_TP_PCI_FC                    (0x30u)

#define APP_CAN_TP_FS_CTS                    (0x00u)
#define APP_CAN_TP_FS_WAIT                   (0x01u)
#define APP_CAN_TP_FS_OVERFLOW               (0x02u)

/* ============================================================
 * Private types
 * ============================================================ */

typedef struct AppCanHwContext {
    IfxCan_Can_Config     can_config;
    IfxCan_Can            can_module;

    IfxCan_Can_Node       can_node;
    IfxCan_Can_NodeConfig can_node_config;

    IfxCan_Filter         can_filter;

    IfxCan_Message        tx_msg;
    IfxCan_Message        rx_msg;

    uint32                tx_data[APP_CAN_FD_DATA_WORDS];
    uint32                rx_data[APP_CAN_FD_DATA_WORDS];
} AppCanHwContext;

typedef struct AppCanTxQueueItem {
    uint32_t id;
    uint8_t  length;
    uint8_t  is_fd;
    uint8_t  data[APP_CAN_FD_MAX_DLC];
} AppCanTxQueueItem;

typedef struct AppCanRawRxObject {
    AppCanRawRxObjectConfig config;
    QueueHandle_t queue;
    AppCanFrame latest_frame;
    uint8_t has_latest_frame;
} AppCanRawRxObject;

typedef enum AppCanTpTxState {
    APP_CAN_TP_TX_IDLE = 0,
    APP_CAN_TP_TX_WAIT_FC,
    APP_CAN_TP_TX_SEND_CF
} AppCanTpTxState;

typedef enum AppCanTpRxState {
    APP_CAN_TP_RX_IDLE = 0,
    APP_CAN_TP_RX_IN_PROGRESS
} AppCanTpRxState;

typedef struct AppCanTpChannel {
    AppCanTpChannelConfig route;
    QueueHandle_t rx_queue;

    AppCanTpTxState tx_state;
    uint8_t  tx_buffer[APP_CAN_TP_MAX_PAYLOAD_SIZE];
    uint16_t tx_length;
    uint16_t tx_offset;
    uint8_t  tx_next_sn;
    uint8_t  tx_remote_block_size;
    uint8_t  tx_cf_since_fc;
    TickType_t tx_st_min_ticks;
    TickType_t tx_next_cf_tick;
    TickType_t tx_deadline_tick;

    AppCanTpRxState rx_state;
    uint8_t  rx_buffer[APP_CAN_TP_MAX_PAYLOAD_SIZE];
    uint16_t rx_expected_length;
    uint16_t rx_offset;
    uint8_t  rx_expected_sn;
    uint8_t  rx_cf_since_fc;
    TickType_t rx_deadline_tick;
} AppCanTpChannel;

/* ============================================================
 * Private variables
 * ============================================================ */

static AppCanHwContext g_can_hw;
static QueueHandle_t g_internal_rx_queue = NULL;
#define APP_CAN_TX_STUCK_LIMIT_MS   20u

static volatile uint32_t g_tx_in_progress_ticks = 0u;
volatile uint32_t g_tx_stuck_recover_count = 0u;

static AppCanTxQueueItem g_tx_queue[APP_CAN_TX_QUEUE_SIZE];
static volatile uint8_t g_tx_queue_head = 0u;
static volatile uint8_t g_tx_queue_tail = 0u;
static volatile uint8_t g_tx_queue_count = 0u;
static volatile boolean g_tx_in_progress = FALSE;
volatile uint32_t g_tx_isr_count = 0u;
volatile uint32_t g_tx_last_start_id = 0u;
volatile uint8_t  g_tx_last_start_is_fd = 0u;
volatile uint8_t  g_tx_last_start_len = 0u;
volatile uint8_t  g_tx_last_start_data0 = 0u;
volatile uint8_t  g_tx_last_start_data1 = 0u;

volatile sint32   g_tx_last_status = 0;

volatile uint32_t g_tx_send_ok_count = 0u;
volatile uint32_t g_tx_send_busy_count = 0u;
volatile uint32_t g_tx_send_error_count = 0u;

static uint32_t g_rx_filter_ids[APP_CAN_MAX_STD_RX_FILTERS];
static uint8_t g_rx_filter_count = 0u;

static AppCanRawRxObject g_raw_rx_objects[APP_CAN_MAX_RAW_RX_OBJECTS];
static uint8_t g_raw_rx_object_count = 0u;

static AppCanTpChannel g_tp_channels[APP_CAN_MAX_TP_CHANNELS];
static uint8_t g_tp_channel_count = 0u;

static volatile uint32_t g_rx_queued_count = 0u;
static volatile uint32_t g_rx_drop_count = 0u;
static volatile uint32_t g_rx_stored_count = 0u;
static volatile uint32_t g_rx_no_owner_count = 0u;
static volatile uint32_t g_tx_queued_count = 0u;
static volatile uint32_t g_tx_sent_count = 0u;
static volatile uint32_t g_tx_busy_count = 0u;

/* TC375 Lite Kit CAN0 Node0 pin mapping: TX P20.8, RX P20.7, STB P20.6 LOW */
static const IfxCan_Can_Pins g_app_can_pins = {
    &IfxCan_TXD00_P20_8_OUT, IfxPort_OutputMode_pushPull,
    &IfxCan_RXD00B_P20_7_IN, IfxPort_InputMode_noPullDevice,
    IfxPort_PadDriver_cmosAutomotiveSpeed1
};

/* ============================================================
 * Private prototypes
 * ============================================================ */

static BaseType_t AppCan_Init(void);
static void AppCan_Task(void *arg);
static void AppCan_ProcessRx(void);
static void AppCan_DispatchRxFrame(const AppCanFrame *frame);
static BaseType_t AppCan_StoreRawFrame(const AppCanFrame *frame);
static BaseType_t AppCan_RawRxObjectsInit(void);
static AppCanRawRxObject *AppCan_FindRawRxObjectById(uint32_t can_id);

static BaseType_t AppCan_BuildRxFilters(void);
static BaseType_t AppCan_AddRxFilterId(uint32_t can_id);
static BaseType_t AppCan_IsValidStandardCanId(uint32_t can_id);

static void AppCan_InitTransceiver(void);
static void AppCan_InitModule(void);
static void AppCan_InitNode(void);
static void AppCan_InitRxFilters(void);
static void AppCan_SetStandardFilter(uint8_t filter_number, uint32_t can_id, IfxCan_RxBufferId rx_buffer_id);

static void AppCan_ReadUpdatedRxBuffers(void);
static void AppCan_ReadRxBuffer(uint8_t rx_buffer_number);
static BaseType_t AppCan_TryStoreLatestFromIsr(const AppCanFrame *frame);
static void AppCan_QueueRxFrameFromIsr(const AppCanFrame *frame);

static BaseType_t AppCan_EnqueueTxMessage(uint32_t id, const uint8_t *data, uint8_t length, uint8_t is_fd);
static void AppCan_TryStartNextTx(void);
static void AppCan_PrepareTxMessage(const AppCanTxQueueItem *item);

static IfxCan_DataLengthCode AppCan_GetClassicDlc(uint8_t dlc);
static IfxCan_DataLengthCode AppCan_GetFdDlc(uint8_t length);
static uint8_t AppCan_GetLengthFromDlc(IfxCan_DataLengthCode dlc);
static void AppCan_CopyBytesToWords(uint32 *word_buffer, const uint8_t *byte_buffer, uint16_t length);
static void AppCan_CopyWordsToBytes(uint8_t *byte_buffer, const uint32 *word_buffer, uint16_t length);

static BaseType_t AppCan_TpInit(void);
static void AppCan_TpMainFunction(void);
static AppCanTpChannel *AppCan_TpFindByChannelId(uint8_t channel_id);
static AppCanTpChannel *AppCan_TpFindByRxCanId(uint32_t rx_can_id);
static BaseType_t AppCan_TpOnCanRx(const AppCanFrame *frame);
static void AppCan_TpHandleSingleFrame(AppCanTpChannel *ch, const AppCanFrame *frame);
static void AppCan_TpHandleFirstFrame(AppCanTpChannel *ch, const AppCanFrame *frame);
static void AppCan_TpHandleConsecutiveFrame(AppCanTpChannel *ch, const AppCanFrame *frame);
static void AppCan_TpHandleFlowControl(AppCanTpChannel *ch, const AppCanFrame *frame);
static BaseType_t AppCan_TpSendFlowControl(AppCanTpChannel *ch, uint8_t flow_status);
static BaseType_t AppCan_TpSendNextConsecutiveFrame(AppCanTpChannel *ch);
static void AppCan_TpResetTx(AppCanTpChannel *ch);
static void AppCan_TpResetRx(AppCanTpChannel *ch);
static void AppCan_TpDeliverMessage(AppCanTpChannel *ch);
static void AppCan_TpReportError(AppCanTpChannel *ch, AppCanTpError error);
static TickType_t AppCan_TpGetNBsTimeout(const AppCanTpChannel *ch);
static TickType_t AppCan_TpGetNCrTimeout(const AppCanTpChannel *ch);
static TickType_t AppCan_TpStMinToTicks(uint8_t st_min);
static BaseType_t AppCan_TickExpired(TickType_t now, TickType_t deadline);
static uint8_t AppCan_MinPayloadCopyLength(uint16_t remain, uint8_t max_copy);

/* ============================================================
 * Interrupt handlers
 * ============================================================ */

IFX_INTERRUPT(AppCan_TxIsr, 0, APP_CAN_ISR_PRIORITY_TX);
IFX_INTERRUPT(AppCan_RxIsr, 0, APP_CAN_ISR_PRIORITY_RX);

void AppCan_TxIsr(void)
{
    g_tx_isr_count++;
    IfxCan_Node_clearInterruptFlag(g_can_hw.can_node.node,
                                   IfxCan_Interrupt_transmissionCompleted);

    g_tx_in_progress = FALSE;
    AppCan_TryStartNextTx();
}

void AppCan_RxIsr(void)
{
    IfxCan_Node_clearInterruptFlag(g_can_hw.can_node.node,
                                   IfxCan_Interrupt_messageStoredToDedicatedRxBuffer);

    AppCan_ReadUpdatedRxBuffers();
}

/* ============================================================
 * Public API
 * ============================================================ */

BaseType_t AppCan_Start(void)
{
    if(AppCan_Init() != pdPASS)
    {
        return pdFAIL;
    }

    return xTaskCreate(AppCan_Task,
                       "APP CAN",
                       APP_CAN_TASK_STACK_SIZE,
                       NULL,
                       APP_CAN_TASK_PRIORITY,
                       NULL);
}

BaseType_t AppCan_RecvById(uint32_t can_id, AppCanFrame *frame, TickType_t wait_ticks)
{
    AppCanRawRxObject *object;

    if((frame == NULL) || (AppCan_IsValidStandardCanId(can_id) != pdPASS))
    {
        return pdFAIL;
    }

    object = AppCan_FindRawRxObjectById(can_id);
    if((object == NULL) || (object->queue == NULL))
    {
        return pdFAIL;
    }

    return xQueueReceive(object->queue, frame, wait_ticks);
}

BaseType_t AppCan_ReadLatestById(uint32_t can_id, AppCanFrame *frame)
{
    AppCanRawRxObject *object;

    if((frame == NULL) || (AppCan_IsValidStandardCanId(can_id) != pdPASS))
    {
        return pdFAIL;
    }

    object = AppCan_FindRawRxObjectById(can_id);
    if((object == NULL) || (object->has_latest_frame == 0u))
    {
        return pdFAIL;
    }

    taskENTER_CRITICAL();
    *frame = object->latest_frame;
    taskEXIT_CRITICAL();

    return pdPASS;
}

UBaseType_t AppCan_GetPendingCountById(uint32_t can_id)
{
    AppCanRawRxObject *object = AppCan_FindRawRxObjectById(can_id);

    if((object == NULL) || (object->queue == NULL))
    {
        return 0u;
    }

    return uxQueueMessagesWaiting(object->queue);
}

BaseType_t AppCan_SendClassic(uint32_t id, const uint8_t *data, uint8_t length)
{
    if((AppCan_IsValidStandardCanId(id) != pdPASS) ||
       (length > APP_CAN_CLASSIC_MAX_DLC) ||
       ((length > 0u) && (data == NULL)))
    {
        return pdFAIL;
    }

    return AppCan_EnqueueTxMessage(id, data, length, 0u);
}

BaseType_t AppCan_SendFd(uint32_t id, const uint8_t *data, uint8_t length)
{
    if((AppCan_IsValidStandardCanId(id) != pdPASS) ||
       (length > APP_CAN_FD_MAX_DLC) ||
       ((length > 0u) && (data == NULL)))
    {
        return pdFAIL;
    }

    return AppCan_EnqueueTxMessage(id, data, length, 1u);
}

BaseType_t AppCan_TpSend(uint8_t channel_id, const uint8_t *payload, uint16_t length)
{
    AppCanTpChannel *ch;
    uint8_t frame[APP_CAN_TP_CLASSIC_DLC];
    uint8_t payload_index;
    BaseType_t send_result;

    ch = AppCan_TpFindByChannelId(channel_id);

    if((ch == NULL) ||
       ((length > 0u) && (payload == NULL)) ||
       (length > APP_CAN_TP_MAX_PAYLOAD_SIZE) ||
       (length > APP_CAN_TP_ISO_MAX_PAYLOAD_SIZE))
    {
        return pdFAIL;
    }

    taskENTER_CRITICAL();
    if(ch->tx_state != APP_CAN_TP_TX_IDLE)
    {
        taskEXIT_CRITICAL();
        AppCan_TpReportError(ch, APP_CAN_TP_ERROR_BUSY);
        return pdFAIL;
    }

    if(length > 0u)
    {
        memcpy(ch->tx_buffer, payload, length);
    }
    ch->tx_length = length;
    ch->tx_offset = 0u;
    ch->tx_next_sn = 1u;
    ch->tx_remote_block_size = 0u;
    ch->tx_cf_since_fc = 0u;
    ch->tx_st_min_ticks = 0u;
    taskEXIT_CRITICAL();

    memset(frame, ch->route.padding_byte, sizeof(frame));

    if(length <= APP_CAN_TP_SINGLE_FRAME_MAX_PAYLOAD)
    {
        frame[0] = (uint8_t)(APP_CAN_TP_PCI_SF | (uint8_t)length);
        for(payload_index = 0u; payload_index < length; payload_index++)
        {
            frame[1u + payload_index] = payload[payload_index];
        }

        if(ch->route.padding_enabled != 0u)
        {
            send_result = AppCan_SendClassic(ch->route.tx_can_id, frame, APP_CAN_TP_CLASSIC_DLC);
        }
        else
        {
            send_result = AppCan_SendClassic(ch->route.tx_can_id, frame, (uint8_t)(length + 1u));
        }

        if(send_result != pdPASS)
        {
            AppCan_TpReportError(ch, APP_CAN_TP_ERROR_TX_FAILED);
        }

        return send_result;
    }

    frame[0] = (uint8_t)(APP_CAN_TP_PCI_FF | ((length >> 8u) & APP_CAN_TP_PCI_LENGTH_MASK));
    frame[1] = (uint8_t)(length & 0xFFu);

    for(payload_index = 0u; payload_index < APP_CAN_TP_FIRST_FRAME_DATA_SIZE; payload_index++)
    {
        frame[2u + payload_index] = ch->tx_buffer[payload_index];
    }

    taskENTER_CRITICAL();
    ch->tx_state = APP_CAN_TP_TX_WAIT_FC;
    ch->tx_offset = APP_CAN_TP_FIRST_FRAME_DATA_SIZE;
    ch->tx_deadline_tick = xTaskGetTickCount() + AppCan_TpGetNBsTimeout(ch);
    taskEXIT_CRITICAL();

    send_result = AppCan_SendClassic(ch->route.tx_can_id, frame, APP_CAN_TP_CLASSIC_DLC);
    if(send_result != pdPASS)
    {
        taskENTER_CRITICAL();
        AppCan_TpResetTx(ch);
        taskEXIT_CRITICAL();
        AppCan_TpReportError(ch, APP_CAN_TP_ERROR_TX_FAILED);
    }

    return send_result;
}

BaseType_t AppCan_TpRecv(uint8_t channel_id, AppCanTpRxMsg *msg, TickType_t wait_ticks)
{
    AppCanTpChannel *ch;

    if(msg == NULL)
    {
        return pdFAIL;
    }

    ch = AppCan_TpFindByChannelId(channel_id);
    if((ch == NULL) || (ch->rx_queue == NULL))
    {
        return pdFAIL;
    }

    return xQueueReceive(ch->rx_queue, msg, wait_ticks);
}

BaseType_t AppCan_TpIsBusy(uint8_t channel_id)
{
    AppCanTpChannel *ch = AppCan_TpFindByChannelId(channel_id);

    if(ch == NULL)
    {
        return pdFALSE;
    }

    return (ch->tx_state == APP_CAN_TP_TX_IDLE) ? pdFALSE : pdTRUE;
}

uint32_t AppCan_GetRxQueuedCount(void) { return g_rx_queued_count; }
uint32_t AppCan_GetRxDropCount(void) { return g_rx_drop_count; }
uint32_t AppCan_GetRxStoredCount(void) { return g_rx_stored_count; }
uint32_t AppCan_GetRxNoOwnerCount(void) { return g_rx_no_owner_count; }
uint32_t AppCan_GetTxQueuedCount(void) { return g_tx_queued_count; }
uint32_t AppCan_GetTxSentCount(void) { return g_tx_sent_count; }
uint32_t AppCan_GetTxBusyCount(void) { return g_tx_busy_count; }

/* ============================================================
 * Private functions: app lifecycle and RX mailbox
 * ============================================================ */

static BaseType_t AppCan_Init(void)
{
    memset(&g_can_hw, 0, sizeof(g_can_hw));
    memset(g_tx_queue, 0, sizeof(g_tx_queue));
    memset(g_rx_filter_ids, 0, sizeof(g_rx_filter_ids));
    memset(g_raw_rx_objects, 0, sizeof(g_raw_rx_objects));

    g_tx_queue_head = 0u;
    g_tx_queue_tail = 0u;
    g_tx_queue_count = 0u;
    g_tx_in_progress = FALSE;

    g_rx_queued_count = 0u;
    g_rx_drop_count = 0u;
    g_rx_stored_count = 0u;
    g_rx_no_owner_count = 0u;
    g_tx_queued_count = 0u;
    g_tx_sent_count = 0u;
    g_tx_busy_count = 0u;

    if((APP_CAN_TP_MAX_PAYLOAD_SIZE > APP_CAN_TP_ISO_MAX_PAYLOAD_SIZE) ||
       (APP_CAN_RAW_RX_OBJECT_COUNT > APP_CAN_MAX_RAW_RX_OBJECTS) ||
       (APP_CAN_TP_CHANNEL_COUNT > APP_CAN_MAX_TP_CHANNELS))
    {
        return pdFAIL;
    }

    if(AppCan_RawRxObjectsInit() != pdPASS)
    {
        return pdFAIL;
    }

    if(AppCan_TpInit() != pdPASS)
    {
        return pdFAIL;
    }

    if(AppCan_BuildRxFilters() != pdPASS)
    {
        return pdFAIL;
    }

    g_internal_rx_queue = xQueueCreate(APP_CAN_INTERNAL_RX_QUEUE_SIZE, sizeof(AppCanFrame));
    if(g_internal_rx_queue == NULL)
    {
        return pdFAIL;
    }

    AppCan_InitTransceiver();
    AppCan_InitModule();
    AppCan_InitNode();
    AppCan_InitRxFilters();

    return pdPASS;
}

static void AppCan_Task(void *arg)
{
    (void)arg;

    for(;;)
    {
        AppCan_ProcessRx();
        AppCan_TpMainFunction();

        taskENTER_CRITICAL();

        if(g_tx_in_progress == TRUE)
        {
            g_tx_in_progress_ticks++;

            if(g_tx_in_progress_ticks >= APP_CAN_TX_STUCK_LIMIT_MS)
            {
                g_tx_in_progress = FALSE;
                g_tx_in_progress_ticks = 0u;
                g_tx_stuck_recover_count++;
            }
        }
        else
        {
            g_tx_in_progress_ticks = 0u;
        }

        AppCan_TryStartNextTx();

        taskEXIT_CRITICAL();

        vTaskDelay(pdMS_TO_TICKS(APP_CAN_TASK_PERIOD_MS));
    }
}

static void AppCan_ProcessRx(void)
{
    AppCanFrame frame;
    uint32_t i;

    if(g_internal_rx_queue == NULL)
    {
        return;
    }

    for(i = 0u; i < APP_CAN_RX_DRAIN_LIMIT; i++)
    {
        if(xQueueReceive(g_internal_rx_queue, &frame, 0u) != pdPASS)
        {
            break;
        }

        AppCan_DispatchRxFrame(&frame);
    }
}

static void AppCan_DispatchRxFrame(const AppCanFrame *frame)
{
    BaseType_t matched = pdFALSE;

    if(frame == NULL)
    {
        return;
    }

    if(AppCan_TpOnCanRx(frame) == pdPASS)
    {
        matched = pdTRUE;
    }

    if(AppCan_StoreRawFrame(frame) == pdPASS)
    {
        matched = pdTRUE;
    }

    if(matched != pdPASS)
    {
        g_rx_no_owner_count++;
    }
}

static BaseType_t AppCan_StoreRawFrame(const AppCanFrame *frame)
{
    AppCanRawRxObject *object;
    BaseType_t stored = pdFALSE;

    if(frame == NULL)
    {
        return pdFAIL;
    }

    object = AppCan_FindRawRxObjectById(frame->id);
    if(object == NULL)
    {
        return pdFAIL;
    }

    if(object->config.keep_latest != 0u)
    {
        taskENTER_CRITICAL();
        object->latest_frame = *frame;
        object->has_latest_frame = 1u;
        taskEXIT_CRITICAL();
        stored = pdTRUE;
    }

    if(object->queue != NULL)
    {
        if(xQueueSend(object->queue, frame, 0u) == pdPASS)
        {
            stored = pdTRUE;
        }
        else
        {
            g_rx_drop_count++;
        }
    }

    if(stored == pdPASS)
    {
        g_rx_stored_count++;
    }

    return stored;
}

static BaseType_t AppCan_RawRxObjectsInit(void)
{
    uint8_t i;
    uint8_t j;

    g_raw_rx_object_count = 0u;

    for(i = 0u; i < g_appCanRawRxObjectConfigCount; i++)
    {
        if(AppCan_IsValidStandardCanId(g_appCanRawRxObjectConfigs[i].can_id) != pdPASS)
        {
            return pdFAIL;
        }

        if((g_appCanRawRxObjectConfigs[i].queue_length == 0u) &&
           (g_appCanRawRxObjectConfigs[i].keep_latest == 0u))
        {
            return pdFAIL;
        }

        for(j = 0u; j < i; j++)
        {
            if(g_appCanRawRxObjectConfigs[j].can_id == g_appCanRawRxObjectConfigs[i].can_id)
            {
                return pdFAIL;
            }
        }

        g_raw_rx_objects[i].config = g_appCanRawRxObjectConfigs[i];
        g_raw_rx_objects[i].queue = NULL;
        g_raw_rx_objects[i].has_latest_frame = 0u;

        if(g_appCanRawRxObjectConfigs[i].queue_length > 0u)
        {
            g_raw_rx_objects[i].queue = xQueueCreate(g_appCanRawRxObjectConfigs[i].queue_length,
                                                     sizeof(AppCanFrame));
            if(g_raw_rx_objects[i].queue == NULL)
            {
                return pdFAIL;
            }
        }
    }

    g_raw_rx_object_count = g_appCanRawRxObjectConfigCount;
    return pdPASS;
}

static AppCanRawRxObject *AppCan_FindRawRxObjectById(uint32_t can_id)
{
    uint8_t i;

    for(i = 0u; i < g_raw_rx_object_count; i++)
    {
        if(g_raw_rx_objects[i].config.can_id == can_id)
        {
            return &g_raw_rx_objects[i];
        }
    }

    return NULL;
}

/* ============================================================
 * Private functions: filter setup
 * ============================================================ */

static BaseType_t AppCan_BuildRxFilters(void)
{
    uint8_t i;

    g_rx_filter_count = 0u;
    memset(g_rx_filter_ids, 0, sizeof(g_rx_filter_ids));

    for(i = 0u; i < g_raw_rx_object_count; i++)
    {
        if(AppCan_AddRxFilterId(g_raw_rx_objects[i].config.can_id) != pdPASS)
        {
            return pdFAIL;
        }
    }

    for(i = 0u; i < g_tp_channel_count; i++)
    {
        if(AppCan_AddRxFilterId(g_tp_channels[i].route.rx_can_id) != pdPASS)
        {
            return pdFAIL;
        }
    }

    return pdPASS;
}

static BaseType_t AppCan_AddRxFilterId(uint32_t can_id)
{
    uint8_t i;

    for(i = 0u; i < g_rx_filter_count; i++)
    {
        if(g_rx_filter_ids[i] == can_id)
        {
            return pdPASS;
        }
    }

    if(g_rx_filter_count >= APP_CAN_MAX_STD_RX_FILTERS)
    {
        return pdFAIL;
    }

    g_rx_filter_ids[g_rx_filter_count] = can_id;
    g_rx_filter_count++;

    return pdPASS;
}

static BaseType_t AppCan_IsValidStandardCanId(uint32_t can_id)
{
    return (can_id <= 0x7FFu) ? pdPASS : pdFAIL;
}

/* ============================================================
 * Private functions: TC375 MCMCAN hardware
 * ============================================================ */

static void AppCan_InitTransceiver(void)
{
    IfxPort_setPinModeOutput(&MODULE_P20,
                             6,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);

    IfxPort_setPinLow(&MODULE_P20, 6);
}

static void AppCan_InitModule(void)
{
    IfxCan_Can_initModuleConfig(&g_can_hw.can_config, &MODULE_CAN0);
    IfxCan_Can_initModule(&g_can_hw.can_module, &g_can_hw.can_config);
}

static void AppCan_InitNode(void)
{
    IfxCan_Can_initNodeConfig(&g_can_hw.can_node_config,
                              &g_can_hw.can_module);

    g_can_hw.can_node_config.nodeId = APP_CAN_USED_NODE_ID;
    g_can_hw.can_node_config.busLoopbackEnabled = FALSE;
    g_can_hw.can_node_config.frame.type = IfxCan_FrameType_transmitAndReceive;
    g_can_hw.can_node_config.frame.mode = IfxCan_FrameMode_fdLong;
    g_can_hw.can_node_config.pins = &g_app_can_pins;

    g_can_hw.can_node_config.txConfig.txMode = IfxCan_TxMode_dedicatedBuffers;
    g_can_hw.can_node_config.txConfig.txBufferDataFieldSize = IfxCan_DataFieldSize_64;

    g_can_hw.can_node_config.rxConfig.rxMode = IfxCan_RxMode_dedicatedBuffers;
    g_can_hw.can_node_config.rxConfig.rxBufferDataFieldSize = IfxCan_DataFieldSize_64;

    g_can_hw.can_node_config.filterConfig.messageIdLength = IfxCan_MessageIdLength_standard;
    g_can_hw.can_node_config.filterConfig.standardListSize = g_rx_filter_count;
    g_can_hw.can_node_config.filterConfig.extendedListSize = 0u;
    g_can_hw.can_node_config.filterConfig.standardFilterForNonMatchingFrames = IfxCan_NonMatchingFrame_reject;
    g_can_hw.can_node_config.filterConfig.extendedFilterForNonMatchingFrames = IfxCan_NonMatchingFrame_reject;
    g_can_hw.can_node_config.filterConfig.rejectRemoteFramesWithStandardId = TRUE;
    g_can_hw.can_node_config.filterConfig.rejectRemoteFramesWithExtendedId = TRUE;

    g_can_hw.can_node_config.interruptConfig.transmissionCompletedEnabled = TRUE;
    g_can_hw.can_node_config.interruptConfig.traco.priority = APP_CAN_ISR_PRIORITY_TX;
    g_can_hw.can_node_config.interruptConfig.traco.interruptLine = IfxCan_InterruptLine_0;
    g_can_hw.can_node_config.interruptConfig.traco.typeOfService = IfxSrc_Tos_cpu0;

    g_can_hw.can_node_config.interruptConfig.messageStoredToDedicatedRxBufferEnabled = TRUE;
    g_can_hw.can_node_config.interruptConfig.reint.priority = APP_CAN_ISR_PRIORITY_RX;
    g_can_hw.can_node_config.interruptConfig.reint.interruptLine = IfxCan_InterruptLine_1;
    g_can_hw.can_node_config.interruptConfig.reint.typeOfService = IfxSrc_Tos_cpu0;

    IfxCan_Can_initNode(&g_can_hw.can_node,
                        &g_can_hw.can_node_config);
}

static void AppCan_InitRxFilters(void)
{
    uint8_t i;

    for(i = 0u; i < g_rx_filter_count; i++)
    {
        AppCan_SetStandardFilter(i,
                                 g_rx_filter_ids[i],
                                 (IfxCan_RxBufferId)i);
    }
}

static void AppCan_SetStandardFilter(uint8_t filter_number, uint32_t can_id, IfxCan_RxBufferId rx_buffer_id)
{
    g_can_hw.can_filter.number = filter_number;
    g_can_hw.can_filter.elementConfiguration = IfxCan_FilterElementConfiguration_storeInRxBuffer;
    g_can_hw.can_filter.type = IfxCan_FilterType_none;
    g_can_hw.can_filter.id1 = can_id;
    g_can_hw.can_filter.id2 = 0u;
    g_can_hw.can_filter.rxBufferOffset = rx_buffer_id;

    IfxCan_Can_setStandardFilter(&g_can_hw.can_node,
                                 &g_can_hw.can_filter);
}

static void AppCan_ReadUpdatedRxBuffers(void)
{
    uint8_t i;

    for(i = 0u; i < g_rx_filter_count; i++)
    {
        if(IfxCan_Node_isRxBufferNewDataUpdated(g_can_hw.can_node.node,
                                                (IfxCan_RxBufferId)i))
        {
            AppCan_ReadRxBuffer(i);
        }
    }
}

static void AppCan_ReadRxBuffer(uint8_t rx_buffer_number)
{
    AppCanFrame frame;

    IfxCan_Can_initMessage(&g_can_hw.rx_msg);
    memset(g_can_hw.rx_data, 0, sizeof(g_can_hw.rx_data));
    memset(&frame, 0, sizeof(frame));

    g_can_hw.rx_msg.readFromRxFifo0 = FALSE;
    g_can_hw.rx_msg.readFromRxFifo1 = FALSE;
    g_can_hw.rx_msg.bufferNumber = rx_buffer_number;

    IfxCan_Can_readMessage(&g_can_hw.can_node,
                           &g_can_hw.rx_msg,
                           g_can_hw.rx_data);

    frame.id = g_can_hw.rx_msg.messageId;
    frame.length = AppCan_GetLengthFromDlc(g_can_hw.rx_msg.dataLengthCode);
    frame.is_fd = (g_can_hw.rx_msg.frameMode == IfxCan_FrameMode_fdLong) ? 1u : 0u;

    if(frame.length > APP_CAN_FD_MAX_DLC)
    {
        frame.length = APP_CAN_FD_MAX_DLC;
    }

    AppCan_CopyWordsToBytes(frame.data, g_can_hw.rx_data, frame.length);

    if(AppCan_TryStoreLatestFromIsr(&frame) == pdPASS)
    {
        return;
    }

    AppCan_QueueRxFrameFromIsr(&frame);
}

static BaseType_t AppCan_TryStoreLatestFromIsr(const AppCanFrame *frame)
{
    AppCanRawRxObject *object;

    if(frame == NULL)
    {
        return pdFAIL;
    }

    if(AppCan_TpFindByRxCanId(frame->id) != NULL)
    {
        return pdFAIL;
    }

    object = AppCan_FindRawRxObjectById(frame->id);
    if((object == NULL) || (object->config.keep_latest == 0u) || (object->queue != NULL))
    {
        return pdFAIL;
    }

    object->latest_frame = *frame;
    object->has_latest_frame = 1u;
    g_rx_stored_count++;

    return pdPASS;
}

static void AppCan_QueueRxFrameFromIsr(const AppCanFrame *frame)
{
    BaseType_t result;
    BaseType_t higher_priority_task_woken = pdFALSE;

    if((g_internal_rx_queue == NULL) || (frame == NULL))
    {
        return;
    }

    result = xQueueSendFromISR(g_internal_rx_queue,
                               frame,
                               &higher_priority_task_woken);

    if(result == pdPASS)
    {
        g_rx_queued_count++;
    }
    else
    {
        g_rx_drop_count++;
    }

#if defined(portYIELD_FROM_ISR)
    portYIELD_FROM_ISR(higher_priority_task_woken);
#elif defined(portEND_SWITCHING_ISR)
    portEND_SWITCHING_ISR(higher_priority_task_woken);
#else
    (void)higher_priority_task_woken;
#endif
}

/* ============================================================
 * Private functions: raw TX queue
 * ============================================================ */

static BaseType_t AppCan_EnqueueTxMessage(uint32_t id, const uint8_t *data, uint8_t length, uint8_t is_fd)
{
    AppCanTxQueueItem *item;
    boolean interrupt_state;

    interrupt_state = IfxCpu_disableInterrupts();

    if(g_tx_queue_count >= APP_CAN_TX_QUEUE_SIZE)
    {
        g_tx_busy_count++;
        IfxCpu_restoreInterrupts(interrupt_state);
        return pdFAIL;
    }

    item = &g_tx_queue[g_tx_queue_tail];
    item->id = id;
    item->length = length;
    item->is_fd = is_fd;
    memset(item->data, 0, sizeof(item->data));

    if((length > 0u) && (data != NULL))
    {
        memcpy(item->data, data, length);
    }

    g_tx_queue_tail++;
    if(g_tx_queue_tail >= APP_CAN_TX_QUEUE_SIZE)
    {
        g_tx_queue_tail = 0u;
    }

    g_tx_queue_count++;
    g_tx_queued_count++;

    AppCan_TryStartNextTx();

    IfxCpu_restoreInterrupts(interrupt_state);
    return pdPASS;
}

static void AppCan_TryStartNextTx(void)
{
    AppCanTxQueueItem *item;
    IfxCan_Status status;
    //debug
    volatile uint32_t g_tx_ota600_start_count = 0u;
    volatile uint8_t  g_tx_ota600_data0 = 0u;
    volatile uint8_t  g_tx_ota600_data1 = 0u;
    volatile uint8_t  g_tx_ota600_len = 0u;
    volatile uint8_t  g_tx_ota600_is_fd = 0u;

    if((g_tx_in_progress == TRUE) || (g_tx_queue_count == 0u))
    {
        return;
    }

    item = &g_tx_queue[g_tx_queue_head];

    /*
     * Debug: 실제 HW TX buffer에 넣으려는 frame 정보
     */
    g_tx_last_start_id = item->id;
    g_tx_last_start_is_fd = item->is_fd;
    g_tx_last_start_len = item->length;

    if(item->length > 0u)
    {
        g_tx_last_start_data0 = item->data[0];
    }
    else
    {
        g_tx_last_start_data0 = 0u;
    }

    if(item->length > 1u)
    {
        g_tx_last_start_data1 = item->data[1];
    }
    else
    {
        g_tx_last_start_data1 = 0u;
    }
    if(item->id == 0x600u)
{
    g_tx_ota600_start_count++;
    g_tx_ota600_len = item->length;
    g_tx_ota600_is_fd = item->is_fd;
    g_tx_ota600_data0 = (item->length > 0u) ? item->data[0] : 0u;
    g_tx_ota600_data1 = (item->length > 1u) ? item->data[1] : 0u;
}
    AppCan_PrepareTxMessage(item);

    status = IfxCan_Can_sendMessage(&g_can_hw.can_node,
                                    &g_can_hw.tx_msg,
                                    g_can_hw.tx_data);

    /*
     * Debug: sendMessage return status 기록
     */
    g_tx_last_status = (sint32)status;

    if(status == IfxCan_Status_notSentBusy)
    {
        g_tx_busy_count++;
        g_tx_send_busy_count++;
        return;
    }

    /*
     * 중요:
     * notSentBusy가 아니라고 무조건 성공 처리하면 안 됨.
     * 반드시 OK일 때만 queue에서 제거해야 함.
     */
    if(status != IfxCan_Status_ok)
    {
        g_tx_send_error_count++;
        return;
    }

    g_tx_send_ok_count++;

    g_tx_queue_head++;
    if(g_tx_queue_head >= APP_CAN_TX_QUEUE_SIZE)
    {
        g_tx_queue_head = 0u;
    }

    g_tx_queue_count--;
    g_tx_in_progress = TRUE;
    g_tx_sent_count++;
}
static void AppCan_PrepareTxMessage(const AppCanTxQueueItem *item)
{
    IfxCan_Can_initMessage(&g_can_hw.tx_msg);
    memset(g_can_hw.tx_data, 0, sizeof(g_can_hw.tx_data));

    AppCan_CopyBytesToWords(g_can_hw.tx_data, item->data, item->length);

    g_can_hw.tx_msg.messageId = item->id;
    g_can_hw.tx_msg.messageIdLength = IfxCan_MessageIdLength_standard;

    if(item->is_fd != 0u)
    {
        g_can_hw.tx_msg.frameMode = IfxCan_FrameMode_fdLong;
        g_can_hw.tx_msg.dataLengthCode = AppCan_GetFdDlc(item->length);
    }
    else
    {
        g_can_hw.tx_msg.frameMode = IfxCan_FrameMode_standard;
        g_can_hw.tx_msg.dataLengthCode = AppCan_GetClassicDlc(item->length);
    }
}

static IfxCan_DataLengthCode AppCan_GetClassicDlc(uint8_t dlc)
{
    switch(dlc)
    {
        case 0u: return IfxCan_DataLengthCode_0;
        case 1u: return IfxCan_DataLengthCode_1;
        case 2u: return IfxCan_DataLengthCode_2;
        case 3u: return IfxCan_DataLengthCode_3;
        case 4u: return IfxCan_DataLengthCode_4;
        case 5u: return IfxCan_DataLengthCode_5;
        case 6u: return IfxCan_DataLengthCode_6;
        case 7u: return IfxCan_DataLengthCode_7;
        case 8u: return IfxCan_DataLengthCode_8;
        default: return IfxCan_DataLengthCode_0;
    }
}

static IfxCan_DataLengthCode AppCan_GetFdDlc(uint8_t length)
{
    if(length <= 8u)
    {
        return AppCan_GetClassicDlc(length);
    }
    else if(length <= 12u)
    {
        return IfxCan_DataLengthCode_12;
    }
    else if(length <= 16u)
    {
        return IfxCan_DataLengthCode_16;
    }
    else if(length <= 20u)
    {
        return IfxCan_DataLengthCode_20;
    }
    else if(length <= 24u)
    {
        return IfxCan_DataLengthCode_24;
    }
    else if(length <= 32u)
    {
        return IfxCan_DataLengthCode_32;
    }
    else if(length <= 48u)
    {
        return IfxCan_DataLengthCode_48;
    }

    return IfxCan_DataLengthCode_64;
}

static uint8_t AppCan_GetLengthFromDlc(IfxCan_DataLengthCode dlc)
{
    switch(dlc)
    {
        case IfxCan_DataLengthCode_0:  return 0u;
        case IfxCan_DataLengthCode_1:  return 1u;
        case IfxCan_DataLengthCode_2:  return 2u;
        case IfxCan_DataLengthCode_3:  return 3u;
        case IfxCan_DataLengthCode_4:  return 4u;
        case IfxCan_DataLengthCode_5:  return 5u;
        case IfxCan_DataLengthCode_6:  return 6u;
        case IfxCan_DataLengthCode_7:  return 7u;
        case IfxCan_DataLengthCode_8:  return 8u;
        case IfxCan_DataLengthCode_12: return 12u;
        case IfxCan_DataLengthCode_16: return 16u;
        case IfxCan_DataLengthCode_20: return 20u;
        case IfxCan_DataLengthCode_24: return 24u;
        case IfxCan_DataLengthCode_32: return 32u;
        case IfxCan_DataLengthCode_48: return 48u;
        case IfxCan_DataLengthCode_64: return 64u;
        default:                       return 0u;
    }
}

static void AppCan_CopyBytesToWords(uint32 *word_buffer, const uint8_t *byte_buffer, uint16_t length)
{
    memset(word_buffer, 0, APP_CAN_FD_DATA_WORDS * sizeof(uint32));

    if((length > 0u) && (byte_buffer != NULL))
    {
        memcpy((uint8_t *)word_buffer, byte_buffer, length);
    }
}

static void AppCan_CopyWordsToBytes(uint8_t *byte_buffer, const uint32 *word_buffer, uint16_t length)
{
    if((length > 0u) && (byte_buffer != NULL) && (word_buffer != NULL))
    {
        memcpy(byte_buffer, (const uint8_t *)word_buffer, length);
    }
}

/* ============================================================
 * Private functions: ISO-TP
 * ============================================================ */

static BaseType_t AppCan_TpInit(void)
{
    uint8_t i;
    uint8_t j;
    uint8_t queue_length;

    memset(g_tp_channels, 0, sizeof(g_tp_channels));
    g_tp_channel_count = 0u;

    for(i = 0u; i < g_appCanTpChannelCount; i++)
    {
        if((AppCan_IsValidStandardCanId(g_appCanTpChannels[i].tx_can_id) != pdPASS) ||
           (AppCan_IsValidStandardCanId(g_appCanTpChannels[i].rx_can_id) != pdPASS))
        {
            return pdFAIL;
        }

        for(j = 0u; j < i; j++)
        {
            if((g_appCanTpChannels[j].channel_id == g_appCanTpChannels[i].channel_id) ||
               (g_appCanTpChannels[j].rx_can_id == g_appCanTpChannels[i].rx_can_id))
            {
                return pdFAIL;
            }
        }

        g_tp_channels[i].route = g_appCanTpChannels[i];
        if(g_tp_channels[i].route.local_st_min_ms > 127u)
        {
            g_tp_channels[i].route.local_st_min_ms = 127u;
        }

        queue_length = g_tp_channels[i].route.rx_queue_length;
        if(queue_length == 0u)
        {
            queue_length = APP_CAN_TP_DEFAULT_RX_QUEUE_LENGTH;
        }

        g_tp_channels[i].rx_queue = xQueueCreate(queue_length, sizeof(AppCanTpRxMsg));
        if(g_tp_channels[i].rx_queue == NULL)
        {
            return pdFAIL;
        }

        AppCan_TpResetTx(&g_tp_channels[i]);
        AppCan_TpResetRx(&g_tp_channels[i]);
    }

    g_tp_channel_count = g_appCanTpChannelCount;
    return pdPASS;
}

static void AppCan_TpMainFunction(void)
{
    uint8_t i;
    TickType_t now = xTaskGetTickCount();

    for(i = 0u; i < g_tp_channel_count; i++)
    {
        AppCanTpChannel *ch = &g_tp_channels[i];

        if((ch->tx_state == APP_CAN_TP_TX_WAIT_FC) &&
           (AppCan_TickExpired(now, ch->tx_deadline_tick) == pdTRUE))
        {
            AppCan_TpResetTx(ch);
            AppCan_TpReportError(ch, APP_CAN_TP_ERROR_TIMEOUT_BS);
        }

        if((ch->rx_state == APP_CAN_TP_RX_IN_PROGRESS) &&
           (AppCan_TickExpired(now, ch->rx_deadline_tick) == pdTRUE))
        {
            AppCan_TpResetRx(ch);
            AppCan_TpReportError(ch, APP_CAN_TP_ERROR_TIMEOUT_CR);
        }

        if(ch->tx_state == APP_CAN_TP_TX_SEND_CF)
        {
            if(AppCan_TickExpired(now, ch->tx_next_cf_tick) == pdTRUE)
            {
                (void)AppCan_TpSendNextConsecutiveFrame(ch);
            }
        }
    }
}

static AppCanTpChannel *AppCan_TpFindByChannelId(uint8_t channel_id)
{
    uint8_t i;

    for(i = 0u; i < g_tp_channel_count; i++)
    {
        if(g_tp_channels[i].route.channel_id == channel_id)
        {
            return &g_tp_channels[i];
        }
    }

    return NULL;
}

static AppCanTpChannel *AppCan_TpFindByRxCanId(uint32_t rx_can_id)
{
    uint8_t i;

    for(i = 0u; i < g_tp_channel_count; i++)
    {
        if(g_tp_channels[i].route.rx_can_id == rx_can_id)
        {
            return &g_tp_channels[i];
        }
    }

    return NULL;
}

static BaseType_t AppCan_TpOnCanRx(const AppCanFrame *frame)
{
    AppCanTpChannel *ch;
    uint8_t pci_type;

    if((frame == NULL) || (frame->length == 0u))
    {
        return pdFAIL;
    }

    ch = AppCan_TpFindByRxCanId(frame->id);
    if(ch == NULL)
    {
        return pdFAIL;
    }

    /* 현재 구현은 Classical CAN ISO-TP 기준이다. CAN FD frame은 raw route에서 처리한다. */
    if(frame->is_fd != 0u)
    {
        AppCan_TpReportError(ch, APP_CAN_TP_ERROR_UNSUPPORTED_FRAME);
        return pdPASS;
    }

    pci_type = frame->data[0] & APP_CAN_TP_PCI_TYPE_MASK;

    switch(pci_type)
    {
        case APP_CAN_TP_PCI_SF:
            AppCan_TpHandleSingleFrame(ch, frame);
            break;

        case APP_CAN_TP_PCI_FF:
            AppCan_TpHandleFirstFrame(ch, frame);
            break;

        case APP_CAN_TP_PCI_CF:
            AppCan_TpHandleConsecutiveFrame(ch, frame);
            break;

        case APP_CAN_TP_PCI_FC:
            AppCan_TpHandleFlowControl(ch, frame);
            break;

        default:
            AppCan_TpReportError(ch, APP_CAN_TP_ERROR_UNSUPPORTED_FRAME);
            break;
    }

    return pdPASS;
}

static void AppCan_TpHandleSingleFrame(AppCanTpChannel *ch, const AppCanFrame *frame)
{
    uint8_t length;

    if((ch == NULL) || (frame == NULL))
    {
        return;
    }

    length = frame->data[0] & APP_CAN_TP_PCI_LENGTH_MASK;

    if((length > APP_CAN_TP_SINGLE_FRAME_MAX_PAYLOAD) ||
       ((uint8_t)(length + 1u) > frame->length))
    {
        AppCan_TpReportError(ch, APP_CAN_TP_ERROR_UNSUPPORTED_FRAME);
        return;
    }

    AppCan_TpResetRx(ch);
    if(length > 0u)
    {
        memcpy(ch->rx_buffer, &frame->data[1], length);
    }
    ch->rx_expected_length = length;
    ch->rx_offset = length;

    AppCan_TpDeliverMessage(ch);
    AppCan_TpResetRx(ch);
}

static void AppCan_TpHandleFirstFrame(AppCanTpChannel *ch, const AppCanFrame *frame)
{
    uint16_t total_length;
    uint8_t copy_length;

    if((ch == NULL) || (frame == NULL))
    {
        return;
    }

    if(frame->length < APP_CAN_TP_CLASSIC_DLC)
    {
        AppCan_TpReportError(ch, APP_CAN_TP_ERROR_UNSUPPORTED_FRAME);
        return;
    }

    total_length = (uint16_t)(((uint16_t)(frame->data[0] & APP_CAN_TP_PCI_LENGTH_MASK) << 8u) |
                              (uint16_t)frame->data[1]);

    if((total_length <= APP_CAN_TP_SINGLE_FRAME_MAX_PAYLOAD) ||
       (total_length > APP_CAN_TP_MAX_PAYLOAD_SIZE) ||
       (total_length > APP_CAN_TP_ISO_MAX_PAYLOAD_SIZE))
    {
        (void)AppCan_TpSendFlowControl(ch, APP_CAN_TP_FS_OVERFLOW);
        AppCan_TpResetRx(ch);
        AppCan_TpReportError(ch, APP_CAN_TP_ERROR_RX_OVERFLOW);
        return;
    }

    AppCan_TpResetRx(ch);

    copy_length = AppCan_MinPayloadCopyLength(total_length, APP_CAN_TP_FIRST_FRAME_DATA_SIZE);
    memcpy(ch->rx_buffer, &frame->data[2], copy_length);

    ch->rx_state = APP_CAN_TP_RX_IN_PROGRESS;
    ch->rx_expected_length = total_length;
    ch->rx_offset = copy_length;
    ch->rx_expected_sn = 1u;
    ch->rx_cf_since_fc = 0u;
    ch->rx_deadline_tick = xTaskGetTickCount() + AppCan_TpGetNCrTimeout(ch);

    (void)AppCan_TpSendFlowControl(ch, APP_CAN_TP_FS_CTS);
}

static void AppCan_TpHandleConsecutiveFrame(AppCanTpChannel *ch, const AppCanFrame *frame)
{
    uint8_t sequence_number;
    uint16_t remain;
    uint8_t copy_length;

    if((ch == NULL) || (frame == NULL))
    {
        return;
    }

    if((ch->rx_state != APP_CAN_TP_RX_IN_PROGRESS) || (frame->length < 2u))
    {
        AppCan_TpReportError(ch, APP_CAN_TP_ERROR_UNSUPPORTED_FRAME);
        return;
    }

    sequence_number = frame->data[0] & APP_CAN_TP_SN_MASK;
    if(sequence_number != ch->rx_expected_sn)
    {
        AppCan_TpResetRx(ch);
        AppCan_TpReportError(ch, APP_CAN_TP_ERROR_WRONG_SEQUENCE);
        return;
    }

    remain = (uint16_t)(ch->rx_expected_length - ch->rx_offset);
    copy_length = AppCan_MinPayloadCopyLength(remain, APP_CAN_TP_CONSEC_FRAME_DATA_SIZE);

    if((uint16_t)(ch->rx_offset + copy_length) > APP_CAN_TP_MAX_PAYLOAD_SIZE)
    {
        AppCan_TpResetRx(ch);
        AppCan_TpReportError(ch, APP_CAN_TP_ERROR_RX_OVERFLOW);
        return;
    }

    memcpy(&ch->rx_buffer[ch->rx_offset], &frame->data[1], copy_length);
    ch->rx_offset = (uint16_t)(ch->rx_offset + copy_length);
    ch->rx_expected_sn = (uint8_t)((ch->rx_expected_sn + 1u) & APP_CAN_TP_SN_MASK);
    ch->rx_cf_since_fc++;
    ch->rx_deadline_tick = xTaskGetTickCount() + AppCan_TpGetNCrTimeout(ch);

    if(ch->rx_offset >= ch->rx_expected_length)
    {
        AppCan_TpDeliverMessage(ch);
        AppCan_TpResetRx(ch);
        return;
    }

    if((ch->route.local_block_size != 0u) &&
       (ch->rx_cf_since_fc >= ch->route.local_block_size))
    {
        ch->rx_cf_since_fc = 0u;
        (void)AppCan_TpSendFlowControl(ch, APP_CAN_TP_FS_CTS);
    }
}

static void AppCan_TpHandleFlowControl(AppCanTpChannel *ch, const AppCanFrame *frame)
{
    uint8_t flow_status;

    if((ch == NULL) || (frame == NULL))
    {
        return;
    }

    if(frame->length < 3u)
    {
        AppCan_TpReportError(ch, APP_CAN_TP_ERROR_UNSUPPORTED_FRAME);
        return;
    }

    if((ch->tx_state != APP_CAN_TP_TX_WAIT_FC) &&
       (ch->tx_state != APP_CAN_TP_TX_SEND_CF))
    {
        return;
    }

    flow_status = frame->data[0] & APP_CAN_TP_PCI_LENGTH_MASK;

    switch(flow_status)
    {
        case APP_CAN_TP_FS_CTS:
            ch->tx_remote_block_size = frame->data[1];
            ch->tx_cf_since_fc = 0u;
            ch->tx_st_min_ticks = AppCan_TpStMinToTicks(frame->data[2]);
            ch->tx_next_cf_tick = xTaskGetTickCount();
            ch->tx_state = APP_CAN_TP_TX_SEND_CF;
            break;

        case APP_CAN_TP_FS_WAIT:
            ch->tx_deadline_tick = xTaskGetTickCount() + AppCan_TpGetNBsTimeout(ch);
            break;

        case APP_CAN_TP_FS_OVERFLOW:
            AppCan_TpResetTx(ch);
            AppCan_TpReportError(ch, APP_CAN_TP_ERROR_FLOW_CONTROL_OVERFLOW);
            break;

        default:
            AppCan_TpResetTx(ch);
            AppCan_TpReportError(ch, APP_CAN_TP_ERROR_UNSUPPORTED_FRAME);
            break;
    }
}

static BaseType_t AppCan_TpSendFlowControl(AppCanTpChannel *ch, uint8_t flow_status)
{
    uint8_t frame[APP_CAN_TP_CLASSIC_DLC];

    if(ch == NULL)
    {
        return pdFAIL;
    }

    memset(frame, ch->route.padding_byte, sizeof(frame));
    frame[0] = (uint8_t)(APP_CAN_TP_PCI_FC | (flow_status & APP_CAN_TP_PCI_LENGTH_MASK));
    frame[1] = ch->route.local_block_size;
    frame[2] = ch->route.local_st_min_ms;

    return AppCan_SendClassic(ch->route.tx_can_id, frame, APP_CAN_TP_CLASSIC_DLC);
}

static BaseType_t AppCan_TpSendNextConsecutiveFrame(AppCanTpChannel *ch)
{
    uint8_t frame[APP_CAN_TP_CLASSIC_DLC];
    uint16_t remain;
    uint8_t copy_length;
    BaseType_t result;

    if(ch == NULL)
    {
        return pdFAIL;
    }

    if(ch->tx_state != APP_CAN_TP_TX_SEND_CF)
    {
        return pdFAIL;
    }

    if(ch->tx_offset >= ch->tx_length)
    {
        AppCan_TpResetTx(ch);
        return pdPASS;
    }

    memset(frame, ch->route.padding_byte, sizeof(frame));
    frame[0] = (uint8_t)(APP_CAN_TP_PCI_CF | (ch->tx_next_sn & APP_CAN_TP_SN_MASK));

    remain = (uint16_t)(ch->tx_length - ch->tx_offset);
    copy_length = AppCan_MinPayloadCopyLength(remain, APP_CAN_TP_CONSEC_FRAME_DATA_SIZE);
    memcpy(&frame[1], &ch->tx_buffer[ch->tx_offset], copy_length);

    result = AppCan_SendClassic(ch->route.tx_can_id, frame, APP_CAN_TP_CLASSIC_DLC);
    if(result != pdPASS)
    {
        return pdFAIL;
    }

    ch->tx_offset = (uint16_t)(ch->tx_offset + copy_length);
    ch->tx_next_sn = (uint8_t)((ch->tx_next_sn + 1u) & APP_CAN_TP_SN_MASK);
    ch->tx_cf_since_fc++;

    if(ch->tx_offset >= ch->tx_length)
    {
        AppCan_TpResetTx(ch);
        return pdPASS;
    }

    if((ch->tx_remote_block_size != 0u) &&
       (ch->tx_cf_since_fc >= ch->tx_remote_block_size))
    {
        ch->tx_state = APP_CAN_TP_TX_WAIT_FC;
        ch->tx_deadline_tick = xTaskGetTickCount() + AppCan_TpGetNBsTimeout(ch);
        return pdPASS;
    }

    ch->tx_next_cf_tick = xTaskGetTickCount() + ch->tx_st_min_ticks;
    return pdPASS;
}

static void AppCan_TpResetTx(AppCanTpChannel *ch)
{
    if(ch == NULL)
    {
        return;
    }

    ch->tx_state = APP_CAN_TP_TX_IDLE;
    ch->tx_length = 0u;
    ch->tx_offset = 0u;
    ch->tx_next_sn = 1u;
    ch->tx_remote_block_size = 0u;
    ch->tx_cf_since_fc = 0u;
    ch->tx_st_min_ticks = 0u;
    ch->tx_next_cf_tick = 0u;
    ch->tx_deadline_tick = 0u;
}

static void AppCan_TpResetRx(AppCanTpChannel *ch)
{
    if(ch == NULL)
    {
        return;
    }

    ch->rx_state = APP_CAN_TP_RX_IDLE;
    ch->rx_expected_length = 0u;
    ch->rx_offset = 0u;
    ch->rx_expected_sn = 1u;
    ch->rx_cf_since_fc = 0u;
    ch->rx_deadline_tick = 0u;
}

static void AppCan_TpDeliverMessage(AppCanTpChannel *ch)
{
    AppCanTpRxMsg rx_msg;

    if((ch == NULL) || (ch->rx_queue == NULL))
    {
        return;
    }

    memset(&rx_msg, 0, sizeof(rx_msg));
    rx_msg.msg_type = APP_CAN_TP_RX_MESSAGE;
    rx_msg.channel_id = ch->route.channel_id;
    rx_msg.error = APP_CAN_TP_ERROR_NONE;
    rx_msg.length = ch->rx_expected_length;

    if(ch->rx_expected_length > 0u)
    {
        memcpy(rx_msg.data, ch->rx_buffer, ch->rx_expected_length);
    }

    if(xQueueSend(ch->rx_queue, &rx_msg, 0u) != pdPASS)
    {
        g_rx_drop_count++;
    }
}

static void AppCan_TpReportError(AppCanTpChannel *ch, AppCanTpError error)
{
    AppCanTpRxMsg rx_msg;

    if((ch == NULL) || (ch->rx_queue == NULL))
    {
        return;
    }

    memset(&rx_msg, 0, sizeof(rx_msg));
    rx_msg.msg_type = APP_CAN_TP_RX_ERROR;
    rx_msg.channel_id = ch->route.channel_id;
    rx_msg.error = error;
    rx_msg.length = 0u;

    if(xQueueSend(ch->rx_queue, &rx_msg, 0u) != pdPASS)
    {
        g_rx_drop_count++;
    }
}

static TickType_t AppCan_TpGetNBsTimeout(const AppCanTpChannel *ch)
{
    if((ch != NULL) && (ch->route.n_bs_timeout_ticks != 0u))
    {
        return ch->route.n_bs_timeout_ticks;
    }

    return pdMS_TO_TICKS(APP_CAN_DEFAULT_N_BS_MS);
}

static TickType_t AppCan_TpGetNCrTimeout(const AppCanTpChannel *ch)
{
    if((ch != NULL) && (ch->route.n_cr_timeout_ticks != 0u))
    {
        return ch->route.n_cr_timeout_ticks;
    }

    return pdMS_TO_TICKS(APP_CAN_DEFAULT_N_CR_MS);
}

static TickType_t AppCan_TpStMinToTicks(uint8_t st_min)
{
    TickType_t ticks;

    if(st_min <= 0x7Fu)
    {
        ticks = pdMS_TO_TICKS(st_min);
        if((st_min > 0u) && (ticks == 0u))
        {
            ticks = 1u;
        }
        return ticks;
    }

    /* 0xF1..0xF9는 100us 단위지만 FreeRTOS tick 기반이므로 1 tick으로 근사한다. */
    if((st_min >= 0xF1u) && (st_min <= 0xF9u))
    {
        return 1u;
    }

    return 0u;
}

static BaseType_t AppCan_TickExpired(TickType_t now, TickType_t deadline)
{
    return (now >= deadline) ? pdTRUE : pdFALSE;
}

static uint8_t AppCan_MinPayloadCopyLength(uint16_t remain, uint8_t max_copy)
{
    return (remain >= (uint16_t)max_copy) ? max_copy : (uint8_t)remain;
}
