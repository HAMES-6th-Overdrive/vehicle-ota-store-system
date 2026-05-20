# App_Can Interface Guide

`App_Can`은 TC375 MCMCAN을 FreeRTOS 앱 레벨에서 쉽게 쓰기 위한 CAN 통합 모듈입니다.

이 모듈 하나가 CAN 하드웨어 초기화, raw Classical CAN/CAN FD 송수신, CAN ID별 수신 mailbox, Classical CAN ISO-TP 송수신을 담당합니다. 상위 앱은 CAN 하드웨어 레지스터나 iLLD 세부 구현을 직접 만지지 않고 `App_Can.h`에 공개된 함수만 호출하면 됩니다.

## 핵심 개념

`App_Can`의 기본 설계는 **App별 라우팅이 아니라 CAN ID별 mailbox**입니다.

CAN frame에는 이미 CAN ID가 들어 있고, TC375 MCMCAN은 acceptance filter로 받을 ID를 하드웨어에서 먼저 거릅니다. 그래서 SOME/IP처럼 service ID를 보고 앱 queue로 다시 라우팅하는 구조보다, 앱이 필요한 CAN ID를 직접 읽는 구조가 CAN에 더 잘 맞습니다.

수신 흐름은 다음과 같습니다.

1. `App_Can.c` 상단의 설정 테이블에 받을 CAN ID를 등록합니다.
2. `AppCan_Start()`가 MCMCAN, RX filter, 내부 queue, FreeRTOS task를 초기화합니다.
3. CAN RX ISR이 수신 frame을 내부 queue에 넣습니다.
4. `AppCan_Task`가 내부 queue를 비우며 CAN ID별 raw mailbox 또는 ISO-TP channel로 전달합니다.
5. 상위 앱은 `AppCan_RecvById()`, `AppCan_ReadLatestById()`, `AppCan_TpRecv()`로 데이터를 읽습니다.

## 파일 구성

- `App_Can.h`: 상위 앱에 공개되는 인터페이스입니다. 앱 코드는 이 헤더만 include합니다.
- `App_Can.c`: MCMCAN 초기화, ISR, FreeRTOS task, raw mailbox, ISO-TP 구현이 들어 있습니다.
- `README.md`: 지금 보고 있는 사용자/설계 설명서입니다.

## 현재 하드웨어 전제

현재 구현은 다음 보드를 기준으로 작성되어 있습니다.

- MCU: AURIX TC375 계열
- CAN 모듈: CAN0
- CAN node: Node0
- TX pin: `P20.8`
- RX pin: `P20.7`
- Transceiver STB pin: `P20.6`, LOW 활성
- CAN ID: Standard 11-bit ID만 지원
- Raw frame: Classical CAN, CAN FD 지원
- ISO-TP: Classical CAN normal addressing 지원

핀 또는 CAN node를 바꾸려면 `App_Can.c`의 아래 설정을 수정해야 합니다.

```c
#define APP_CAN_USED_NODE_ID                 IfxCan_NodeId_0

static const IfxCan_Can_Pins g_app_can_pins = {
    &IfxCan_TXD00_P20_8_OUT, IfxPort_OutputMode_pushPull,
    &IfxCan_RXD00B_P20_7_IN, IfxPort_InputMode_noPullDevice,
    IfxPort_PadDriver_cmosAutomotiveSpeed1
};
```

Transceiver STB pin은 `AppCan_InitTransceiver()`에서 설정합니다.

```c
IfxPort_setPinModeOutput(&MODULE_P20, 6, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
IfxPort_setPinLow(&MODULE_P20, 6);
```

## 빌드 주의사항

`App_Can.c`는 iLLD CAN 드라이버를 사용합니다. 프로젝트 빌드 설정에서 CAN iLLD 소스가 제외되어 있으면 링크 에러가 납니다.

필요한 대표 소스/헤더는 다음 계열입니다.

- `Libraries/iLLD/TC37A/Tricore/Can/Can/IfxCan_Can.c`
- `Libraries/iLLD/TC37A/Tricore/Can/Std/IfxCan.c`
- `Libraries/iLLD/TC37A/Tricore/_PinMap/IfxCan_PinMap.c`

ADS `.cproject`에서 `Libraries/iLLD/TC37A/Tricore/Can`, `Can/Can`, `Can/Std`가 exclude 되어 있으면 `App_Can.c`를 추가해도 빌드되지 않습니다.

## 인터럽트 우선순위

현재 CAN ISR 우선순위는 `App_Can.c` 내부에서만 정의합니다.

```c
#define APP_CAN_ISR_PRIORITY_TX              (10u)
#define APP_CAN_ISR_PRIORITY_RX              (11u)
```

FreeRTOS 포트가 priority `1`, `2`를 context/tick interrupt로 사용하므로 CAN ISR은 그 값과 겹치면 안 됩니다. 또한 CAN RX ISR은 `xQueueSendFromISR()`를 호출하므로 `configMAX_API_CALL_INTERRUPT_PRIORITY` 이하의 우선순위를 사용해야 합니다.

## 빠른 시작

상위 앱에서는 `App_Can.h`만 include합니다.

```c
#include "App_Can/App_Can.h"

void App_StartAll(void)
{
    BaseType_t result;

    result = AppCan_Start();
    configASSERT(result == pdPASS);
}
```

`AppCan_Start()`는 scheduler 시작 전에 호출하는 것을 기준으로 설계되어 있습니다. 예를 들어 `Cpu0_Main.c`에서 다른 앱 태스크 시작 함수들과 같은 위치에 호출하면 됩니다.

```c
AppCan_Start();
AppEth_Start();
AppSomeip_Start();
AppInfoService_Start();

vTaskStartScheduler();
```

## 공개 데이터 타입

### `AppCanFrame`

Raw CAN/CAN FD frame을 표현합니다.

```c
typedef struct AppCanFrame {
    uint32_t id;
    uint8_t  length;
    uint8_t  is_fd;
    uint8_t  data[APP_CAN_FD_MAX_DLC];
} AppCanFrame;
```

필드 의미:

- `id`: Standard 11-bit CAN ID입니다. 유효 범위는 `0x000`부터 `0x7FF`까지입니다.
- `length`: payload 길이입니다. Classical CAN은 `0..8`, CAN FD는 `0..64`입니다.
- `is_fd`: `0`이면 Classical CAN, `1`이면 CAN FD입니다.
- `data`: payload byte buffer입니다. 유효 데이터는 `data[0]`부터 `data[length - 1]`까지입니다.

### `AppCanTpRxMsg`

ISO-TP 수신 결과를 표현합니다.

```c
typedef struct AppCanTpRxMsg {
    AppCanTpRxType msg_type;
    uint8_t        channel_id;
    AppCanTpError  error;
    uint16_t       length;
    uint8_t        data[APP_CAN_TP_MAX_PAYLOAD_SIZE];
} AppCanTpRxMsg;
```

`msg_type`이 `APP_CAN_TP_RX_MESSAGE`이면 정상 메시지이고, `APP_CAN_TP_RX_ERROR`이면 `error` 필드를 확인해야 합니다.

## Raw CAN 수신 설정

수신할 raw CAN ID는 `App_Can.c` 상단의 `g_appCanRawRxObjectConfigs`에 등록합니다.

```c
#define APP_CAN_RAW_RX_OBJECT_COUNT (2u)

static const AppCanRawRxObjectConfig g_appCanRawRxObjectConfigs[APP_CAN_MAX_RAW_RX_OBJECTS] = {
    { 0x201u, 4u, 1u },
    { 0x202u, 1u, 1u }
};
```

각 항목의 의미는 다음과 같습니다.

```c
{ can_id, queue_length, keep_latest }
```

- `can_id`: 받을 Standard CAN ID입니다.
- `queue_length`: 순서 보존 수신 queue 길이입니다. `0`이면 `AppCan_RecvById()`로 읽을 수 없습니다.
- `keep_latest`: 최신 frame 보관 여부입니다. `1`이면 `AppCan_ReadLatestById()`로 마지막 frame을 읽을 수 있습니다.

둘 다 `0`이면 그 ID를 등록할 의미가 없으므로 초기화가 실패합니다.

### Queue 방식과 latest 방식의 차이

`queue_length > 0`은 frame을 순서대로 처리해야 할 때 씁니다. 예를 들어 진단 응답, 이벤트성 메시지, 손실 없이 처리하고 싶은 제어 메시지에 적합합니다.

`keep_latest = 1`은 최신값만 중요할 때 씁니다. 예를 들어 속도, 거리, 센서값처럼 주기적으로 계속 갱신되는 신호는 이전 frame을 모두 처리할 필요가 없을 수 있습니다.

둘을 함께 켤 수도 있습니다. 이 경우 순서 처리도 가능하고 최신값 조회도 가능합니다.

## Raw CAN 수신 API

### `AppCan_RecvById`

```c
BaseType_t AppCan_RecvById(uint32_t can_id, AppCanFrame *frame, TickType_t wait_ticks);
```

등록된 CAN ID의 queue에서 frame 하나를 꺼냅니다.

반환값:

- `pdPASS`: frame 수신 성공
- `pdFAIL`: 잘못된 인자, 미등록 CAN ID, queue 미설정, timeout 등

사용 예:

```c
static void SensorTask(void *arg)
{
    AppCanFrame frame;

    (void)arg;

    for (;;)
    {
        if (AppCan_RecvById(0x201u, &frame, pdMS_TO_TICKS(10u)) == pdPASS)
        {
            /* frame.data 해석 */
        }

        vTaskDelay(pdMS_TO_TICKS(1u));
    }
}
```

주의:

- 해당 ID의 `queue_length`가 `0`이면 항상 실패합니다.
- 같은 CAN ID를 여러 task가 동시에 `RecvById()`로 읽으면 하나의 queue를 나눠 읽게 됩니다. 보통 한 ID는 한 task가 소유하는 식으로 설계하는 것이 좋습니다.

### `AppCan_ReadLatestById`

```c
BaseType_t AppCan_ReadLatestById(uint32_t can_id, AppCanFrame *frame);
```

등록된 CAN ID의 최신 frame을 복사해서 읽습니다. queue에서 제거하는 방식이 아닙니다.

반환값:

- `pdPASS`: 최신 frame 읽기 성공
- `pdFAIL`: 잘못된 인자, 미등록 CAN ID, latest 미설정, 아직 수신된 frame 없음

사용 예:

```c
AppCanFrame latest;

if (AppCan_ReadLatestById(0x202u, &latest) == pdPASS)
{
    /* latest.data 사용 */
}
```

주의:

- 해당 ID의 `keep_latest`가 `0`이면 읽을 수 없습니다.
- 아직 한 번도 수신되지 않은 ID는 실패합니다.

### `AppCan_GetPendingCountById`

```c
UBaseType_t AppCan_GetPendingCountById(uint32_t can_id);
```

등록된 CAN ID queue에 쌓여 있는 frame 개수를 반환합니다.

사용 예:

```c
if (AppCan_GetPendingCountById(0x201u) > 0u)
{
    /* 처리할 frame 있음 */
}
```

## Raw CAN 송신 API

### `AppCan_SendClassic`

```c
BaseType_t AppCan_SendClassic(uint32_t id, const uint8_t *data, uint8_t length);
```

Classical CAN frame을 송신 queue에 넣습니다.

조건:

- `id <= 0x7FF`
- `length <= 8`
- `length > 0`이면 `data != NULL`

사용 예:

```c
uint8_t data[3] = { 0x7Fu, 0x7Fu, 0x01u };

if (AppCan_SendClassic(0x100u, data, 3u) != pdPASS)
{
    /* 송신 queue full 또는 인자 오류 */
}
```

### `AppCan_SendFd`

```c
BaseType_t AppCan_SendFd(uint32_t id, const uint8_t *data, uint8_t length);
```

CAN FD frame을 송신 queue에 넣습니다.

조건:

- `id <= 0x7FF`
- `length <= 64`
- `length > 0`이면 `data != NULL`

사용 예:

```c
uint8_t payload[16] = {0};

(void)AppCan_SendFd(0x700u, payload, 16u);
```

송신은 내부 software queue를 거쳐 진행됩니다. 하드웨어가 busy이면 다음 `AppCan_Task()` 주기 또는 TX 완료 ISR 이후 다시 송신을 시도합니다.

## ISO-TP 채널 설정

ISO-TP는 `channel_id` 단위로 사용합니다. 채널 설정은 `App_Can.c` 상단의 `g_appCanTpChannels`에 등록합니다.

```c
#define APP_CAN_TP_CHANNEL_COUNT (1u)

static const AppCanTpChannelConfig g_appCanTpChannels[APP_CAN_MAX_TP_CHANNELS] = {
    {
        .channel_id = 0u,
        .tx_can_id = 0x600u,
        .rx_can_id = 0x601u,
        .rx_queue_length = 2u,
        .padding_enabled = 1u,
        .padding_byte = 0x00u,
        .local_block_size = 0u,
        .local_st_min_ms = 0u,
        .n_bs_timeout_ticks = 0u,
        .n_cr_timeout_ticks = 0u
    }
};
```

필드 의미:

- `channel_id`: 상위 앱이 사용할 채널 번호입니다.
- `tx_can_id`: 로컬 ECU가 송신할 때 사용할 CAN ID입니다.
- `rx_can_id`: 상대 ECU에서 들어오는 ISO-TP frame의 CAN ID입니다.
- `rx_queue_length`: 완성된 ISO-TP 메시지 또는 에러 이벤트를 담을 queue 길이입니다. `0`이면 기본값 `APP_CAN_TP_DEFAULT_RX_QUEUE_LENGTH`를 사용합니다.
- `padding_enabled`: single frame 송신 시 8 byte padding 여부입니다.
- `padding_byte`: padding에 채울 byte입니다.
- `local_block_size`: 상대 ECU에게 전달할 block size입니다. `0`이면 추가 Flow Control 없이 계속 받겠다는 의미입니다.
- `local_st_min_ms`: 상대 ECU에게 요청할 consecutive frame 간 최소 간격입니다. `0..127ms` 범위로 제한됩니다.
- `n_bs_timeout_ticks`: 송신 중 Flow Control 대기 timeout입니다. `0`이면 기본값을 사용합니다.
- `n_cr_timeout_ticks`: 수신 중 Consecutive Frame 대기 timeout입니다. `0`이면 기본값을 사용합니다.

## ISO-TP 송수신 API

### `AppCan_TpSend`

```c
BaseType_t AppCan_TpSend(uint8_t channel_id, const uint8_t *payload, uint16_t length);
```

지정한 ISO-TP channel로 payload를 송신합니다.

조건:

- `channel_id`가 등록되어 있어야 합니다.
- `length <= APP_CAN_TP_MAX_PAYLOAD_SIZE`
- `length <= APP_CAN_TP_ISO_MAX_PAYLOAD_SIZE`
- `length > 0`이면 `payload != NULL`
- 해당 channel이 이미 송신 중이면 실패합니다.

사용 예:

```c
uint8_t uds_request[] = { 0x10u, 0x02u };

if (AppCan_TpSend(0u, uds_request, sizeof(uds_request)) != pdPASS)
{
    /* channel busy 또는 인자 오류 */
}
```

### `AppCan_TpRecv`

```c
BaseType_t AppCan_TpRecv(uint8_t channel_id, AppCanTpRxMsg *msg, TickType_t wait_ticks);
```

지정한 ISO-TP channel의 수신 queue에서 메시지 하나를 꺼냅니다.

사용 예:

```c
AppCanTpRxMsg msg;

if (AppCan_TpRecv(0u, &msg, pdMS_TO_TICKS(100u)) == pdPASS)
{
    if (msg.msg_type == APP_CAN_TP_RX_MESSAGE)
    {
        /* msg.data[0..msg.length-1] 사용 */
    }
    else
    {
        /* msg.error 확인 */
    }
}
```

### `AppCan_TpIsBusy`

```c
BaseType_t AppCan_TpIsBusy(uint8_t channel_id);
```

해당 ISO-TP channel이 송신 중이면 `pdTRUE`, idle이면 `pdFALSE`를 반환합니다.

사용 예:

```c
if (AppCan_TpIsBusy(0u) == pdFALSE)
{
    (void)AppCan_TpSend(0u, payload, payload_length);
}
```

## ISO-TP 에러 코드

`AppCanTpRxMsg.msg_type == APP_CAN_TP_RX_ERROR`이면 `error` 필드를 확인합니다.

- `APP_CAN_TP_ERROR_INVALID_PARAM`: 잘못된 인자 또는 설정
- `APP_CAN_TP_ERROR_BUSY`: 송신 중인 channel에 다시 송신 요청
- `APP_CAN_TP_ERROR_TX_FAILED`: CAN 송신 queue 입력 실패
- `APP_CAN_TP_ERROR_RX_OVERFLOW`: 수신 payload가 버퍼보다 큼
- `APP_CAN_TP_ERROR_WRONG_SEQUENCE`: Consecutive Frame sequence number 불일치
- `APP_CAN_TP_ERROR_TIMEOUT_BS`: Flow Control 대기 timeout
- `APP_CAN_TP_ERROR_TIMEOUT_CR`: Consecutive Frame 대기 timeout
- `APP_CAN_TP_ERROR_FLOW_CONTROL_OVERFLOW`: 상대가 Flow Control Overflow 응답
- `APP_CAN_TP_ERROR_UNSUPPORTED_FRAME`: 지원하지 않는 frame 형식

## 상태/진단 카운터

다음 함수들은 동작 상태를 확인하기 위한 카운터를 반환합니다.

```c
uint32_t AppCan_GetRxQueuedCount(void);
uint32_t AppCan_GetRxDropCount(void);
uint32_t AppCan_GetRxStoredCount(void);
uint32_t AppCan_GetRxNoOwnerCount(void);
uint32_t AppCan_GetTxQueuedCount(void);
uint32_t AppCan_GetTxSentCount(void);
uint32_t AppCan_GetTxBusyCount(void);
```

의미:

- `RxQueuedCount`: RX ISR이 내부 queue에 넣은 frame 수
- `RxDropCount`: 내부 queue 또는 raw mailbox queue가 가득 차서 버린 frame 수
- `RxStoredCount`: raw mailbox에 저장된 frame 수
- `RxNoOwnerCount`: 하드웨어 filter는 통과했지만 raw mailbox/ISO-TP channel 어느 쪽에도 매칭되지 않은 frame 수
- `TxQueuedCount`: 송신 요청이 software queue에 들어간 수
- `TxSentCount`: 하드웨어 송신 요청까지 진행된 수
- `TxBusyCount`: software queue full 또는 CAN hardware busy로 대기/실패한 수

디버깅 팁:

- `RxDropCount`가 증가하면 `APP_CAN_INTERNAL_RX_QUEUE_SIZE`, raw object `queue_length`, 또는 task 처리 주기를 점검합니다.
- `RxNoOwnerCount`가 증가하면 raw RX object와 ISO-TP `rx_can_id` 설정이 의도와 맞는지 확인합니다.
- `TxBusyCount`가 계속 증가하면 송신 요청 속도가 너무 빠르거나 bus/hardware 상태가 busy일 수 있습니다.

## 설정 매크로

`App_Can.h`에 공개된 주요 크기 설정은 다음과 같습니다.

```c
#define APP_CAN_CLASSIC_MAX_DLC               (8u)
#define APP_CAN_FD_MAX_DLC                    (64u)
#define APP_CAN_INTERNAL_RX_QUEUE_SIZE        (16u)
#define APP_CAN_TX_QUEUE_SIZE                 (8u)
#define APP_CAN_MAX_STD_RX_FILTERS            (8u)
#define APP_CAN_MAX_RAW_RX_OBJECTS            (8u)
#define APP_CAN_MAX_TP_CHANNELS               (4u)
#define APP_CAN_TP_MAX_PAYLOAD_SIZE           (512u)
```

`#ifndef`로 감싼 값들은 컴파일 옵션 또는 상위 설정에서 재정의할 수 있습니다. 다만 현재 raw object table과 TP channel table은 `App_Can.c` 내부 static 설정이므로, 실제 수신 ID와 ISO-TP channel 추가는 `App_Can.c`를 수정해야 합니다.

## 권장 사용 패턴

센서처럼 최신값만 중요한 신호:

```c
{ 0x202u, 0u, 1u }
```

이벤트처럼 순서대로 처리해야 하는 신호:

```c
{ 0x300u, 8u, 0u }
```

최신값 조회와 순서 처리 둘 다 필요한 신호:

```c
{ 0x201u, 4u, 1u }
```

UDS 같은 진단 채널:

```c
{
    .channel_id = 0u,
    .tx_can_id = 0x600u,
    .rx_can_id = 0x601u,
    .rx_queue_length = 2u,
    .padding_enabled = 1u,
    .padding_byte = 0x00u,
    .local_block_size = 0u,
    .local_st_min_ms = 0u,
    .n_bs_timeout_ticks = 0u,
    .n_cr_timeout_ticks = 0u
}
```

## 제한 사항

현재 구현에서 지원하지 않는 기능은 다음과 같습니다.

- Extended CAN ID, 29-bit ID
- ISO-TP extended addressing
- ISO-TP mixed addressing
- CAN FD 기반 ISO-TP
- CAN ID별 의미 해석
- UDS SID 해석
- 앱별 callback 라우팅

이 모듈은 “CAN transport/library layer”까지만 담당합니다. 예를 들어 `0x201`이 거리인지, `0x600`이 UDS 요청인지 같은 의미 해석은 상위 앱에서 처리해야 합니다.

## 체크리스트

App_Can을 새 프로젝트에 붙일 때는 다음을 확인합니다.

1. `AppCan_Start()`가 scheduler 시작 전에 호출되는가?
2. 빌드 설정에서 CAN iLLD 소스가 제외되어 있지 않은가?
3. `APP_CAN_ISR_PRIORITY_TX/RX`가 FreeRTOS context/tick ISR과 겹치지 않는가?
4. 사용하는 CAN ID가 `0x7FF` 이하인가?
5. 받을 raw CAN ID가 `g_appCanRawRxObjectConfigs`에 등록되어 있는가?
6. ISO-TP를 쓴다면 `g_appCanTpChannels`에 `tx_can_id/rx_can_id`가 맞게 등록되어 있는가?
7. 같은 CAN ID를 여러 task가 동시에 queue 방식으로 읽고 있지 않은가?
8. `RxDropCount`, `RxNoOwnerCount`, `TxBusyCount`가 비정상적으로 증가하지 않는가?
