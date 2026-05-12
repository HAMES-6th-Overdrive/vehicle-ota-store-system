#include "Ifx_Types.h"
#include "IfxAsclin_Asc.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "Ifx_Cfg_Ssw.h"

#define PI_START_BYTE   0x3B
#define PI_END_BYTE     0x0D

// ==========================================
// UART 핸들러
// ASCLIN0 : 라즈베리파이 수신 / 자동차 송신
// RX : P15.3 (X304-1)
// TX : P15.2 (X304-2)
// ==========================================

IfxAsclin_Asc g_asc;
IfxCpu_syncEvent cpuSyncEvent = 0;

uint8 frame[11];

// ==========================================
// FIFO 버퍼
// ==========================================

#define TX_BUFFER_SIZE  (64 + sizeof(Ifx_Fifo) + 8)
#define RX_BUFFER_SIZE  (64 + sizeof(Ifx_Fifo) + 8)

IFX_ALIGN(4) uint8 g_txBuffer[64 + sizeof(Ifx_Fifo) + 8];
IFX_ALIGN(4) uint8 g_rxBuffer[64 + sizeof(Ifx_Fifo) + 8];

// ==========================================
// 인터럽트 우선순위
// ==========================================

#define ISR_PRIORITY_TX  10
#define ISR_PRIORITY_RX  11
#define ISR_PRIORITY_ER  12

// ==========================================
// ISR 핸들러
// ==========================================

IFX_INTERRUPT(asclin0TxISR, 0, ISR_PRIORITY_TX)
{
    IfxAsclin_Asc_isrTransmit(&g_asc);
}

IFX_INTERRUPT(asclin0RxISR, 0, ISR_PRIORITY_RX)
{
    IfxAsclin_Asc_isrReceive(&g_asc);
}

IFX_INTERRUPT(asclin0ErISR, 0, ISR_PRIORITY_ER)
{
    IfxAsclin_Asc_isrError(&g_asc);
}

// ==========================================
// 수신 데이터
// ==========================================

volatile uint8 speed_value = 127;
volatile uint8 steer_value = 127;

// ==========================================
// UART 상태 머신
// ==========================================

typedef enum
{
    WAIT_START,
    RECEIVE_SPEED,
    RECEIVE_STEER,
    WAIT_END

} UART_STATE;

UART_STATE uart_state = WAIT_START;

// ==========================================
// UART 초기화
// ==========================================

void initUART(void)
{
    IfxAsclin_Asc_Config ascConfig;
    IfxAsclin_Asc_initModuleConfig(&ascConfig, &MODULE_ASCLIN0);

    ascConfig.baudrate.baudrate  = 9600;
    ascConfig.baudrate.prescaler = 1;

    ascConfig.interrupt.txPriority    = ISR_PRIORITY_TX;
    ascConfig.interrupt.rxPriority    = ISR_PRIORITY_RX;
    ascConfig.interrupt.erPriority    = ISR_PRIORITY_ER;
    ascConfig.interrupt.typeOfService = IfxSrc_Tos_cpu0;

    ascConfig.txBuffer     = g_txBuffer;
    ascConfig.txBufferSize = 64;
    ascConfig.rxBuffer     = g_rxBuffer;
    ascConfig.rxBufferSize = 64;

    const IfxAsclin_Asc_Pins pins =
    {
        NULL,                           IfxPort_InputMode_pullUp,
        &IfxAsclin0_RXB_P15_3_IN,      IfxPort_InputMode_pullUp,
        NULL,                           IfxPort_OutputMode_pushPull,
        &IfxAsclin0_TX_P15_2_OUT,      IfxPort_OutputMode_pushPull,
        IfxPort_PadDriver_cmosAutomotiveSpeed1
    };

    ascConfig.pins = &pins;

    // IfxCpu_Irq_installInterruptHandler 3줄 제거

    IfxAsclin_Asc_initModule(&g_asc, &ascConfig);
}

// ==========================================
// UART 1Byte 수신
// ==========================================

uint8 uartReceiveByte(void)
{
    return IfxAsclin_Asc_blockingRead(&g_asc);
}

// ==========================================
// UART 바이트 배열 송신 (길이 기반)
// ==========================================

void uartSendBytes(uint8 *data, uint32 len)
{
    for(uint32 i = 0; i < len; i++)
    {
        // FIFO 비워질 때까지 대기 후 송신
        IfxAsclin_Asc_flushTx(&g_asc, TIME_INFINITE);
        IfxAsclin_Asc_blockingWrite(&g_asc, data[i]);
    }
}

// ==========================================
// 0~255 → 3자리 HEX 변환
// ==========================================

void convertThrottle(uint8 speed, char *out)
{
    int value;

    // -------------------------------
    // 중립 = 0x3B3
    // 최대 전진 = 0x440
    // 최대 후진 = 0x36D
    // -------------------------------

    if(speed < 127)
    {
        value = 0x3B3 +
                ((127 - speed) * (0x440 - 0x3B3)) / 127;
    }
    else if(speed > 127)
    {
        value = 0x3B3 -
                ((speed - 127) * (0x3B3 - 0x36D)) / 128;
    }
    else
    {
        value = 0x3B3;
    }

    const char hex[] = "0123456789ABCDEF";
    out[0] = hex[(value >> 8) & 0xF];
    out[1] = hex[(value >> 4) & 0xF];
    out[2] = hex[(value >> 0) & 0xF];
    out[3] = '\0';
}

// ==========================================
// 조향 변환
// ==========================================

void convertSteering(uint8 steer, char *out)
{
    int value;

    // -------------------------------
    // 중립 = 0x381
    // 최대 좌회전 = 0x253
    // 최대 우회전 = 0x505
    // -------------------------------

    if(steer < 127)
    {
        value = 0x381 -
                ((127 - steer) * (0x381 - 0x253)) / 127;
    }
    else if(steer > 127)
    {
        value = 0x381 +
                ((steer - 127) * (0x505 - 0x381)) / 128;
    }
    else
    {
        value = 0x381;
    }

    const char hex[] = "0123456789ABCDEF";
    out[0] = hex[(value >> 8) & 0xF];
    out[1] = hex[(value >> 4) & 0xF];
    out[2] = hex[(value >> 0) & 0xF];
    out[3] = '\0';
}

// ==========================================
// 자동차 프레임 생성 및 송신
// ==========================================

void sendCarFrame(uint8 speed, uint8 steer)
{
    char throttleStr[4];
    char steeringStr[4];

    convertThrottle(speed, throttleStr);
    convertSteering(steer, steeringStr);

    // -------------------------------
    // 프레임 구성
    // NULL STX [Throttle 3자리] [Steer 3자리] 0 1 ETX
    // 총 11바이트
    // -------------------------------


    frame[0]  = 0x00;
    frame[1]  = 0x02;
    frame[2]  = throttleStr[0];
    frame[3]  = throttleStr[1];
    frame[4]  = throttleStr[2];
    frame[5]  = steeringStr[0];
    frame[6]  = steeringStr[1];
    frame[7]  = steeringStr[2];
    frame[8]  = '0';
    frame[9]  = '1';
    frame[10] = 0x03;

    uartSendBytes(frame, 11);
}

// ==========================================
// MAIN
// ==========================================

int core0_main(void)
{
    // 1. 인터럽트 먼저 활성화
    IfxCpu_enableInterrupts();

    // 2. 워치독 비활성화 (안하면 리셋/트랩 발생)
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    // 3. 그 다음 동기화
    IfxCpu_emitEvent(&cpuSyncEvent);
    IfxCpu_waitEvent(&cpuSyncEvent, 1);

    // 4. UART 초기화
    initUART();

    while(1)
    {
        if(IfxAsclin_Asc_canReadCount(&g_asc, 1, 0))
        {
            uint8 rx = IfxAsclin_Asc_blockingRead(&g_asc);

            switch(uart_state)
            {
                case WAIT_START:
                    if(rx == PI_START_BYTE)
                    {
                        uart_state = RECEIVE_SPEED;
                    }
                    break;

                case RECEIVE_SPEED:
                    speed_value = rx;
                    uart_state  = RECEIVE_STEER;
                    break;

                case RECEIVE_STEER:
                    steer_value = rx;
                    uart_state  = WAIT_END;
                    break;

                case WAIT_END:
                    if(rx == PI_END_BYTE)
                    {
                        sendCarFrame(speed_value, steer_value);
                    }
                    uart_state = WAIT_START;
                    break;

                default:
                    uart_state = WAIT_START;
                    break;
            }
        }
    }
    return 0;
}
