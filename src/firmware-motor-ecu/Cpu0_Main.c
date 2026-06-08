/**********************************************************************************************************************
 * \file Cpu0_Main.c
 * \brief Motor ECU Main - CAN RX -> Vehicle UART TX
 *
 * 역할:
 *  - CAN 0x100 VehicleControlCmd 수신
 *  - driveCmd / steeringCmd / stopCmd를 차량 UART 프레임으로 변환
 *  - ASCLIN0 TX(P15.2)로 차량 제어보드에 송신
 *
 * 설계 기준:
 *  - Motor ECU는 GearState를 받지 않는다.
 *  - Motor ECU는 CAN 0x080 VehicleState를 사용하지 않는다.
 *  - Motor ECU는 CAN 0x100 VehicleControlCmd만 기준으로 차량 제어보드에 UART 프레임을 송신한다.
 *********************************************************************************************************************/

#include "Ifx_Types.h"
#include "IfxAsclin_Asc.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "IfxPort.h"
#include "Ifx_Cfg_Ssw.h"

#include "MCMCAN.h"
#include "can_type_def.h"

/* ============================================================
   Motor ECU MCMCAN.c에서 제공하는 함수
   MCMCAN.h에 선언하지 않았다면 여기 extern으로 선언
   ============================================================ */

extern boolean MotorCan_getLatestControlCmd(VehicleControlCmd_t *outCmd);

/* ============================================================
   ASCLIN0 UART
   차량 제어보드 송신용
   TX : P15.2
   RX : P15.3
   Baudrate : 9600
   ============================================================ */

static IfxAsclin_Asc g_asc;
IFX_ALIGN(4) IfxCpu_syncEvent g_cpuSyncEvent = 0;

static uint8 frame[11];

/* ============================================================
   FIFO Buffer
   ============================================================ */

#define TX_BUFFER_SIZE  (64 + sizeof(Ifx_Fifo) + 8)
#define RX_BUFFER_SIZE  (64 + sizeof(Ifx_Fifo) + 8)

IFX_ALIGN(4) static uint8 g_txBuffer[TX_BUFFER_SIZE];
IFX_ALIGN(4) static uint8 g_rxBuffer[RX_BUFFER_SIZE];

/* ============================================================
   ASCLIN Interrupt Priority
   CAN RX는 MCMCAN.h에서 ISR_PRIORITY_CAN_RX 사용
   UART는 기존처럼 10, 11, 12 사용
   ============================================================ */

#define ISR_PRIORITY_ASCLIN_TX  10
#define ISR_PRIORITY_ASCLIN_RX  11
#define ISR_PRIORITY_ASCLIN_ER  12

/* ============================================================
   ASCLIN ISR
   ============================================================ */

IFX_INTERRUPT(asclin0TxISR, 0, ISR_PRIORITY_ASCLIN_TX)
{
    IfxAsclin_Asc_isrTransmit(&g_asc);
}

IFX_INTERRUPT(asclin0RxISR, 0, ISR_PRIORITY_ASCLIN_RX)
{
    IfxAsclin_Asc_isrReceive(&g_asc);
}

IFX_INTERRUPT(asclin0ErISR, 0, ISR_PRIORITY_ASCLIN_ER)
{
    IfxAsclin_Asc_isrError(&g_asc);
}

/* ============================================================
   Private Function Prototypes
   ============================================================ */

static void initUART(void);
static void uartSendBytes(uint8 *data, uint32 len);

static void convertThrottle(uint8 speed, char *out);
static void convertSteering(uint8 steer, char *out);

static void sendCarFrame(uint8 speed, uint8 steer, StopCmd_t stopCmd);
static void handleVehicleControlCmd(const VehicleControlCmd_t *cmd);

/* ============================================================
   UART 초기화
   ============================================================ */

static void initUART(void)
{
    IfxAsclin_Asc_Config ascConfig;

    IfxAsclin_Asc_initModuleConfig(&ascConfig, &MODULE_ASCLIN0);

    ascConfig.baudrate.baudrate  = 9600;
    ascConfig.baudrate.prescaler = 1;

    ascConfig.interrupt.txPriority    = ISR_PRIORITY_ASCLIN_TX;
    ascConfig.interrupt.rxPriority    = ISR_PRIORITY_ASCLIN_RX;
    ascConfig.interrupt.erPriority    = ISR_PRIORITY_ASCLIN_ER;
    ascConfig.interrupt.typeOfService = IfxSrc_Tos_cpu0;

    ascConfig.txBuffer     = g_txBuffer;
    ascConfig.txBufferSize = 64;
    ascConfig.rxBuffer     = g_rxBuffer;
    ascConfig.rxBufferSize = 64;

    const IfxAsclin_Asc_Pins pins =
    {
        NULL,                          IfxPort_InputMode_pullUp,
        &IfxAsclin0_RXB_P15_3_IN,      IfxPort_InputMode_pullUp,
        NULL,                          IfxPort_OutputMode_pushPull,
        &IfxAsclin0_TX_P15_2_OUT,      IfxPort_OutputMode_pushPull,
        IfxPort_PadDriver_cmosAutomotiveSpeed1
    };

    ascConfig.pins = &pins;

    IfxAsclin_Asc_initModule(&g_asc, &ascConfig);
}

/* ============================================================
   UART 바이트 배열 송신
   ============================================================ */

static void uartSendBytes(uint8 *data, uint32 len)
{
    for(uint32 i = 0U; i < len; i++)
    {
        IfxAsclin_Asc_flushTx(&g_asc, TIME_INFINITE);
        IfxAsclin_Asc_blockingWrite(&g_asc, data[i]);
    }
}

/* ============================================================
   0~255 -> Throttle 3자리 ASCII HEX 변환
   기존 코드 유지
   ============================================================ */

static void convertThrottle(uint8 speed, char *out)
{
    int value;

    /*
     * 중립       = 0x3B3
     * 최대 전진  = 0x440
     * 최대 후진  = 0x36D
     *
     * 기존 코드 기준:
     * speed < 127 : 전진 방향
     * speed > 127 : 후진 방향
     */
    if(speed < 127U)
    {
        value = 0x3B3 +
                ((127 - speed) * (0x440 - 0x3B3)) / 127;
    }
    else if(speed > 127U)
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

/* ============================================================
   0~255 -> Steering 3자리 ASCII HEX 변환
   기존 코드 유지
   ============================================================ */

static void convertSteering(uint8 steer, char *out)
{
    int value;

    /*
     * 중립        = 0x381
     * 최대 좌회전  = 0x253
     * 최대 우회전  = 0x505
     */
    if(steer < 127U)
    {
        value = 0x381 -
                ((127 - steer) * (0x381 - 0x253)) / 127;
    }
    else if(steer > 127U)
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

/* ============================================================
   차량 UART Frame 생성 및 송신
   ============================================================ */

static void sendCarFrame(uint8 speed, uint8 steer, StopCmd_t stopCmd)
{
    char throttleStr[4];
    char steeringStr[4];

    /*
     * STOP이면 강제로 중립값으로 변환
     */
    if(stopCmd == STOP_CMD_STOP)
    {
        speed = DRIVE_CMD_STOP_VALUE;
        steer = STEERING_CMD_CENTER_VALUE;
    }

    convertThrottle(speed, throttleStr);
    convertSteering(steer, steeringStr);

    /*
     * Frame 구성:
     * frame[0]      = NULL
     * frame[1]      = STX
     * frame[2]~[4]  = throttle ASCII HEX 3자리
     * frame[5]~[7]  = steering ASCII HEX 3자리
     * frame[8]~[9]  = mode/stop field
     * frame[10]     = ETX
     */

    frame[0]  = 0x00;
    frame[1]  = 0x02;

    frame[2]  = throttleStr[0];
    frame[3]  = throttleStr[1];
    frame[4]  = throttleStr[2];

    frame[5]  = steeringStr[0];
    frame[6]  = steeringStr[1];
    frame[7]  = steeringStr[2];

    /*
     * 기존 주행 프레임: 0 1
     * 비상 정지 프레임: 1 1
     */
    if(stopCmd == STOP_CMD_STOP)
    {
        frame[8] = '1';
        frame[9] = '1';
    }
    else
    {
        frame[8] = '0';
        frame[9] = '1';
    }

    frame[10] = 0x03;

    uartSendBytes(frame, 11U);
}

/* ============================================================
   CAN VehicleControlCmd 처리
   ============================================================ */

static void handleVehicleControlCmd(const VehicleControlCmd_t *cmd)
{
    if(cmd == NULL_PTR)
    {
        return;
    }

    /*
     * Motor ECU는 GearState를 판단하지 않는다.
     * ZCU에서 온 0x100 VehicleControlCmd만 기준으로 동작한다.
     *
     * B2 stopCmd가 STOP이면 강제 정지.
     */
    if(cmd->stopCmd == STOP_CMD_STOP)
    {
        sendCarFrame(DRIVE_CMD_STOP_VALUE,
                     STEERING_CMD_CENTER_VALUE,
                     STOP_CMD_STOP);
        return;
    }

    /*
     * 정상 주행 명령.
     */
    sendCarFrame(cmd->driveCmd,
                 cmd->steeringCmd,
                 STOP_CMD_GO);
}

/* ============================================================
   MAIN
   ============================================================ */

int core0_main(void)
{
    VehicleControlCmd_t cmd;

    IfxCpu_enableInterrupts();

    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());

    IfxCpu_emitEvent(&g_cpuSyncEvent);
    IfxCpu_waitEvent(&g_cpuSyncEvent, 1);

    /*
     * CAN 먼저 초기화
     * Motor ECU MCMCAN.c는 0x100 VehicleControlCmd 수신 필터만 사용한다.
     */
    initMcmcan();

    /*
     * 차량 제어보드 UART 송신 초기화
     */
    initUART();

    /*
     * 시작 시 안전을 위해 정지 프레임 1회 송신.
     * stopCmd는 GO로 두되, speed/steer가 중립이므로 차량은 정지 상태.
     */
    sendCarFrame(DRIVE_CMD_STOP_VALUE,
                 STEERING_CMD_CENTER_VALUE,
                 STOP_CMD_GO);

    cmd.driveCmd    = DRIVE_CMD_STOP_VALUE;
    cmd.steeringCmd = STEERING_CMD_CENTER_VALUE;
    cmd.stopCmd     = STOP_CMD_GO;

    while(1)
    {
        /*
         * MCMCAN.c에서 CAN 0x100 수신 시
         * g_motorNewCmdFlag = TRUE로 설정됨.
         *
         * 여기서는 새 0x100 명령이 들어왔을 때만 차량 UART 프레임을 송신.
         */
        if(MotorCan_getLatestControlCmd(&cmd) == TRUE)
        {
            handleVehicleControlCmd(&cmd);
        }
    }

    return 0;
}
