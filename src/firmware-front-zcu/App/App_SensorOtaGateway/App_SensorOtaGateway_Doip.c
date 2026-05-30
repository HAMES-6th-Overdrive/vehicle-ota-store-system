/**********************************************************************************************************************
 * \file App_SensorOtaGateway_Doip.c
 * \brief Sensor ECU OTA Gateway DoIP server
 *
 * 역할:
 *  - Pi/HPC가 TCP 13400으로 보내는 DoIP 메시지를 수신한다.
 *  - Routing Activation을 처리한다.
 *  - Diagnostic Message 안의 UDS payload를 App_SensorOtaGateway_Uds로 전달한다.
 *
 * 구조:
 *  Pi/HPC
 *      ↓ TCP / DoIP
 *  App_SensorOtaGateway_Doip
 *      ↓ UDS payload
 *  App_SensorOtaGateway_Uds
 *      ↓
 *  App_OtaReceiver
 *      ↓
 *  App_OtaGateway
 *      ↓ CAN FD 0x600 / 0x601
 *  Sensor ECU
 *
 * 주의:
 *  - 이 모듈은 ZCU Local OTA용 DoIP가 아니다.
 *  - Sensor ECU OTA 중계용 DoIP 입력부이다.
 *********************************************************************************************************************/

#include "App_SensorOtaGateway_Doip.h"
#include "App_SensorOtaGateway_Uds.h"

#include <string.h>

#include "lwip/opt.h"
#include "lwip/tcp.h"
#include "lwip/mem.h"

/* ============================================================
   Private variables
   ============================================================ */

static tcpPcb2 *g_sensorOtaDoipPcb = NULL;

typedef struct
{
    boolean active;
    tcpPcb2 *pcb;
    AppSensorOtaGatewayDoip_Session_t *session;
    uint16 testerAddr;
    uint8 requestSid;
} AppSensorOtaGatewayDoip_PendingUdsResponse_t;

typedef struct
{
    boolean active;
    tcpPcb2 *pcb;
    AppSensorOtaGatewayDoip_Session_t *session;
    uint16 testerAddr;
    uint8 requestSid;
    uint8 nrc;
} AppSensorOtaGatewayDoip_DeferredNegativeResponse_t;

static AppSensorOtaGatewayDoip_PendingUdsResponse_t g_sensorOtaDoipPendingUdsResponse;
static AppSensorOtaGatewayDoip_DeferredNegativeResponse_t g_sensorOtaDoipDeferredNegativeResponse;

/* ============================================================
   Debug variables
   ============================================================ */

volatile uint32 g_sensorOtaDoipInitCount = 0U;
volatile uint32 g_sensorOtaDoipBindOkCount = 0U;
volatile uint32 g_sensorOtaDoipBindFailCount = 0U;
volatile uint32 g_sensorOtaDoipAcceptCount = 0U;
volatile uint32 g_sensorOtaDoipRecvCount = 0U;
volatile uint32 g_sensorOtaDoipCloseCount = 0U;
volatile uint32 g_sensorOtaDoipErrorCount = 0U;

volatile uint32 g_sensorOtaDoipRoutingActReqCount = 0U;
volatile uint32 g_sensorOtaDoipRoutingActResCount = 0U;
volatile uint32 g_sensorOtaDoipDiagMsgCount = 0U;
volatile uint32 g_sensorOtaDoipDiagAckCount = 0U;
volatile uint32 g_sensorOtaDoipDiagResCount = 0U;

volatile uint32 g_sensorOtaDoipLastPayloadType = 0U;
volatile uint32 g_sensorOtaDoipLastPayloadLen = 0U;
volatile uint32 g_sensorOtaDoipLastSrcAddr = 0U;
volatile uint32 g_sensorOtaDoipLastTargetAddr = 0U;
volatile uint32 g_sensorOtaDoipLastUdsSid = 0U;
volatile uint32 g_sensorOtaDoipLastUdsReqLen = 0U;
volatile uint32 g_sensorOtaDoipLastUdsResLen = 0U;

/* ============================================================
   Private prototypes
   ============================================================ */

static err_t AppSensorOtaGatewayDoip_Accept(void *arg,
                                            tcpPcb2 *newPcb,
                                            err_t err);

static err_t AppSensorOtaGatewayDoip_Recv(void *arg,
                                          tcpPcb2 *tpcb,
                                          pBuf2 *p,
                                          err_t err);

static void AppSensorOtaGatewayDoip_Error(void *arg,
                                          err_t err);

static err_t AppSensorOtaGatewayDoip_Poll(void *arg,
                                          tcpPcb2 *tpcb);

static void AppSensorOtaGatewayDoip_Close(tcpPcb2 *tpcb,
                                          AppSensorOtaGatewayDoip_Session_t *ds);

static void AppSensorOtaGatewayDoip_Process(tcpPcb2 *tpcb,
                                            AppSensorOtaGatewayDoip_Session_t *ds);

static void AppSensorOtaGatewayDoip_HandleRoutingActivation(tcpPcb2 *tpcb,
                                                            AppSensorOtaGatewayDoip_Session_t *ds,
                                                            uint8 *payload,
                                                            uint32 payloadLen);

static void AppSensorOtaGatewayDoip_HandleDiagMessage(tcpPcb2 *tpcb,
                                                      AppSensorOtaGatewayDoip_Session_t *ds,
                                                      uint8 *payload,
                                                      uint32 payloadLen);

static boolean AppSensorOtaGatewayDoip_SendDiagResponse(tcpPcb2 *tpcb,
                                                        uint16 testerAddr,
                                                        const uint8 *udsRes,
                                                        uint16 udsResLen);

static uint16 AppSensorOtaGatewayDoip_BuildNegativeResponse(uint8 *udsRes,
                                                            uint8 requestSid,
                                                            uint8 nrc);

static void AppSensorOtaGatewayDoip_SetDeferredNegativeResponse(tcpPcb2 *tpcb,
                                                                AppSensorOtaGatewayDoip_Session_t *ds,
                                                                uint16 testerAddr,
                                                                uint8 requestSid,
                                                                uint8 nrc);

static void AppSensorOtaGatewayDoip_PollDeferredNegativeResponse(void);
static void AppSensorOtaGatewayDoip_PollUdsResponse(void);
static void AppSensorOtaGatewayDoip_ForgetPendingSession(tcpPcb2 *tpcb,
                                                         AppSensorOtaGatewayDoip_Session_t *ds);

static err_t AppSensorOtaGatewayDoip_SendRaw(tcpPcb2 *tpcb,
                                            uint8 *buf,
                                            uint16 len);

static void AppSensorOtaGatewayDoip_BuildHeader(uint8 *buf,
                                                uint16 payloadType,
                                                uint32 payloadLen);

/* ============================================================
   Public functions
   ============================================================ */

void AppSensorOtaGatewayDoip_Init(void)
{
    err_t err;

    g_sensorOtaDoipInitCount++;
    memset((void *)&g_sensorOtaDoipPendingUdsResponse,
           0,
           sizeof(g_sensorOtaDoipPendingUdsResponse));
    memset((void *)&g_sensorOtaDoipDeferredNegativeResponse,
           0,
           sizeof(g_sensorOtaDoipDeferredNegativeResponse));

    /*
     * UDS adapter도 함께 초기화한다.
     */
    AppSensorOtaGatewayUds_Init();

    g_sensorOtaDoipPcb = tcp_new();

    if(g_sensorOtaDoipPcb != NULL)
    {
        err = tcp_bind(g_sensorOtaDoipPcb,
                       IP_ADDR_ANY,
                       APP_SENSOR_OTA_GATEWAY_DOIP_PORT);

        if(err == ERR_OK)
        {
            g_sensorOtaDoipPcb = tcp_listen(g_sensorOtaDoipPcb);
            tcp_accept(g_sensorOtaDoipPcb, AppSensorOtaGatewayDoip_Accept);

            g_sensorOtaDoipBindOkCount++;
        }
        else
        {
            g_sensorOtaDoipBindFailCount++;
        }
    }
    else
    {
        g_sensorOtaDoipBindFailCount++;
    }
}


void AppSensorOtaGatewayDoip_MainFunction(void)
{
    AppSensorOtaGatewayDoip_PollUdsResponse();

    if(g_sensorOtaDoipPendingUdsResponse.active == TRUE)
    {
        return;
    }

    AppSensorOtaGatewayDoip_PollDeferredNegativeResponse();
}

/* ============================================================
   TCP callbacks
   ============================================================ */

static err_t AppSensorOtaGatewayDoip_Accept(void *arg,
                                            tcpPcb2 *newPcb,
                                            err_t err)
{
    AppSensorOtaGatewayDoip_Session_t *ds;

    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(err);

    ds = (AppSensorOtaGatewayDoip_Session_t *)mem_malloc(sizeof(AppSensorOtaGatewayDoip_Session_t));

    if(ds == NULL)
    {
        return ERR_MEM;
    }

    ds->state = APP_SENSOR_OTA_GATEWAY_DOIP_STATE_ACCEPTED;
    ds->pcb = newPcb;
    ds->p = NULL;
    ds->rxLen = 0U;
    memset(ds->rxBuf, 0, sizeof(ds->rxBuf));

    tcp_arg(newPcb, ds);
    tcp_nagle_disable(newPcb);
    tcp_recv(newPcb, AppSensorOtaGatewayDoip_Recv);
    tcp_err(newPcb, AppSensorOtaGatewayDoip_Error);
    tcp_poll(newPcb, AppSensorOtaGatewayDoip_Poll, 2);

    g_sensorOtaDoipAcceptCount++;

    return ERR_OK;
}


static err_t AppSensorOtaGatewayDoip_Recv(void *arg,
                                          tcpPcb2 *tpcb,
                                          pBuf2 *p,
                                          err_t err)
{
    AppSensorOtaGatewayDoip_Session_t *ds;
    uint16 copyLen;

    ds = (AppSensorOtaGatewayDoip_Session_t *)arg;

    if(ds == NULL)
    {
        if(p != NULL)
        {
            pbuf_free(p);
        }

        return ERR_ARG;
    }

    /*
     * 연결 종료.
     */
    if(p == NULL)
    {
        ds->state = APP_SENSOR_OTA_GATEWAY_DOIP_STATE_CLOSING;
        AppSensorOtaGatewayDoip_Close(tpcb, ds);
        return ERR_OK;
    }

    if(err != ERR_OK)
    {
        pbuf_free(p);
        return err;
    }

    copyLen = p->tot_len;

    if(copyLen > APP_SENSOR_OTA_GATEWAY_DOIP_RX_BUF_SIZE)
    {
        copyLen = APP_SENSOR_OTA_GATEWAY_DOIP_RX_BUF_SIZE;
    }

    memset(ds->rxBuf, 0, sizeof(ds->rxBuf));
    pbuf_copy_partial(p, ds->rxBuf, copyLen, 0);
    ds->rxLen = copyLen;

    tcp_recved(tpcb, p->tot_len);
    pbuf_free(p);

    g_sensorOtaDoipRecvCount++;

    AppSensorOtaGatewayDoip_Process(tpcb, ds);

    return ERR_OK;
}


static void AppSensorOtaGatewayDoip_Error(void *arg,
                                          err_t err)
{
    AppSensorOtaGatewayDoip_Session_t *ds;

    LWIP_UNUSED_ARG(err);

    ds = (AppSensorOtaGatewayDoip_Session_t *)arg;

    if(ds != NULL)
    {
        AppSensorOtaGatewayDoip_ForgetPendingSession(ds->pcb, ds);
        mem_free(ds);
    }

    g_sensorOtaDoipErrorCount++;
}


static err_t AppSensorOtaGatewayDoip_Poll(void *arg,
                                          tcpPcb2 *tpcb)
{
    AppSensorOtaGatewayDoip_Session_t *ds;

    ds = (AppSensorOtaGatewayDoip_Session_t *)arg;

    if(ds == NULL)
    {
        tcp_abort(tpcb);
        return ERR_ABRT;
    }

    if(ds->state == APP_SENSOR_OTA_GATEWAY_DOIP_STATE_CLOSING)
    {
        AppSensorOtaGatewayDoip_Close(tpcb, ds);
    }

    return ERR_OK;
}


static void AppSensorOtaGatewayDoip_Close(tcpPcb2 *tpcb,
                                          AppSensorOtaGatewayDoip_Session_t *ds)
{
    AppSensorOtaGatewayDoip_ForgetPendingSession(tpcb, ds);

    tcp_arg(tpcb, NULL);
    tcp_recv(tpcb, NULL);
    tcp_err(tpcb, NULL);
    tcp_poll(tpcb, NULL, 0);

    if(ds != NULL)
    {
        mem_free(ds);
    }

    (void)tcp_close(tpcb);

    g_sensorOtaDoipCloseCount++;
}

/* ============================================================
   DoIP processing
   ============================================================ */

static void AppSensorOtaGatewayDoip_Process(tcpPcb2 *tpcb,
                                            AppSensorOtaGatewayDoip_Session_t *ds)
{
    uint8 *buf;
    uint16 len;
    uint16 payloadType;
    uint32 payloadLen;
    uint8 *payload;

    buf = ds->rxBuf;
    len = ds->rxLen;

    if(len < APP_SENSOR_OTA_GATEWAY_DOIP_HEADER_LEN)
    {
        return;
    }

    /*
     * DoIP header:
     *   [0] protocol version
     *   [1] inverse protocol version
     *   [2..3] payload type
     *   [4..7] payload length
     */
    if((buf[0] != APP_SENSOR_OTA_GATEWAY_DOIP_PROTOCOL_VERSION) ||
       (buf[1] != (uint8)(~APP_SENSOR_OTA_GATEWAY_DOIP_PROTOCOL_VERSION)))
    {
        return;
    }

    payloadType = ((uint16)buf[2] << 8) | buf[3];
    payloadLen  = ((uint32)buf[4] << 24) |
                  ((uint32)buf[5] << 16) |
                  ((uint32)buf[6] << 8)  |
                  ((uint32)buf[7]);

    g_sensorOtaDoipLastPayloadType = payloadType;
    g_sensorOtaDoipLastPayloadLen = payloadLen;

    if(payloadLen > (uint32)(len - APP_SENSOR_OTA_GATEWAY_DOIP_HEADER_LEN))
    {
        /*
         * 현재 구현은 단일 TCP receive 안에 DoIP message 하나가 들어오는 단순 구조이다.
         * payload가 아직 다 들어오지 않았다면 이번에는 처리하지 않는다.
         *
         * 추후 대용량/분할 수신 안정화를 위해 rx accumulation 구조로 확장 가능.
         */
        return;
    }

    payload = &buf[APP_SENSOR_OTA_GATEWAY_DOIP_HEADER_LEN];

    switch(payloadType)
    {
        case APP_SENSOR_OTA_GATEWAY_DOIP_ROUTING_ACT_REQ:
        {
            AppSensorOtaGatewayDoip_HandleRoutingActivation(tpcb,
                                                            ds,
                                                            payload,
                                                            payloadLen);
            break;
        }

        case APP_SENSOR_OTA_GATEWAY_DOIP_DIAG_MESSAGE:
        {
            AppSensorOtaGatewayDoip_HandleDiagMessage(tpcb,
                                                      ds,
                                                      payload,
                                                      payloadLen);
            break;
        }

        default:
        {
            break;
        }
    }
}


static void AppSensorOtaGatewayDoip_HandleRoutingActivation(tcpPcb2 *tpcb,
                                                            AppSensorOtaGatewayDoip_Session_t *ds,
                                                            uint8 *payload,
                                                            uint32 payloadLen)
{
    uint16 testerAddr;
    uint8 res[21];

    if(payloadLen < 2U)
    {
        return;
    }

    g_sensorOtaDoipRoutingActReqCount++;

    testerAddr = ((uint16)payload[0] << 8) | payload[1];

    AppSensorOtaGatewayDoip_BuildHeader(res,
                                        APP_SENSOR_OTA_GATEWAY_DOIP_ROUTING_ACT_RES,
                                        13U);

    /*
     * Routing Activation Response payload:
     *   [0..1] tester logical address
     *   [2..3] ZCU logical address
     *   [4]    response code 0x10 = routing activation successful
     *   [5..12] reserved
     */
    res[8]  = (uint8)((testerAddr >> 8) & 0xFFU);
    res[9]  = (uint8)(testerAddr & 0xFFU);
    res[10] = (uint8)((APP_SENSOR_OTA_GATEWAY_DOIP_ZCU_ADDR >> 8) & 0xFFU);
    res[11] = (uint8)(APP_SENSOR_OTA_GATEWAY_DOIP_ZCU_ADDR & 0xFFU);
    res[12] = 0x10U;

    res[13] = 0x00U;
    res[14] = 0x00U;
    res[15] = 0x00U;
    res[16] = 0x00U;
    res[17] = 0x00U;
    res[18] = 0x00U;
    res[19] = 0x00U;
    res[20] = 0x00U;

    AppSensorOtaGatewayDoip_SendRaw(tpcb, res, sizeof(res));

    ds->state = APP_SENSOR_OTA_GATEWAY_DOIP_STATE_ROUTING_ACTIVE;

    g_sensorOtaDoipRoutingActResCount++;
}


static void AppSensorOtaGatewayDoip_HandleDiagMessage(tcpPcb2 *tpcb,
                                                      AppSensorOtaGatewayDoip_Session_t *ds,
                                                      uint8 *payload,
                                                      uint32 payloadLen)
{
    uint16 srcAddr;
    uint16 targetAddr;
    uint8 ack[13];

    uint8 *udsData;
    uint16 udsLen;
    uint8 requestSid;

    if(ds->state != APP_SENSOR_OTA_GATEWAY_DOIP_STATE_ROUTING_ACTIVE)
    {
        return;
    }

    if(payloadLen < 4U)
    {
        return;
    }

    /*
     * Diagnostic Message payload:
     *   [0..1] source address
     *   [2..3] target address
     *   [4..]  UDS payload
     */
    srcAddr = ((uint16)payload[0] << 8) | payload[1];
    targetAddr = ((uint16)payload[2] << 8) | payload[3];

    g_sensorOtaDoipLastSrcAddr = srcAddr;
    g_sensorOtaDoipLastTargetAddr = targetAddr;

    if(targetAddr != APP_SENSOR_OTA_GATEWAY_DOIP_ZCU_ADDR)
    {
        return;
    }

    g_sensorOtaDoipDiagMsgCount++;

    /*
     * Diagnostic Message ACK:
     *   DoIP payload type 0x8002
     *   source = ZCU
     *   target = Tester
     *   ACK code = 0x00
     */
    AppSensorOtaGatewayDoip_BuildHeader(ack,
                                        APP_SENSOR_OTA_GATEWAY_DOIP_DIAG_MESSAGE_ACK,
                                        5U);

    ack[8]  = (uint8)((APP_SENSOR_OTA_GATEWAY_DOIP_ZCU_ADDR >> 8) & 0xFFU);
    ack[9]  = (uint8)(APP_SENSOR_OTA_GATEWAY_DOIP_ZCU_ADDR & 0xFFU);
    ack[10] = (uint8)((srcAddr >> 8) & 0xFFU);
    ack[11] = (uint8)(srcAddr & 0xFFU);
    ack[12] = 0x00U;

    AppSensorOtaGatewayDoip_SendRaw(tpcb, ack, sizeof(ack));

    g_sensorOtaDoipDiagAckCount++;

    udsData = &payload[4];
    udsLen = (uint16)(payloadLen - 4U);
    requestSid = (udsLen > 0U) ? udsData[0] : 0U;

    if(udsLen > 0U)
    {
        g_sensorOtaDoipLastUdsSid = udsData[0];
    }
    else
    {
        g_sensorOtaDoipLastUdsSid = 0U;
    }

    g_sensorOtaDoipLastUdsReqLen = udsLen;

    /*
     * UDS payload 처리.
     *
     * 기존 팀원 코드:
     *   UDS_HandleService()
     *
     * Sensor ECU Gateway 구조:
     */
    g_sensorOtaDoipLastUdsResLen = 0U;

    if(g_sensorOtaDoipPendingUdsResponse.active == TRUE)
    {
        AppSensorOtaGatewayDoip_SetDeferredNegativeResponse(
            tpcb,
            ds,
            srcAddr,
            requestSid,
            APP_SENSOR_OTA_GATEWAY_UDS_NRC_GENERAL_REJECT);
        return;
    }

    if(AppSensorOtaGatewayUds_TryStartRequest(udsData, udsLen) == TRUE)
    {
        g_sensorOtaDoipPendingUdsResponse.active = TRUE;
        g_sensorOtaDoipPendingUdsResponse.pcb = tpcb;
        g_sensorOtaDoipPendingUdsResponse.session = ds;
        g_sensorOtaDoipPendingUdsResponse.testerAddr = srcAddr;
        g_sensorOtaDoipPendingUdsResponse.requestSid = requestSid;
        return;
    }

    AppSensorOtaGatewayDoip_SetDeferredNegativeResponse(
        tpcb,
        ds,
        srcAddr,
        requestSid,
        APP_SENSOR_OTA_GATEWAY_UDS_NRC_GENERAL_REJECT);

}


static boolean AppSensorOtaGatewayDoip_SendDiagResponse(tcpPcb2 *tpcb,
                                                        uint16 testerAddr,
                                                        const uint8 *udsRes,
                                                        uint16 udsResLen)
{
    uint32 diagLen;
    uint8 res[APP_SENSOR_OTA_GATEWAY_DOIP_HEADER_LEN +
              4U +
              APP_SENSOR_OTA_GATEWAY_UDS_TX_BUF_SIZE];

    if((tpcb == NULL) ||
       (udsRes == NULL_PTR) ||
       (udsResLen == 0U) ||
       (udsResLen > APP_SENSOR_OTA_GATEWAY_UDS_TX_BUF_SIZE))
    {
        return FALSE;
    }

    diagLen = 4U + udsResLen;

    AppSensorOtaGatewayDoip_BuildHeader(res,
                                        APP_SENSOR_OTA_GATEWAY_DOIP_DIAG_MESSAGE,
                                        diagLen);

    res[8]  = (uint8)((APP_SENSOR_OTA_GATEWAY_DOIP_ZCU_ADDR >> 8) & 0xFFU);
    res[9]  = (uint8)(APP_SENSOR_OTA_GATEWAY_DOIP_ZCU_ADDR & 0xFFU);
    res[10] = (uint8)((testerAddr >> 8) & 0xFFU);
    res[11] = (uint8)(testerAddr & 0xFFU);

    memcpy(&res[12], udsRes, udsResLen);

    return (AppSensorOtaGatewayDoip_SendRaw(
                tpcb,
                res,
                (uint16)(APP_SENSOR_OTA_GATEWAY_DOIP_HEADER_LEN + diagLen)) == ERR_OK) ? TRUE : FALSE;
}


static uint16 AppSensorOtaGatewayDoip_BuildNegativeResponse(uint8 *udsRes,
                                                            uint8 requestSid,
                                                            uint8 nrc)
{
    udsRes[0] = APP_SENSOR_OTA_GATEWAY_UDS_NEGATIVE_RSP;
    udsRes[1] = requestSid;
    udsRes[2] = nrc;

    return 3U;
}


static void AppSensorOtaGatewayDoip_SetDeferredNegativeResponse(tcpPcb2 *tpcb,
                                                                AppSensorOtaGatewayDoip_Session_t *ds,
                                                                uint16 testerAddr,
                                                                uint8 requestSid,
                                                                uint8 nrc)
{
    if(g_sensorOtaDoipDeferredNegativeResponse.active == TRUE)
    {
        return;
    }

    g_sensorOtaDoipDeferredNegativeResponse.active = TRUE;
    g_sensorOtaDoipDeferredNegativeResponse.pcb = tpcb;
    g_sensorOtaDoipDeferredNegativeResponse.session = ds;
    g_sensorOtaDoipDeferredNegativeResponse.testerAddr = testerAddr;
    g_sensorOtaDoipDeferredNegativeResponse.requestSid = requestSid;
    g_sensorOtaDoipDeferredNegativeResponse.nrc = nrc;
}


static void AppSensorOtaGatewayDoip_PollDeferredNegativeResponse(void)
{
    uint8 udsRes[3];
    uint16 udsResLen;

    if(g_sensorOtaDoipDeferredNegativeResponse.active != TRUE)
    {
        return;
    }

    if((g_sensorOtaDoipDeferredNegativeResponse.pcb == NULL) ||
       (g_sensorOtaDoipDeferredNegativeResponse.session == NULL) ||
       (g_sensorOtaDoipDeferredNegativeResponse.session->state !=
        APP_SENSOR_OTA_GATEWAY_DOIP_STATE_ROUTING_ACTIVE))
    {
        memset((void *)&g_sensorOtaDoipDeferredNegativeResponse,
               0,
               sizeof(g_sensorOtaDoipDeferredNegativeResponse));
        return;
    }

    udsResLen = AppSensorOtaGatewayDoip_BuildNegativeResponse(
                    udsRes,
                    g_sensorOtaDoipDeferredNegativeResponse.requestSid,
                    g_sensorOtaDoipDeferredNegativeResponse.nrc);

    if(AppSensorOtaGatewayDoip_SendDiagResponse(
           g_sensorOtaDoipDeferredNegativeResponse.pcb,
           g_sensorOtaDoipDeferredNegativeResponse.testerAddr,
           udsRes,
           udsResLen) == TRUE)
    {
        memset((void *)&g_sensorOtaDoipDeferredNegativeResponse,
               0,
               sizeof(g_sensorOtaDoipDeferredNegativeResponse));
    }
}


static void AppSensorOtaGatewayDoip_PollUdsResponse(void)
{
    uint8 udsRes[APP_SENSOR_OTA_GATEWAY_UDS_TX_BUF_SIZE];
    uint16 udsResLen = 0U;
    AppSensorOtaGatewayUds_ResponseStatus_t status;
    boolean responseConsumed = FALSE;

    if(g_sensorOtaDoipPendingUdsResponse.active != TRUE)
    {
        return;
    }

    status = AppSensorOtaGatewayUds_TryReadResponse(udsRes, &udsResLen);
    if(status == APP_SENSOR_OTA_GATEWAY_UDS_RESPONSE_NOT_READY)
    {
        return;
    }

    if(status == APP_SENSOR_OTA_GATEWAY_UDS_RESPONSE_ERROR)
    {
        udsResLen = AppSensorOtaGatewayDoip_BuildNegativeResponse(
                        udsRes,
                        g_sensorOtaDoipPendingUdsResponse.requestSid,
                        APP_SENSOR_OTA_GATEWAY_UDS_NRC_GENERAL_PROG_FAILURE);
    }

    g_sensorOtaDoipLastUdsResLen = udsResLen;

    if((g_sensorOtaDoipPendingUdsResponse.pcb == NULL) ||
       (g_sensorOtaDoipPendingUdsResponse.session == NULL) ||
       (g_sensorOtaDoipPendingUdsResponse.session->state !=
        APP_SENSOR_OTA_GATEWAY_DOIP_STATE_ROUTING_ACTIVE) ||
       (udsResLen == 0U))
    {
        responseConsumed = TRUE;
    }
    else
    {
        responseConsumed = AppSensorOtaGatewayDoip_SendDiagResponse(
                               g_sensorOtaDoipPendingUdsResponse.pcb,
                               g_sensorOtaDoipPendingUdsResponse.testerAddr,
                               udsRes,
                               udsResLen);
    }

    if(responseConsumed == TRUE)
    {
        AppSensorOtaGatewayUds_ReleaseResponse();
        memset((void *)&g_sensorOtaDoipPendingUdsResponse,
               0,
               sizeof(g_sensorOtaDoipPendingUdsResponse));
        g_sensorOtaDoipDiagResCount++;
    }
}


static void AppSensorOtaGatewayDoip_ForgetPendingSession(tcpPcb2 *tpcb,
                                                         AppSensorOtaGatewayDoip_Session_t *ds)
{
    if((g_sensorOtaDoipPendingUdsResponse.active == TRUE) &&
       ((g_sensorOtaDoipPendingUdsResponse.pcb == tpcb) ||
        (g_sensorOtaDoipPendingUdsResponse.session == ds)))
    {
        g_sensorOtaDoipPendingUdsResponse.pcb = NULL;
        g_sensorOtaDoipPendingUdsResponse.session = NULL;
    }

    if((g_sensorOtaDoipDeferredNegativeResponse.active == TRUE) &&
       ((g_sensorOtaDoipDeferredNegativeResponse.pcb == tpcb) ||
        (g_sensorOtaDoipDeferredNegativeResponse.session == ds)))
    {
        memset((void *)&g_sensorOtaDoipDeferredNegativeResponse,
               0,
               sizeof(g_sensorOtaDoipDeferredNegativeResponse));
    }
}


static void AppSensorOtaGatewayDoip_BuildHeader(uint8 *buf,
                                                uint16 payloadType,
                                                uint32 payloadLen)
{
    buf[0] = APP_SENSOR_OTA_GATEWAY_DOIP_PROTOCOL_VERSION;
    buf[1] = (uint8)(~APP_SENSOR_OTA_GATEWAY_DOIP_PROTOCOL_VERSION);

    buf[2] = (uint8)((payloadType >> 8) & 0xFFU);
    buf[3] = (uint8)(payloadType & 0xFFU);

    buf[4] = (uint8)((payloadLen >> 24) & 0xFFU);
    buf[5] = (uint8)((payloadLen >> 16) & 0xFFU);
    buf[6] = (uint8)((payloadLen >> 8) & 0xFFU);
    buf[7] = (uint8)(payloadLen & 0xFFU);
}


static err_t AppSensorOtaGatewayDoip_SendRaw(tcpPcb2 *tpcb,
                                            uint8 *buf,
                                            uint16 len)
{
    err_t err;

    err = tcp_write(tpcb,
                    buf,
                    len,
                    TCP_WRITE_FLAG_COPY);

    if(err == ERR_OK)
    {
        (void)tcp_output(tpcb);
    }

    return err;
}
