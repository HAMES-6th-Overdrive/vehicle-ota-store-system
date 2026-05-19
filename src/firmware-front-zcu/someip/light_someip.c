#include "light_someip.h"
#include "light_someip_udp.h"

#include <string.h>


/* -------------------- Internal Variabls -------------------- */

/* SOMEIP 서비스 관련 내부 변수 */
static int g_initialized = 0;

static uint16_t g_client_id = 0;
static uint16_t g_port = 0;
static uint16_t g_session_id = 1;

/* 정적 라우팅용 라우팅 정보 배열 */
static LightSomeipServiceRoute g_routes[SOMEIP_MAX_ROUTES];

/* 송수신용 버퍼 */
static uint8_t g_tx_buf[SOMEIP_HEADER_LEN + SOMEIP_MAX_PAYLOAD_LEN];
static uint8_t g_rx_buf[SOMEIP_HEADER_LEN + SOMEIP_MAX_PAYLOAD_LEN];


/* -------------------- Static Function Prototypes -------------------- */

/* 바이트 배열로부터 데이터 읽어오는 헬퍼 함수 */
static uint16_t read_uint16(const uint8_t* ptr);
static uint32_t read_uint32(const uint8_t* ptr);

/* 엔드포인트를 찾는 정적 라우팅 함수 */
static const LightSomeipEndpoint* find_endpoint(uint16_t service_id);
static LightSomeipStatus build_packet(uint8_t* buf_ptr, const LightSomeipPacket* packet_ptr);
static LightSomeipStatus parse_packet(const uint8_t* raw_buf_ptr, uint32_t raw_len, LightSomeipPacket* packet_ptr);
static LightSomeipStatus send_packet(const char* dst_ip, uint16_t dst_port, const LightSomeipPacket* packet_ptr);


/* -------------------- Static Function Implementations -------------------- */

static uint16_t read_uint16(const uint8_t* ptr) {
    return (uint16_t)(((uint16_t)ptr[0]) << 8 | ptr[1]);
}

static uint32_t read_uint32(const uint8_t* ptr) {
    return ((((uint32_t)ptr[0]) << 24) | (((uint32_t)ptr[1]) << 16) | (((uint32_t)ptr[2]) << 8) | (uint32_t)ptr[3]);
}

static const LightSomeipEndpoint* find_endpoint(uint16_t service_id) {
    for(int i = 0; i < SOMEIP_MAX_ROUTES; i++) {
        if(g_routes[i].service_id == 0x0000) continue;
        if(g_routes[i].service_id == service_id) {
            return &(g_routes[i].endpoint);
        }
    }

    return NULL;
}

static LightSomeipStatus build_packet(uint8_t* buf_ptr, const LightSomeipPacket* packet_ptr) {
    uint32_t someip_length = 8u + packet_ptr->payload_len;

    buf_ptr[0] = (uint8_t)((packet_ptr->service_id >> 8) & 0xFF);
    buf_ptr[1] = (uint8_t)(packet_ptr->service_id & 0xFF);

    buf_ptr[2] = (uint8_t)((packet_ptr->method_id >> 8) & 0xFF);
    buf_ptr[3] = (uint8_t)(packet_ptr->method_id & 0xFF);

    buf_ptr[4] = (uint8_t)((someip_length >> 24) & 0xFF);
    buf_ptr[5] = (uint8_t)((someip_length >> 16) & 0xFF);
    buf_ptr[6] = (uint8_t)((someip_length >> 8) & 0xFF);
    buf_ptr[7] = (uint8_t)(someip_length & 0xFF);

    buf_ptr[8] = (uint8_t)((packet_ptr->client_id >> 8) & 0xFF);
    buf_ptr[9] = (uint8_t)(packet_ptr->client_id & 0xFF);

    buf_ptr[10] = (uint8_t)((packet_ptr->session_id >> 8) & 0xFF);
    buf_ptr[11] = (uint8_t)(packet_ptr->session_id & 0xFF);

    buf_ptr[12] = packet_ptr->protocol_version;
    buf_ptr[13] = packet_ptr->interface_version;
    buf_ptr[14] = packet_ptr->message_type;
    buf_ptr[15] = packet_ptr->return_code;

    if (packet_ptr->payload_len > 0) {
        memcpy(&buf_ptr[SOMEIP_HEADER_LEN], packet_ptr->payload_arr, packet_ptr->payload_len);
    }

    return SOMEIP_OK;
}

static LightSomeipStatus parse_packet(const uint8_t* raw_buf_ptr, uint32_t raw_len, LightSomeipPacket* packet_ptr) {
    if(raw_buf_ptr == NULL || packet_ptr == NULL) return SOMEIP_NULL_PTR;
    if(raw_len < SOMEIP_HEADER_LEN) return SOMEIP_MALFORMED_MSG;

    uint32_t someip_length = read_uint32(&raw_buf_ptr[4]);
    if(someip_length < 8u) return SOMEIP_MALFORMED_MSG;

    uint32_t parsed_payload_len = someip_length - 8u;
    if(parsed_payload_len > SOMEIP_MAX_PAYLOAD_LEN) return SOMEIP_TOO_LONG_PAYLOAD;

    if(raw_buf_ptr[12] != SOMEIP_PROTOCOL_VER) return SOMEIP_WRONG_PROTOCOL;
    if(raw_buf_ptr[13] != SOMEIP_INTERFACE_VER) return SOMEIP_WRONG_INTERFACE;


    if(raw_buf_ptr[14] != SOMEIP_MSGTYPE_REQUEST && raw_buf_ptr[14] != SOMEIP_MSGTYPE_RESPONSE && raw_buf_ptr[14] != SOMEIP_MSGTYPE_NOTIFICATION) return SOMEIP_MALFORMED_MSG;

    memset(packet_ptr, 0, sizeof(LightSomeipPacket));

    packet_ptr->service_id = read_uint16(&raw_buf_ptr[0]);
    packet_ptr->method_id = read_uint16(&raw_buf_ptr[2]);
    packet_ptr->client_id = read_uint16(&raw_buf_ptr[8]);
    packet_ptr->session_id = read_uint16(&raw_buf_ptr[10]);
    packet_ptr->protocol_version = raw_buf_ptr[12];
    packet_ptr->interface_version = raw_buf_ptr[13];
    packet_ptr->message_type = raw_buf_ptr[14];
    packet_ptr->return_code = raw_buf_ptr[15];
    packet_ptr->payload_len = parsed_payload_len;

    if(parsed_payload_len > 0) {
        memcpy(packet_ptr->payload_arr, &raw_buf_ptr[SOMEIP_HEADER_LEN], parsed_payload_len);
    }

    return SOMEIP_OK;
}

static LightSomeipStatus send_packet(const char* dst_ip, uint16_t dst_port, const LightSomeipPacket* packet_ptr) {
    if(dst_ip == NULL || packet_ptr == NULL) return SOMEIP_NULL_PTR;
    if(packet_ptr->payload_len > SOMEIP_MAX_PAYLOAD_LEN) return SOMEIP_TOO_LONG_PAYLOAD;
    if(dst_port == 0) return SOMEIP_WRONG_PORT;

    build_packet(g_tx_buf, packet_ptr);

    uint32_t raw_len = SOMEIP_HEADER_LEN + packet_ptr->payload_len;
    
    /* UDP 전송 */
    if(light_someip_udp_send(dst_ip, dst_port, g_tx_buf, raw_len) != 0) return SOMEIP_SEND_ERR;

    return SOMEIP_OK;
}


/* -------------------- Public Function Implementations -------------------- */

LightSomeipStatus light_someip_init(const LightSomeipConfig* config_ptr) {
    if(config_ptr == NULL) return SOMEIP_NULL_PTR;
    if(config_ptr->port == 0) return SOMEIP_WRONG_PORT;

    g_client_id = config_ptr->client_id;
    g_port = config_ptr->port;
    g_session_id = 1;
    memset(g_routes, 0, sizeof(g_routes));

    for(int i = 0; i < SOMEIP_MAX_ROUTES; i++) {
        g_routes[i] = config_ptr->routes[i];
    }

    if(light_someip_udp_init(g_port) != 0) return SOMEIP_UDP_INIT_ERR;

    g_initialized = 1;

    return SOMEIP_OK;
}

LightSomeipStatus light_someip_packet_init(LightSomeipPacket* packet_ptr, uint16_t service_id, uint16_t method_id, const uint8_t* payload_arr, uint32_t payload_len) {
    if(packet_ptr == NULL) return SOMEIP_NULL_PTR;
    if(payload_len > SOMEIP_MAX_PAYLOAD_LEN) return SOMEIP_TOO_LONG_PAYLOAD;
    if(payload_len > 0 && payload_arr == NULL) return SOMEIP_NULL_PTR;

    memset(packet_ptr, 0, sizeof(LightSomeipPacket));

    packet_ptr->service_id = service_id;
    packet_ptr->method_id = method_id;
    packet_ptr->protocol_version = SOMEIP_PROTOCOL_VER;
    packet_ptr->interface_version = SOMEIP_INTERFACE_VER;
    packet_ptr->return_code = SOMEIP_RET_OK;
    packet_ptr->payload_len = payload_len;

    if(payload_len > 0) {
        memcpy(packet_ptr->payload_arr, payload_arr, payload_len);
    }

    return SOMEIP_OK;
}

LightSomeipStatus light_someip_request(LightSomeipPacket* packet_ptr) {
    if(!g_initialized) return SOMEIP_NOT_INITIALIZED;
    if(packet_ptr == NULL) return SOMEIP_NULL_PTR;

    const LightSomeipEndpoint* dst_endpoint_ptr = find_endpoint(packet_ptr->service_id);
    if(dst_endpoint_ptr == NULL) return SOMEIP_UNKNOWN_SERVICE;

    packet_ptr->client_id = g_client_id;
    packet_ptr->session_id = g_session_id;
    packet_ptr->protocol_version = SOMEIP_PROTOCOL_VER;
    packet_ptr->interface_version = SOMEIP_INTERFACE_VER;
    packet_ptr->message_type = SOMEIP_MSGTYPE_REQUEST;
    packet_ptr->return_code = SOMEIP_RET_OK;

    LightSomeipStatus ret = send_packet(dst_endpoint_ptr->ip, dst_endpoint_ptr->port, packet_ptr);
    if(ret == SOMEIP_OK) g_session_id++;

    return ret;
}

LightSomeipStatus light_someip_respond(const char* remote_ip, uint16_t remote_port, const LightSomeipPacket* request_packet_ptr, LightSomeipPacket* response_packet_ptr) {
    if(!g_initialized) return SOMEIP_NOT_INITIALIZED;
    if(remote_ip == NULL || request_packet_ptr == NULL || response_packet_ptr == NULL) return SOMEIP_NULL_PTR;
    if(remote_port == 0) return SOMEIP_WRONG_PORT;
    if(request_packet_ptr->message_type != SOMEIP_MSGTYPE_REQUEST) return SOMEIP_NOT_REQUEST;

    response_packet_ptr->service_id = request_packet_ptr->service_id;
    response_packet_ptr->method_id = request_packet_ptr->method_id;

    response_packet_ptr->client_id = request_packet_ptr->client_id;
    response_packet_ptr->session_id = request_packet_ptr->session_id;

    response_packet_ptr->protocol_version = SOMEIP_PROTOCOL_VER;
    response_packet_ptr->interface_version = SOMEIP_INTERFACE_VER;
    response_packet_ptr->message_type = SOMEIP_MSGTYPE_RESPONSE;
    response_packet_ptr->return_code = SOMEIP_RET_OK;

    return send_packet(remote_ip,remote_port, response_packet_ptr);
}
 
LightSomeipStatus light_someip_event_notify(const LightSomeipEndpoint* dst_endpoint_ptr, LightSomeipPacket* packet_ptr) {
    if(!g_initialized) return SOMEIP_NOT_INITIALIZED;
    if(dst_endpoint_ptr == NULL || packet_ptr == NULL) return SOMEIP_NULL_PTR;

    packet_ptr->client_id = g_client_id;
    packet_ptr->session_id = g_session_id;

    packet_ptr->protocol_version = SOMEIP_PROTOCOL_VER;
    packet_ptr->interface_version = SOMEIP_INTERFACE_VER;
    packet_ptr->message_type = SOMEIP_MSGTYPE_NOTIFICATION;
    packet_ptr->return_code = SOMEIP_RET_OK;

    LightSomeipStatus ret = send_packet(dst_endpoint_ptr->ip, dst_endpoint_ptr->port, packet_ptr);

    if(ret == SOMEIP_OK) g_session_id++;

    return ret;
}

LightSomeipStatus light_someip_recv(LightSomeipPacket* packet_ptr, char remote_ip[SOMEIP_IP_LEN], uint16_t* remote_port) {
    if(!g_initialized) return SOMEIP_NOT_INITIALIZED;
    if(packet_ptr == NULL || remote_ip == NULL || remote_port == NULL) return SOMEIP_NULL_PTR;

    int recv_len = light_someip_udp_recv(g_rx_buf, sizeof(g_rx_buf), remote_ip, remote_port);
    if(recv_len == 0) return SOMEIP_NO_MSG;
    if(recv_len < 0) return SOMEIP_RECV_ERR;

    return parse_packet(g_rx_buf, (uint32_t)recv_len, packet_ptr);
}

