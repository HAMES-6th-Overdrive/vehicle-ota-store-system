#ifndef __LIGHT_SOMEIP_H__
#define __LIGHT_SOMEIP_H__


#include <stdint.h>

#define SOMEIP_PROTOCOL_VER     (0x01u)
#define SOMEIP_INTERFACE_VER    (0x01u)

#define SOMEIP_HEADER_LEN       (16u)
#define SOMEIP_MAX_PAYLOAD_LEN  (128u)
#define SOMEIP_MAX_ROUTES       (10u)
#define SOMEIP_IP_LEN           (16u)


/* -------------------- Structures & Enums -------------------- */
typedef enum LightSomeipStatus {
    SOMEIP_OK = 0,
    SOMEIP_ERR,
    SOMEIP_UNKNOWN_SERVICE,
    SOMEIP_TOO_LONG_PAYLOAD,
    SOMEIP_NULL_PTR,
    SOMEIP_NOT_INITIALIZED,
    SOMEIP_MALFORMED_MSG,
    //SOMEIP_INVALID_PARAM,
    SOMEIP_WRONG_PROTOCOL,
    SOMEIP_WRONG_INTERFACE,
    SOMEIP_WRONG_PORT,
    SOMEIP_SEND_ERR,
    SOMEIP_UDP_INIT_ERR,
    SOMEIP_NOT_REQUEST,
    SOMEIP_NOT_RESPONSE,
    SOMEIP_NO_MSG,
    SOMEIP_RECV_ERR
} LightSomeipStatus;

typedef enum LightSomeipMsgType {
    SOMEIP_MSGTYPE_REQUEST = 0x00,
    SOMEIP_MSGTYPE_NOTIFICATION = 0x02,
    SOMEIP_MSGTYPE_RESPONSE = 0x80
} LightSomeipMsgType;

typedef enum LightSomeipRetCode {
    SOMEIP_RET_OK = 0x00
} LightSomeipRetCode;

typedef struct LightSomeipEndpoint {
    char ip[SOMEIP_IP_LEN];
    uint16_t port;
} LightSomeipEndpoint;

typedef struct LightSomeipServiceRoute {
    uint16_t service_id;
    LightSomeipEndpoint endpoint;
} LightSomeipServiceRoute;

typedef struct LightSomeipConfig {
    uint16_t client_id;
    uint16_t port;
    LightSomeipServiceRoute routes[SOMEIP_MAX_ROUTES];
} LightSomeipConfig;

typedef struct LightSomeipPacket {
    uint16_t service_id;
    uint16_t method_id;

    uint16_t client_id;
    uint16_t session_id;

    uint8_t protocol_version;
    uint8_t interface_version;
    uint8_t message_type;
    uint8_t return_code;

    uint8_t payload_arr[SOMEIP_MAX_PAYLOAD_LEN];
    uint32_t payload_len;
} LightSomeipPacket;


/* -------------------- Public Functions -------------------- */
/* SOMEIP 서비스 초기화 */
LightSomeipStatus light_someip_init(const LightSomeipConfig* config_ptr);

/* 송신용 패킷 초기화 */
LightSomeipStatus light_someip_packet_init(
    LightSomeipPacket* packet_ptr,
    uint16_t service_id,
    uint16_t method_id,
    const uint8_t* payload_arr,
    uint32_t payload_len
);

/* REQUEST 메시지 송신 */
LightSomeipStatus light_someip_request(LightSomeipPacket* packet_ptr);

/* RESPOND 메시지 송신 */
LightSomeipStatus light_someip_respond(const char* remote_ip, uint16_t remote_port, const LightSomeipPacket* request_packet_ptr, LightSomeipPacket* response_packet_ptr);

/* EVENT NOTIFIFY 메시지 송신 */
LightSomeipStatus light_someip_event_notify(const LightSomeipEndpoint* dst_endpoint_ptr, LightSomeipPacket* packet_ptr);

/* SOMEIP 패킷 수신 */
LightSomeipStatus light_someip_recv(LightSomeipPacket* packet_ptr, char remote_ip[SOMEIP_IP_LEN], uint16_t* remote_port);


#endif