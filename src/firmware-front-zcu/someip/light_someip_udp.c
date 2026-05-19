/*
 * light_someip_udp.c
 *
 * Platform selectable UDP backend for light SOME/IP.
 *
 * Select exactly one:
 *   - SOMEIP_UDP_PLATFORM_RPI    : Raspberry Pi / Linux socket
 *   - SOMEIP_UDP_PLATFORM_TC375  : TC375 + lwIP raw UDP
 */

#include "light_someip_udp.h"

#include <stdint.h>
#include <string.h>

/* #define SOMEIP_UDP_PLATFORM_RPI */
#define SOMEIP_UDP_PLATFORM_TC375

#if defined(SOMEIP_UDP_PLATFORM_RPI) && defined(SOMEIP_UDP_PLATFORM_TC375)
#error "Select only one platform: SOMEIP_UDP_PLATFORM_RPI or SOMEIP_UDP_PLATFORM_TC375"
#endif

#if !defined(SOMEIP_UDP_PLATFORM_RPI) && !defined(SOMEIP_UDP_PLATFORM_TC375)
#error "Select one platform: SOMEIP_UDP_PLATFORM_RPI or SOMEIP_UDP_PLATFORM_TC375"
#endif

#ifndef SOMEIP_IP_LEN
#define SOMEIP_IP_LEN (16u)
#endif

#ifndef LIGHT_SOMEIP_UDP_MAX_DATAGRAM
#define LIGHT_SOMEIP_UDP_MAX_DATAGRAM (1500u)
#endif

#if defined(SOMEIP_UDP_PLATFORM_RPI)

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

static int g_udp_fd = -1;

static int set_nonblocking(int fd) {
    int flags = fcntl(fd, F_GETFL, 0);

    return (flags < 0) ? -1 : fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

int light_someip_udp_init(uint16_t port) {
    struct sockaddr_in local_addr;

    if (port == 0u) {
        return -1;
    }

    if (g_udp_fd >= 0) {
        return 0;
    }

    g_udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_udp_fd < 0) {
        return -1;
    }

    memset(&local_addr, 0, sizeof(local_addr));
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    local_addr.sin_port = htons(port);

    if ((bind(g_udp_fd, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) || (set_nonblocking(g_udp_fd) < 0)) {
        close(g_udp_fd);
        g_udp_fd = -1;
        return -1;
    }

    return 0;
}

int light_someip_udp_send(const char *dst_ip, uint16_t dst_port, const uint8_t *buf, uint32_t len) {
    struct sockaddr_in dst_addr;
    ssize_t sent_len;

    if ((g_udp_fd < 0) || (dst_ip == NULL) || (dst_port == 0u) || (buf == NULL) || (len == 0u)) {
        return -1;
    }

    memset(&dst_addr, 0, sizeof(dst_addr));
    dst_addr.sin_family = AF_INET;
    dst_addr.sin_port = htons(dst_port);

    if (inet_pton(AF_INET, dst_ip, &dst_addr.sin_addr) != 1) {
        return -1;
    }

    sent_len = sendto(g_udp_fd, buf, len, 0, (struct sockaddr *)&dst_addr, sizeof(dst_addr));

    return (sent_len == (ssize_t)len) ? 0 : -1;
}

int light_someip_udp_recv(uint8_t *buf, uint32_t buf_size, char remote_ip[SOMEIP_IP_LEN], uint16_t *remote_port) {
    struct sockaddr_in src_addr;
    socklen_t src_len = sizeof(src_addr);
    ssize_t recv_len;

    if ((g_udp_fd < 0) || (buf == NULL) || (buf_size == 0u) || (remote_ip == NULL) || (remote_port == NULL)) {
        return -1;
    }

    remote_ip[0] = '\0';
    *remote_port = 0u;

    recv_len = recvfrom(g_udp_fd, buf, buf_size, 0, (struct sockaddr *)&src_addr, &src_len);
    if (recv_len < 0) {
        return ((errno == EAGAIN) || (errno == EWOULDBLOCK)) ? 0 : -1;
    }

    if (recv_len > 0) {
        (void)inet_ntop(AF_INET, &src_addr.sin_addr, remote_ip, SOMEIP_IP_LEN);
        *remote_port = ntohs(src_addr.sin_port);
    }

    return (int)recv_len;
}

#endif /* SOMEIP_UDP_PLATFORM_RPI */

#if defined(SOMEIP_UDP_PLATFORM_TC375)

#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

static struct udp_pcb *g_udp_pcb = NULL;

static volatile uint8_t g_rx_ready = 0u;
static uint8_t g_rx_buf[LIGHT_SOMEIP_UDP_MAX_DATAGRAM];
static uint16_t g_rx_len = 0u;
static ip_addr_t g_rx_remote_ip;
static uint16_t g_rx_remote_port = 0u;

static void light_someip_udp_on_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    (void)arg;
    (void)pcb;

    if (p == NULL) {
        return;
    }

    if ((addr == NULL) || (p->tot_len > LIGHT_SOMEIP_UDP_MAX_DATAGRAM)) {
        pbuf_free(p);
        return;
    }

    g_rx_len = (uint16_t)p->tot_len;
    (void)pbuf_copy_partial(p, g_rx_buf, g_rx_len, 0u);
    g_rx_remote_ip = *addr;
    g_rx_remote_port = (uint16_t)port;
    g_rx_ready = 1u;

    pbuf_free(p);
}

int light_someip_udp_init(uint16_t port) {
    err_t err;

    if (port == 0u) {
        return -1;
    }

    if (g_udp_pcb != NULL) {
        return 0;
    }

    g_rx_ready = 0u;
    g_rx_len = 0u;
    g_rx_remote_port = 0u;

    g_udp_pcb = udp_new();
    if (g_udp_pcb == NULL) {
        return -1;
    }

    err = udp_bind(g_udp_pcb, IP_ADDR_ANY, (u16_t)port);
    if (err != ERR_OK) {
        udp_remove(g_udp_pcb);
        g_udp_pcb = NULL;
        return -1;
    }

    udp_recv(g_udp_pcb, light_someip_udp_on_recv, NULL);

    return 0;
}

int light_someip_udp_send(const char *dst_ip, uint16_t dst_port, const uint8_t *buf, uint32_t len) {
    ip_addr_t dst_addr;
    struct pbuf *tx;
    err_t err;

    if ((g_udp_pcb == NULL) || (dst_ip == NULL) || (dst_port == 0u) || (buf == NULL) || (len == 0u) || (len > 0xFFFFu)) {
        return -1;
    }

    if (ipaddr_aton(dst_ip, &dst_addr) == 0) {
        return -1;
    }

    tx = pbuf_alloc(PBUF_TRANSPORT, (u16_t)len, PBUF_RAM);
    if (tx == NULL) {
        return -1;
    }

    err = pbuf_take(tx, buf, (u16_t)len);
    if (err == ERR_OK) {
        err = udp_sendto(g_udp_pcb, tx, &dst_addr, (u16_t)dst_port);
    }

    pbuf_free(tx);

    return (err == ERR_OK) ? 0 : -1;
}

int light_someip_udp_recv(uint8_t *buf, uint32_t buf_size, char remote_ip[SOMEIP_IP_LEN], uint16_t *remote_port) {
    uint16_t copy_len;

    if ((g_udp_pcb == NULL) || (buf == NULL) || (buf_size == 0u) || (remote_ip == NULL) || (remote_port == NULL)) {
        return -1;
    }

    remote_ip[0] = '\0';
    *remote_port = 0u;

    if (g_rx_ready == 0u) {
        return 0;
    }

    if (g_rx_len > buf_size) {
        g_rx_ready = 0u;
        g_rx_len = 0u;
        return -1;
    }

    copy_len = g_rx_len;
    memcpy(buf, g_rx_buf, copy_len);
    (void)ipaddr_ntoa_r(&g_rx_remote_ip, remote_ip, SOMEIP_IP_LEN);
    *remote_port = g_rx_remote_port;

    g_rx_ready = 0u;
    g_rx_len = 0u;

    return (int)copy_len;
}

#endif /* SOMEIP_UDP_PLATFORM_TC375 */
