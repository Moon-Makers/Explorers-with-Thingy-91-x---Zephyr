#ifndef GPS_COAP_H
#define GPS_COAP_H

// GNSS & CoAP Integration
#include <zephyr/net/coap.h>
#include <zephyr/net/tls_credentials.h>
#include <modem/modem_key_mgmt.h>
#include <nrf_modem_gnss.h>

#define SEC_TAG 12
#define APP_COAP_SEND_INTERVAL_MS 60000
#define APP_COAP_MAX_MSG_LEN 1280
#define APP_COAP_VERSION 1

/* GNSS & CoAP globals declared in source */
extern int sock;
extern struct sockaddr_storage server;
extern uint16_t next_token;
extern struct k_sem gnss_fix_sem;
extern uint8_t coap_buf[APP_COAP_MAX_MSG_LEN];
extern uint8_t coap_sendbuf[64];
extern struct nrf_modem_gnss_pvt_data_frame current_pvt;
extern struct nrf_modem_gnss_pvt_data_frame last_pvt;
extern enum tracker_status {status_nolte = DK_LED1, status_searching = DK_LED2, status_fixed = DK_LED3} device_status;

/* GNSS & CoAP function prototypes */
void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data);
void gnss_event_handler(int event);
int gnss_init_and_start(void);
int server_resolve(void);
int server_connect(void);
int client_handle_get_response(uint8_t *buf, int received);
int client_post_send(void);

// Variable para activar/desactivar el modo de prueba (true = payload falso, false = comportamiento normal)
extern bool test_mode;

#endif // GPS_COAP_H
