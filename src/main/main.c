/*
 * ESP32-C6 BikeControl
 *
 * ESP-IDF + native NimBLE.
 *
 * Minimal OpenBikeControl controller:
 *
 *   BOOT < 1 s  -> SHIFT DOWN (Button ID 0x02)
 *   BOOT > 1 s  -> SHIFT UP   (Button ID 0x01)
 *   BOOT > 2 s  -> 3 x SHIFT UP
 *
 * Hardware:
 *   ESP32-C6-DevKitC-1
 *   BOOT button = GPIO9, active LOW
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "nvs_flash.h"

#include "esp_log.h"
#include "esp_err.h"

#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"

#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "mdns.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include "wifi_credentials.h"

static const char *TAG = "BIKECONTROL";

#define OBC_TCP_PORT 36867
#define BOOT_GPIO              GPIO_NUM_9
#define SHIFT_UP               0x01
#define SHIFT_DOWN             0x02
#define MSG_BUTTON_STATE       0x01
#define LONG_PRESS_MS          1000
#define VERY_LONG_PRESS_MS     2000
#define BUTTON_POLL_MS         20
#define BUTTON_RELEASE_MS      50

static volatile int tcp_client_sock = -1;

static const ble_uuid128_t obc_service_uuid =
    BLE_UUID128_INIT(
        0x29, 0x52, 0x34, 0x72,
        0x04, 0xfa,
        0xd1, 0xb9,
        0x9d, 0x41, 0x48, 0xd5,
        0x80, 0xf6, 0x73, 0xd2);

static const ble_uuid128_t obc_button_uuid =
    BLE_UUID128_INIT(
        0x29, 0x52, 0x34, 0x72,
        0x04, 0xfa,
        0xd1, 0xb9,
        0x9d, 0x41, 0x48, 0xd5,
        0x81, 0xf6, 0x73, 0xd2);

static const ble_uuid128_t obc_haptic_uuid =
    BLE_UUID128_INIT(
        0x29, 0x52, 0x34, 0x72,
        0x04, 0xfa,
        0xd1, 0xb9,
        0x9d, 0x41, 0x48, 0xd5,
        0x82, 0xf6, 0x73, 0xd2);

static const ble_uuid128_t obc_app_info_uuid =
    BLE_UUID128_INIT(
        0x29, 0x52, 0x34, 0x72,
        0x04, 0xfa,
        0xd1, 0xb9,
        0x9d, 0x41, 0x48, 0xd5,
        0x83, 0xf6, 0x73, 0xd2);

/* Device Information Service */
static const ble_uuid16_t dis_service_uuid =
    BLE_UUID16_INIT(0x180A);

static const ble_uuid16_t dis_model_uuid =
    BLE_UUID16_INIT(0x2A24);

static const ble_uuid16_t dis_serial_uuid =
    BLE_UUID16_INIT(0x2A25);

static const ble_uuid16_t dis_firmware_uuid =
    BLE_UUID16_INIT(0x2A26);

static const ble_uuid16_t dis_hardware_uuid =
    BLE_UUID16_INIT(0x2A27);

static const ble_uuid16_t dis_manufacturer_uuid =
    BLE_UUID16_INIT(0x2A29);

static uint16_t button_state_handle;
static uint16_t current_conn_handle = BLE_HS_CONN_HANDLE_NONE;

/*
 * OpenBikeControl Button State:
 *
 * byte 0 = message type 0x01 (button state)
 * byte 1 = button ID
 * byte 2 = state
 *
 * state:
 *   0x01 = pressed
 *   0x00 = released
 */
static uint8_t button_state[3] = { MSG_BUTTON_STATE, 0x00, 0x00 };

static uint8_t own_addr_type;
static bool notify_enabled = false;

static const char *dis_model = "ESP32-C6";
static const char *dis_serial = "98A3169EF04042";
static const char *dis_firmware = "1.0.0";
static const char *dis_hardware = "ESP32-C6-DevKitC-1";
static const char *dis_manufacturer = "OpenBikeControl";

/********************************* WiFi  ************************************/

static bool mdns_started = false;
static bool tcp_started = false;

static void mdns_init_obc(void) {
    
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set("esp32-bikecontrol"));
    ESP_ERROR_CHECK(mdns_instance_name_set("ESP32 BikeControl"));
    ESP_ERROR_CHECK(mdns_service_add( "ESP32 BikeControl", "_openbikecontrol", "_tcp", OBC_TCP_PORT, NULL, 0));
    mdns_txt_item_t txt[] = {
        {"id", "1337"},
        {"manufacturer", "ESP32"},
        {"model", "BikeControl"},
        {"name", "ESP32 BikeControl"},
        {
            "service-uuids",
            "d273f680-d548-419d-b9d1-fa0472345229"
        },
        {"version", "0x01"}
    };
    ESP_ERROR_CHECK(mdns_service_txt_set("_openbikecontrol", "_tcp",txt, sizeof(txt) / sizeof(txt[0])));
    ESP_LOGI(TAG, "mDNS: _openbikecontrol._tcp port=%d", OBC_TCP_PORT);
}

static void tcp_send_button(uint8_t button_id, uint8_t state) {

    if (tcp_client_sock < 0) {
        return;
    }
    uint8_t msg[3] = {MSG_BUTTON_STATE, button_id, state};
    ssize_t rc = send(tcp_client_sock, msg, sizeof(msg), 0);
    if (rc < 0) {
        close(tcp_client_sock);
        tcp_client_sock = -1;
    }    
    ESP_LOGI(TAG, "TCP BUTTON TX: rc=%d [%02X %02X %02X]", rc, msg[0], msg[1], msg[2]);
}

static void tcp_send_status(void) {
    
    if (tcp_client_sock < 0) {
        return;
    }
    uint8_t msg[3] = {0x02, 0xFF, 0x01}; // brak baterii / connected / ready
    ssize_t rc = send(tcp_client_sock, msg, sizeof(msg), 0);
    ESP_LOGI(TAG, "TCP STATUS TX: rc=%d [%02X %02X %02X]", rc, msg[0], msg[1], msg[2]);
}

static void tcp_server_task(void *pvParameters) {

    int listen_sock = -1;
    struct sockaddr_in server_addr;
    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "TCP socket() failed, errno=%d", errno);
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    memset(&server_addr, 0, sizeof(server_addr));

    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    server_addr.sin_port = htons(OBC_TCP_PORT);

    int err = bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));

    if (err != 0) {
        ESP_LOGE(TAG, "TCP bind() failed, errno=%d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    err = listen(listen_sock, 1);

    if (err != 0) {
        ESP_LOGE(TAG, "TCP listen() failed, errno=%d", errno);
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "TCP server listening on port %d", OBC_TCP_PORT);

    while (1) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        ESP_LOGI(TAG, "TCP waiting for client...");
        int sock = accept(listen_sock, (struct sockaddr *)&client_addr, &client_len);

        if (sock < 0) {
            ESP_LOGE(TAG, "TCP accept() failed, errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        tcp_client_sock = sock;
        ESP_LOGI(TAG, "TCP client connected: %s", inet_ntoa(client_addr.sin_addr));
        tcp_send_status();        
        uint8_t rxbuf[128];

        while (1) {
            int len = recv(sock, rxbuf, sizeof(rxbuf), 0);
            if (len > 0) {
                ESP_LOGI(TAG, "TCP RX: %d bytes",len);
                ESP_LOG_BUFFER_HEX(TAG, rxbuf, len);
            } else if (len == 0) {
                ESP_LOGW(TAG, "TCP client disconnected");
                break;
            } else {
                ESP_LOGE(TAG, "TCP recv() error, errno=%d", errno);
                break;
            }
        }
        close(sock);
        tcp_client_sock = -1;
        ESP_LOGI(TAG, "TCP connection closed");
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected - reconnecting");
        esp_wifi_connect();
    }

    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi connected, IP=" IPSTR, IP2STR(&event->ip_info.ip));
        if (!mdns_started) {
            mdns_init_obc();
            mdns_started = true;
        }
        if (!tcp_started) {
            if (xTaskCreate(tcp_server_task, "tcp_server_task", 4096, NULL, 5, NULL) == pdPASS) {
                tcp_started = true;
            }
            else {
                ESP_LOGE(TAG, "tcp_server_task create failed");
            }
        }
    }
}

static void wifi_init(void) {

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));
    wifi_config_t wifi_config = {0};
    strcpy((char *)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char *)wifi_config.sta.password, WIFI_PASS);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {

    char uuid_str[BLE_UUID_STR_LEN];
    switch (ctxt->op) {
        case BLE_GATT_REGISTER_OP_SVC:
            ESP_LOGI(TAG, "REGISTER SVC: handle=%d uuid=%s", ctxt->svc.handle, ble_uuid_to_str(ctxt->svc.svc_def->uuid, uuid_str));
        break;
        case BLE_GATT_REGISTER_OP_CHR:
            ESP_LOGI(TAG, "REGISTER CHR: def_handle=%d val_handle=%d uuid=%s", ctxt->chr.def_handle, ctxt->chr.val_handle, ble_uuid_to_str(ctxt->chr.chr_def->uuid, uuid_str));
        break;
        case BLE_GATT_REGISTER_OP_DSC:
            ESP_LOGI(TAG, "REGISTER DSC: handle=%d uuid=%s", ctxt->dsc.handle, ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, uuid_str));
        break;
    default:
        break;
    }
}    
/* ------------------------------------------------------------------ */
/* GATT access callback                                                */
/* ------------------------------------------------------------------ */

static int gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {

    ESP_LOGI(TAG, "GATT access: conn=%u attr=%u op=%d", conn_handle, attr_handle, ctxt->op);

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        ESP_LOGI(TAG, "GATT WRITE ACCEPTED: conn=%u attr=%u len=%u", conn_handle, attr_handle, len);
        return 0;
    }

    if (ctxt->chr != NULL) {
        /* ---------------------------------------------------------- */
        /* OpenBikeControl - Button State                             */
        /* ---------------------------------------------------------- */
        if (ble_uuid_cmp(ctxt->chr->uuid, &obc_button_uuid.u) == 0) {
            if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
                int rc = os_mbuf_append(ctxt->om, button_state, sizeof(button_state));
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        }
        /* ---------------------------------------------------------- */
        /* OpenBikeControl - Haptic                                   */
        /* ---------------------------------------------------------- */
        if (ble_uuid_cmp(ctxt->chr->uuid, &obc_haptic_uuid.u) == 0) {
            if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
                ESP_LOGI(TAG, "Haptic write received (%d bytes)", OS_MBUF_PKTLEN(ctxt->om));
                return 0;
            }
        }
        /* ---------------------------------------------------------- */
        /* OpenBikeControl - App Information                          */
        /* ---------------------------------------------------------- */
        if (ble_uuid_cmp(ctxt->chr->uuid, &obc_app_info_uuid.u) == 0) {
            if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
                ESP_LOGI(TAG, "App info write received (%d bytes)", OS_MBUF_PKTLEN(ctxt->om));
                return 0;
            }
        }
        /* ---------------------------------------------------------- */
        /* Device Information - Model Number                          */
        /* ---------------------------------------------------------- */
        if (ble_uuid_cmp(ctxt->chr->uuid, &dis_model_uuid.u) == 0) {
            return os_mbuf_append(ctxt->om, dis_model, strlen(dis_model)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        /* ---------------------------------------------------------- */
        /* Device Information - Serial Number                         */
        /* ---------------------------------------------------------- */
        if (ble_uuid_cmp(ctxt->chr->uuid, &dis_serial_uuid.u) == 0) {
            return os_mbuf_append(ctxt->om, dis_serial, strlen(dis_serial)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        /* ---------------------------------------------------------- */
        /* Device Information - Firmware Revision                     */
        /* ---------------------------------------------------------- */
        if (ble_uuid_cmp(ctxt->chr->uuid, &dis_firmware_uuid.u) == 0) {
            return os_mbuf_append(ctxt->om, dis_firmware, strlen(dis_firmware)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        /* ---------------------------------------------------------- */
        /* Device Information - Hardware Revision                     */
        /* ---------------------------------------------------------- */
        if (ble_uuid_cmp(ctxt->chr->uuid, &dis_hardware_uuid.u) == 0) {
            return os_mbuf_append(ctxt->om, dis_hardware, strlen(dis_hardware)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        /* ---------------------------------------------------------- */
        /* Device Information - Manufacturer Name                     */
        /* ---------------------------------------------------------- */
        if (ble_uuid_cmp(ctxt->chr->uuid, &dis_manufacturer_uuid.u) == 0) {

            return os_mbuf_append(ctxt->om, dis_manufacturer, strlen(dis_manufacturer)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
    }

    (void)conn_handle;
    (void)attr_handle;
    (void)arg;
    ESP_LOGE(TAG, "GATT UNLIKELY ERROR: conn=%u attr=%u op=%d", conn_handle, attr_handle, ctxt->op);
    return BLE_ATT_ERR_UNLIKELY;
}

/* ------------------------------------------------------------------ */
/* GATT service table                                                   */
/* ------------------------------------------------------------------ */

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    /* ================================================================ */
    /* OpenBikeControl Service                                         */
    /* ================================================================ */
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &obc_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &obc_button_uuid.u,
                .access_cb = gatt_access_cb,
                .flags = BLE_GATT_CHR_F_READ |
                         BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &button_state_handle,
            },
            {
                .uuid = &obc_haptic_uuid.u,
                .access_cb = gatt_access_cb,

                .flags = BLE_GATT_CHR_F_WRITE |
                         BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &obc_app_info_uuid.u,
                .access_cb = gatt_access_cb,

                .flags = BLE_GATT_CHR_F_WRITE |
                         BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 }
        },
    },
    /* ================================================================ */
    /* Device Information Service                                      */
    /* ================================================================ */
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &dis_service_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                /* Model Number String - 0x2A24 */
                .uuid = &dis_model_uuid.u,
                .access_cb = gatt_access_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                /* Serial Number String - 0x2A25 */
                .uuid = &dis_serial_uuid.u,
                .access_cb = gatt_access_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                /* Firmware Revision String - 0x2A26 */
                .uuid = &dis_firmware_uuid.u,
                .access_cb = gatt_access_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                /* Hardware Revision String - 0x2A27 */
                .uuid = &dis_hardware_uuid.u,
                .access_cb = gatt_access_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                /* Manufacturer Name String - 0x2A29 */
                .uuid = &dis_manufacturer_uuid.u,
                .access_cb = gatt_access_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            { 0 }
        },
    },
    /* End of services */
    { 0 }
};

static void send_button(uint8_t button_id, uint8_t state) {
    button_state[0] = MSG_BUTTON_STATE;
    button_state[1] = button_id;
    button_state[2] = state;
    if (current_conn_handle == BLE_HS_CONN_HANDLE_NONE)
        return;
    if (!notify_enabled)
        return;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(button_state, sizeof(button_state));
    if (om != NULL) {
        int rc = ble_gatts_notify_custom(current_conn_handle, button_state_handle, om);
        ESP_LOGI(TAG,"NOTIFY_CUSTOM: conn=%d attr=%d rc=%d", current_conn_handle, button_state_handle, rc);
    }
}

static void send_shift(uint8_t shift_id) {
    /* Press */
    send_button(shift_id, 0x01);
    /* Short separation between press and release. */
    vTaskDelay(pdMS_TO_TICKS(BUTTON_RELEASE_MS));
    /* Release */
    send_button(shift_id, 0x00);
    tcp_send_button(shift_id, 0x01);
    vTaskDelay(pdMS_TO_TICKS(BUTTON_RELEASE_MS));
    tcp_send_button(shift_id, 0x00);    
}
/* ------------------------------------------------------------------ */
/* GAP / advertising                                                    */
/* ------------------------------------------------------------------ */
static void start_advertising(void);

static int gap_event_handler(struct ble_gap_event *event, void *arg) {

    (void)arg;

    ESP_LOGI(TAG, "GAP event: type=%d", event->type);
    switch (event->type) {
        case BLE_GAP_EVENT_NOTIFY_TX:
            ESP_LOGI(TAG, "NOTIFY_TX: conn=%d attr=%d status=%d indication=%d",
                    event->notify_tx.conn_handle,
                    event->notify_tx.attr_handle,
                    event->notify_tx.status,
                    event->notify_tx.indication);
        return 0;
        case BLE_GAP_EVENT_CONNECT:
            if (event->connect.status == 0) {
                current_conn_handle = event->connect.conn_handle;
                ESP_LOGI(TAG, "BLE connected, handle=%d", current_conn_handle);               
            } else {
                ESP_LOGW(TAG, "BLE connection failed, status=%d", event->connect.status);
                start_advertising();
            }
        return 0;
        case BLE_GAP_EVENT_DISCONNECT:
            notify_enabled = false;
            current_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGI(TAG, "BLE disconnected, reason=%d", event->disconnect.reason);
            start_advertising();
        return 0;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "Advertising complete");
            start_advertising();
        return 0;
        case BLE_GAP_EVENT_SUBSCRIBE:
            ESP_LOGI(TAG,
                "SUBSCRIBE: conn=%d attr=%d reason=%d prev_notify=%d cur_notify=%d",
                event->subscribe.conn_handle,
                event->subscribe.attr_handle,
                event->subscribe.reason,
                event->subscribe.prev_notify,
                event->subscribe.cur_notify);
            if (event->subscribe.attr_handle == button_state_handle) {
                notify_enabled = event->subscribe.cur_notify;
                ESP_LOGI(TAG,"Button notifications %s", notify_enabled ? "ENABLED" : "DISABLED");
            }
        return 0;
        case BLE_GAP_EVENT_MTU:
            ESP_LOGI(TAG, "MTU update: conn=%d mtu=%d",
                    event->mtu.conn_handle,
                    event->mtu.value);
        return 0;
    default:
        return 0;
    }
}

static void start_advertising(void) { 
    
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    /*
     * Generic discoverable, BLE only.
     */
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.uuids128 = (ble_uuid128_t *)&obc_service_uuid;
    fields.num_uuids128 = 1;
    fields.uuids128_is_complete = 1;

    // fields.name = (uint8_t *)ble_svc_gap_device_name();
    // fields.name_len = strlen(ble_svc_gap_device_name());
    // fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_set_fields failed: rc=%d", rc);
        return;
    }
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_gap_adv_start failed: rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Advertising started");
        ESP_LOGI(TAG, "Name: ESP32 BikeControl");
        ESP_LOGI(TAG,"Service: d273f680-d548-419d-b9d1-fa0472345229");
    }
}
/* ------------------------------------------------------------------ */
/* NimBLE synchronization                                               */
/* ------------------------------------------------------------------ */
static void on_sync(void) {
    /*
     * Infer an address type that the controller can use for advertising.
     */
    int rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_hs_id_infer_auto failed: rc=%d", rc);
        return;
    }
    start_advertising();
}

static void on_reset(int reason) {
    ESP_LOGE(TAG, "NimBLE host reset, reason=%d", reason);
}

static void nimble_host_task(void *param) {

    (void)param;
    ESP_LOGI(TAG, "NimBLE host task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/* Button task                                                          */
/* ------------------------------------------------------------------ */
static void button_task(void *arg) {

    bool pressed = false;
    TickType_t press_start = 0;
    bool long_press_sent = false;
    bool very_long_press_sent = false;

    while (1) {

        int level = gpio_get_level(BOOT_GPIO);

        /* BOOT pressed */
        if (!pressed && level == 0) {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
            level = gpio_get_level(BOOT_GPIO);
            if (level == 0) {
                pressed = true;
                press_start = xTaskGetTickCount();
                long_press_sent = false;
                very_long_press_sent = false;
                ESP_LOGI(TAG, "BOOT pressed");
            }
        }
        if (pressed) {
            
            TickType_t elapsed = xTaskGetTickCount() - press_start;
            uint32_t elapsed_ms = pdTICKS_TO_MS(elapsed);

            // > VERY_LONG_PRESS_MS -> additional 2x SHIFT UP 
            if (elapsed_ms >= VERY_LONG_PRESS_MS && !very_long_press_sent) {
                send_shift(SHIFT_UP);
                vTaskDelay(pdMS_TO_TICKS(100));
                send_shift(SHIFT_UP);
                very_long_press_sent = true;
                ESP_LOGI(TAG, "BOOT held >%dms -> total 3x SHIFT UP", VERY_LONG_PRESS_MS);
            }

            // > LONG_PRESS_MS -> SHIFT UP 
            else if (elapsed_ms >= LONG_PRESS_MS && !long_press_sent) {
                send_shift(SHIFT_UP);
                long_press_sent = true;
                ESP_LOGI(TAG, "BOOT held >%dms -> SHIFT UP", LONG_PRESS_MS);
            }
            // Button released 
            if (level != 0) {
                pressed = false;
                // short click
                if (!long_press_sent &&
                    elapsed_ms < LONG_PRESS_MS) {
                    send_shift(SHIFT_DOWN);
                    ESP_LOGI(TAG, "BOOT held <%dms -> SHIFT DOWN", LONG_PRESS_MS);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
}
/* ------------------------------------------------------------------ */
/* GATT init                                                            */
/* ------------------------------------------------------------------ */
static void gatt_svr_init(void) {

    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {ESP_LOGE(TAG, "ble_gatts_count_cfg failed: rc=%d", rc);
        abort();
    }
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {ESP_LOGE(TAG, "ble_gatts_add_svcs failed: rc=%d", rc);
        abort();
    }
}
/* ------------------------------------------------------------------ */
/* app_main                                                           */
/* ------------------------------------------------------------------ */
void app_main(void) {

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_log_level_set("*", ESP_LOG_WARN);    //TAG
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, " ESP32 BikeControl");
    ESP_LOGI(TAG, " ESP-IDF + native NimBLE");
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "FreeRTOS version: %s", tskKERNEL_VERSION_NUMBER);
    
    /* NVS is required by the Bluetooth stack. */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");
    wifi_init();
    /* BOOT button on ESP32-C6-DevKitC-1. */
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << BOOT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    /*
     * IMPORTANT:
     * Initialize NimBLE BEFORE configuring GAP/GATT.
     */
    ESP_ERROR_CHECK(nimble_port_init());
    /*
     * GAP/GATT services.
     */
    ble_svc_gap_init();
    ble_svc_gatt_init();
    /*
     * Configure our OpenBikeControl GATT database.
     */
    gatt_svr_init();
    /*
     * Device name.
     */
    int rc = ble_svc_gap_device_name_set("ESP32 BikeControl");
    if (rc != 0) {
        ESP_LOGE(TAG, "ble_svc_gap_device_name_set failed: rc=%d", rc);
    }
    /*
     * NimBLE host callbacks.
     */
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_register_cb;
    /*
     * Start NimBLE host task.
     */
    nimble_port_freertos_init(nimble_host_task);
    /*
     * Start the physical BOOT button task.
     */

    rc =xTaskCreate(button_task, "button_task", 4096, NULL, 5, NULL);
    if (rc != pdPASS){
        ESP_LOGE(TAG, "button task create failed");
    }

}