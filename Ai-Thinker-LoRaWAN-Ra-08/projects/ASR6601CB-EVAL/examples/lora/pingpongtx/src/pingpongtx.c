#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "tremo_uart.h"
#include "tremo_gpio.h"
#include "tremo_rcc.h"
#include "tremo_delay.h"
#include "radio.h"
#include "delay.h"

extern int __wrap_printf(const char *fmt, ...);
extern int __wrap_snprintf(char *str, size_t size, const char *fmt, ...);

#define RF_FREQUENCY                865500000
#define TX_OUTPUT_POWER             14
#define LORA_BANDWIDTH              0      // 125 kHz
#define LORA_SPREADING_FACTOR       7
#define LORA_CODINGRATE             1      // 4/5
#define LORA_PREAMBLE_LENGTH        8
#define LORA_SYMBOL_TIMEOUT         0
#define LORA_FIX_LENGTH_PAYLOAD_ON  false
#define LORA_IQ_INVERSION_ON        false

#define MAX_CHUNK_SIZE              240

static RadioEvents_t RadioEvents;

// UART0 line buffer from Teensy 4.1 (supports full 360+ byte telemetry lines)
static uint8_t  line_buf[512];
static uint16_t line_len = 0;

// Outgoing LoRa buffers (ACK priority buffer + 2-part telemetry buffers)
static uint8_t  ack_buf[256];
static uint16_t ack_len = 0;

static uint8_t  send_buf1[256];
static uint16_t pending_len1 = 0;

static uint8_t  send_buf2[256];
static uint16_t pending_len2 = 0;

// Incoming LoRa command buffer from Ground Station
static uint8_t  rx_cmd_buf[256];
static volatile bool got_cmd = false;

// Radio state flags
static volatile bool tx_busy = false;
static volatile bool restart_rx = false;
static uint32_t busy_counter = 0;

// Telemetry gating & saved precision landing GPS coordinates
// Defaults to true because Teensy 4.1 gates Serial1 output via telemetryEnabled;
// STOP over RF also gates UART queue here as a backup.
static bool tx_enabled = true;
static char saved_gps[80] = {0};

static void start_rx(void) {
    Radio.Standby();
    Radio.SetMaxPayloadLength(MODEM_LORA, 0xFF);
    Radio.Rx(0);
}

void OnTxDone(void) {
    tx_busy = false;
    restart_rx = true;
}

void OnTxTimeout(void) {
    tx_busy = false;
    restart_rx = true;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    (void)rssi;
    (void)snr;
    if (size > 250) size = 250;
    memcpy(rx_cmd_buf, payload, size);
    rx_cmd_buf[size] = '\0';
    got_cmd = true;
}

void OnRxTimeout(void) {
    restart_rx = true;
}

void OnRxError(void) {
    restart_rx = true;
}

static int str_ieq(const char *a, const char *b) {
    while (*a && *b) {
        if (toupper((unsigned char)*a) != toupper((unsigned char)*b)) return 0;
        a++;
        b++;
    }
    return (*a == '\0' && *b == '\0');
}

static int str_istarts(const char *str, const char *prefix) {
    while (*prefix) {
        if (toupper((unsigned char)*str) != toupper((unsigned char)*prefix)) return 0;
        str++;
        prefix++;
    }
    return 1;
}

static void queue_ack(const char *msg) {
    size_t n = strlen(msg);
    if (n > 250) n = 250;
    memcpy(ack_buf, msg, n);
    ack_len = (uint16_t)n;
}

static void queue_telemetry_line(const uint8_t *buf, uint16_t len) {
    if (len == 0) return;
    if (len <= MAX_CHUNK_SIZE) {
        memcpy(send_buf1, buf, len);
        pending_len1 = len;
        pending_len2 = 0;
    } else {
        // Part 1: 0x02 continuation header + first 240 bytes
        send_buf1[0] = '\x02';
        memcpy(&send_buf1[1], buf, MAX_CHUNK_SIZE);
        pending_len1 = MAX_CHUNK_SIZE + 1;

        // Part 2: remaining bytes
        uint16_t rem = len - MAX_CHUNK_SIZE;
        if (rem > 250) rem = 250;
        memcpy(send_buf2, &buf[MAX_CHUNK_SIZE], rem);
        pending_len2 = rem;
    }
}

static void poll_uart_rx(void) {
    while (uart_get_flag_status(UART0, UART_FLAG_RX_FIFO_EMPTY) != SET) {
        char c = uart_receive_data(UART0);
        if (c == '\r' || c == '\n') {
            if (line_len > 0) {
                line_buf[line_len] = '\0';
                if (strncmp((char *)line_buf, "ACK:", 4) == 0) {
                    queue_ack((char *)line_buf);
                } else if (tx_enabled) {
                    queue_telemetry_line(line_buf, line_len);
                }
                line_len = 0;
            }
        } else if (line_len < 480) {
            line_buf[line_len++] = (uint8_t)c;
        }
    }
}

static void handle_ground_command(char *cmd) {
    // Strip leading whitespace
    while (*cmd == ' ' || *cmd == '\t') cmd++;

    // Strip trailing whitespace / CR / LF
    size_t len = strlen(cmd);
    while (len > 0 && (cmd[len - 1] == '\r' || cmd[len - 1] == '\n' || cmd[len - 1] == ' ' || cmd[len - 1] == '\t')) {
        cmd[--len] = '\0';
    }
    if (len == 0) return;

    // Optional "CMD:" prefix
    if (str_istarts(cmd, "CMD:")) {
        cmd += 4;
        while (*cmd == ' ') cmd++;
    }

    // 1. START Telemetry Command
    if (str_ieq(cmd, "START") || str_ieq(cmd, "TELEM_START") || str_ieq(cmd, "TELEM:START")) {
        tx_enabled = true;
        line_len = 0;

        // Re-forward saved GPS coordinates (if any) to Teensy before starting
        if (saved_gps[0] != '\0') {
            __wrap_printf("CMD:GPS:%s\r\n", saved_gps);
            delay_ms(5);
        }
        __wrap_printf("CMD:START\r\n");

        char msg[120];
        if (saved_gps[0] != '\0') {
            __wrap_snprintf(msg, sizeof(msg), "ACK:TELEM_STARTED,TARGET_GPS=%s", saved_gps);
        } else {
            __wrap_snprintf(msg, sizeof(msg), "ACK:TELEM_STARTED");
        }
        queue_ack(msg);
        return;
    }

    // 2. STOP Telemetry Command
    if (str_ieq(cmd, "STOP") || str_ieq(cmd, "TELEM_STOP") || str_ieq(cmd, "TELEM:STOP")) {
        tx_enabled = false;
        line_len = 0;
        pending_len1 = 0;
        pending_len2 = 0;
        __wrap_printf("CMD:STOP\r\n");
        queue_ack("ACK:TELEM_STOPPED");
        return;
    }

    // 3. Precision Landing Target GPS Coordinates:
    //    Supports "GPS:lat,lon", "TARGET:lat,lon", or raw "lat,lon"
    const char *coords = cmd;
    if (str_istarts(cmd, "GPS:")) {
        coords = cmd + 4;
    } else if (str_istarts(cmd, "TARGET:")) {
        coords = cmd + 7;
    }
    while (*coords == ' ') coords++;

    if (strchr(coords, ',') != NULL) {
        strncpy(saved_gps, coords, sizeof(saved_gps) - 1);
        saved_gps[sizeof(saved_gps) - 1] = '\0';

        // Forward to Teensy 4.1 over UART0 TX (Teensy Serial1 RX Pin 0)
        ack_len = 0;
        __wrap_printf("CMD:GPS:%s\r\n", saved_gps);

        // Poll UART0 for up to 30ms to let Teensy compute local East/North & send detailed ACK
        for (int w = 0; w < 30; w++) {
            poll_uart_rx();
            if (ack_len > 0) break;
            delay_ms(1);
        }

        // Fallback ACK if Teensy did not reply on UART0 RX
        if (ack_len == 0) {
            char msg[120];
            __wrap_snprintf(msg, sizeof(msg), "ACK:TARGET_GPS=%s", saved_gps);
            queue_ack(msg);
        }
        return;
    }

    // Forward any other command (e.g. state changes 1-8, STATE:x, AUTO, L/N/R/G) to Teensy
    ack_len = 0;
    __wrap_printf("CMD:%s\r\n", cmd);
    for (int w = 0; w < 30; w++) {
        poll_uart_rx();
        if (ack_len > 0) break;
        delay_ms(1);
    }
    if (ack_len == 0) {
        char msg[120];
        __wrap_snprintf(msg, sizeof(msg), "ACK:CMD_RX=%s", cmd);
        queue_ack(msg);
    }
}

int app_start(void) {
    RadioEvents.TxDone    = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxDone    = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError   = OnRxError;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON,
                      true); // Continuous RX when not transmitting

    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
    gpio_set_iomux(GPIOB, GPIO_PIN_0, 1);   // UART0 RX (switch on BOOT)
    gpio_set_iomux(GPIOB, GPIO_PIN_1, 1);   // UART0 TX

    __wrap_printf("CanSat TX ready (Waiting for GPS & START)\r\n");
    start_rx();

    while (1) {
        // 1. Read incoming telemetry / ACKs from Teensy over UART0
        poll_uart_rx();

        // 2. Process any LoRa command received from Ground Station
        if (got_cmd) {
            got_cmd = false;
            handle_ground_command((char *)rx_cmd_buf);
        }

        // 3. Transmit queued ACK or telemetry packets when radio is free
        if (!tx_busy) {
            if (ack_len > 0) {
                tx_busy = true;
                busy_counter = 0;
                uint16_t len = ack_len;
                ack_len = 0;
                Radio.Standby();
                Radio.Send(ack_buf, (uint8_t)len);
            } else if (pending_len1 > 0) {
                tx_busy = true;
                busy_counter = 0;
                uint16_t len = pending_len1;
                pending_len1 = 0;
                Radio.Standby();
                Radio.Send(send_buf1, (uint8_t)len);
            } else if (pending_len2 > 0) {
                tx_busy = true;
                busy_counter = 0;
                uint16_t len = pending_len2;
                pending_len2 = 0;
                Radio.Standby();
                Radio.Send(send_buf2, (uint8_t)len);
            } else if (restart_rx) {
                restart_rx = false;
                start_rx();
            }
        }

        // Safety watchdog in case TX interrupt is ever missed
        if (tx_busy && ++busy_counter > 3000000) {
            tx_busy = false;
            busy_counter = 0;
            restart_rx = true;
        }

        Radio.IrqProcess();
    }
}
