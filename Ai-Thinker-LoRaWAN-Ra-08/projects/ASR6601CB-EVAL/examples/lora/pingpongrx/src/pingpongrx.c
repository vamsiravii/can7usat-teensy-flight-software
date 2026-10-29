#include <stdio.h>
#include <string.h>
#include "tremo_uart.h"
#include "tremo_gpio.h"
#include "tremo_rcc.h"
#include "radio.h"
#include "delay.h"
#include "timer.h"

extern int __wrap_printf(const char *fmt, ...);

#define RF_FREQUENCY                865500000
#define TX_OUTPUT_POWER             14
#define LORA_BANDWIDTH              0      // 125 kHz
#define LORA_SPREADING_FACTOR       7
#define LORA_CODINGRATE             1      // 4/5
#define LORA_PREAMBLE_LENGTH        8
#define LORA_SYMBOL_TIMEOUT         0
#define LORA_FIX_LENGTH_PAYLOAD_ON  false
#define LORA_IQ_INVERSION_ON        false

static RadioEvents_t RadioEvents;

// LoRa RX buffer (Telemetry & ACKs from CanSat)
static uint8_t  rx_buf[256];
static uint16_t rx_len = 0;
static int16_t  rx_rssi = 0;
static volatile bool got_packet = false;
static volatile bool restart_rx = false;

// Ground Station UART0 command input buffer -> LoRa TX
static uint8_t  cmd_buf[256];
static uint16_t cmd_len = 0;
static uint8_t  tx_buf[256];
static uint16_t tx_len = 0;

// Automatic retry state if command collides with an in-flight telemetry packet
static uint8_t  last_cmd_buf[256];
static uint16_t last_cmd_len = 0;
static bool     waiting_for_ack = false;
static uint8_t  retries_left = 0;
static TimerTime_t last_cmd_sent_time = 0;

static volatile bool tx_busy = false;
static uint32_t busy_counter = 0;

static void start_rx(void) {
    Radio.Standby();
    Radio.SetMaxPayloadLength(MODEM_LORA, 0xFF);
    Radio.Rx(0);
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    (void)snr;
    if (size > 250) size = 250;
    memcpy(rx_buf, payload, size);
    rx_buf[size] = '\0';
    rx_len = size;
    rx_rssi = rssi;
    got_packet = true;
}

void OnRxTimeout(void) {
    restart_rx = true;
}

void OnRxError(void) {
    restart_rx = true;
}

void OnTxDone(void) {
    tx_busy = false;
    restart_rx = true;
}

void OnTxTimeout(void) {
    tx_busy = false;
    restart_rx = true;
}

int app_start(void) {
    RadioEvents.RxDone    = OnRxDone;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError   = OnRxError;
    RadioEvents.TxDone    = OnTxDone;
    RadioEvents.TxTimeout = OnTxTimeout;

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
                      true); // Continuous RX

    rcc_enable_peripheral_clk(RCC_PERIPHERAL_GPIOB, true);
    gpio_set_iomux(GPIOB, GPIO_PIN_0, 1);   // UART0 RX (switch on BOOT)
    gpio_set_iomux(GPIOB, GPIO_PIN_1, 1);   // UART0 TX

    __wrap_printf("Ground RX ready (Send GPS:<lat>,<lon> | START | STOP)\r\n");
    start_rx();

    while (1) {
        // 1. Poll UART0 RX for commands typed in Ground Station Serial Monitor
        while (uart_get_flag_status(UART0, UART_FLAG_RX_FIFO_EMPTY) != SET) {
            char c = uart_receive_data(UART0);
            if (c == '\r' || c == '\n') {
                if (cmd_len > 0) {
                    cmd_buf[cmd_len] = '\0';
                    memcpy(tx_buf, cmd_buf, cmd_len + 1);
                    tx_len = cmd_len;

                    memcpy(last_cmd_buf, cmd_buf, cmd_len + 1);
                    last_cmd_len = cmd_len;
                    waiting_for_ack = true;
                    retries_left = 3;

                    cmd_len = 0;
                }
            } else if (cmd_len < 245) {
                cmd_buf[cmd_len++] = (uint8_t)c;
            }
        }

        // 2. Print received telemetry or ACK packets from CanSat
        if (got_packet) {
            got_packet = false;
            if (strncmp((char *)rx_buf, "ACK:", 4) == 0) {
                waiting_for_ack = false;
                retries_left = 0;
                __wrap_printf("%s\r\n", (char *)rx_buf);
            } else if (rx_buf[0] == '\x02') {
                // Part 1 of a 2-part telemetry line: print without newline so Part 2 joins seamlessly
                __wrap_printf("%s", (char *)(rx_buf + 1));
            } else {
                __wrap_printf("%s\r\n", (char *)rx_buf);
            }
        }

        // 3. Auto-retry command if no ACK received within 350 ms (e.g. if sent during telemetry packet)
        if (waiting_for_ack && !tx_busy && tx_len == 0) {
            if (TimerGetElapsedTime(last_cmd_sent_time) >= 350) {
                if (retries_left > 0) {
                    retries_left--;
                    memcpy(tx_buf, last_cmd_buf, last_cmd_len + 1);
                    tx_len = last_cmd_len;
                } else {
                    waiting_for_ack = false;
                }
            }
        }

        // 4. Transmit Ground Station command over LoRa
        if (!tx_busy) {
            if (tx_len > 0) {
                tx_busy = true;
                busy_counter = 0;
                uint16_t len = tx_len;
                tx_len = 0;
                last_cmd_sent_time = TimerGetCurrentTime();
                __wrap_printf("[GS -> CANSAT] %s\r\n", (char *)tx_buf);
                Radio.Standby();
                Radio.Send(tx_buf, (uint8_t)len);
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
