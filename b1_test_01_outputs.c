/*
 * ============================================================================
 * MOECC-BOARD-1  TEST 01  –  DIGITAL OUTPUT TEST  (v1.1 – bugfix)
 * Board : MOECC-BOARD-1 v1.1.1  |  MCU: ESP32-S3-WROOM-1
 *
 * Fixes over v1.0:
 *   BUG-1  AUTO cmd had no effect – auto_sequence_task ran forever with no
 *          stop flag. ANY cmd (ON/OFF/ALL OFF etc.) now pauses the sequence.
 *          Type AUTO to resume it.
 *
 *   BUG-2  ON <idx> had no effect while auto-sequence was running because
 *          auto_sequence_task was calling set_output() in the same 1-second
 *          window, immediately overwriting whatever CMD set.
 *          FIX: auto-sequence checks g_auto_run flag every loop tick and
 *          stops driving outputs as soon as it is cleared.
 *
 *   BUG-3  PULSE blocked cmd_task for the pulse duration, preventing any
 *          further UART input.
 *          FIX: PULSE now spawns a one-shot task so cmd_task stays live.
 *
 * CMD INTERFACE (UART0, 115200 8N1 on J14):
 *   ON  <idx>          Turn output ON  (1-10)
 *   OFF <idx>          Turn output OFF
 *   PULSE <idx> <ms>   Pulse output for <ms> ms (non-blocking)
 *   ALL ON             All outputs ON
 *   ALL OFF            All outputs OFF  ← also pauses auto-sequence
 *   AUTO               Resume auto-sequence
 *   STATUS             Print all states
 *   HELP
 *
 * INDEX TABLE:
 *   1=Valve1(J4)  2=Valve2(J5)  3=Valve3(J8)  4=Valve4(J9)  5=Valve5(J3)
 *   6=Pump1(J1)   7=Pump2(J2)   8=EM1(J10)    9=EM2(J7)    10=Light(J19)
 * ============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

/* ── Pin map ────────────────────────────────────────────────────────────── */
#define PIN_VALVE1      8
#define PIN_VALVE2     19
#define PIN_VALVE3     20
#define PIN_VALVE4      3
#define PIN_VALVE5     21
#define PIN_PUMP1      14
#define PIN_PUMP2      13
#define PIN_EM1        47
#define PIN_EM2        35
#define PIN_LIGHT       1

/* ── UART ────────────────────────────────────────────────────────────────── */
#define CMD_UART_NUM    UART_NUM_0
#define CMD_UART_BAUD   115200
#define CMD_RX_BUF      512
#define CMD_TX_BUF      256

/* ── Auto-sequence timing ────────────────────────────────────────────────── */
#define SEQ_ON_MS   1000
#define SEQ_OFF_MS  1000

static const char *TAG = "TEST01_OUTPUTS";

/* ── Auto-sequence control flag ──────────────────────────────────────────── */
/* Written by cmd_task, read by auto_sequence_task.
   Declared volatile – no mutex needed (single-writer, single-reader,
   bool is atomic on Xtensa).                                                */
static volatile bool g_auto_run = true;

/* ── Output descriptor ───────────────────────────────────────────────────── */
typedef struct {
    uint8_t     idx;
    gpio_num_t  pin;
    const char *name;
    const char *connector;
    const char *type;
    int         state;
} output_t;

static output_t outputs[] = {
    {  0, 0,           "---",    "---", "---",   0 },
    {  1, PIN_VALVE1,  "Valve1", "J4",  "VALVE", 0 },
    {  2, PIN_VALVE2,  "Valve2", "J5",  "VALVE", 0 },
    {  3, PIN_VALVE3,  "Valve3", "J8",  "VALVE", 0 },
    {  4, PIN_VALVE4,  "Valve4", "J9",  "VALVE", 0 },
    {  5, PIN_VALVE5,  "Valve5", "J3",  "VALVE", 0 },
    {  6, PIN_PUMP1,   "Pump1",  "J1",  "PUMP",  0 },
    {  7, PIN_PUMP2,   "Pump2",  "J2",  "PUMP",  0 },
    {  8, PIN_EM1,     "EM1",    "J10", "EM",    0 },
    {  9, PIN_EM2,     "EM2",    "J7",  "EM",    0 },
    { 10, PIN_LIGHT,   "Light",  "J19", "LIGHT", 0 },
};
#define NUM_OUTPUTS 10

/* ── GPIO init ───────────────────────────────────────────────────────────── */
static void init_outputs(void)
{
    const gpio_num_t pins[] = {
        PIN_VALVE1, PIN_VALVE2, PIN_VALVE3, PIN_VALVE4, PIN_VALVE5,
        PIN_PUMP1, PIN_PUMP2, PIN_EM1, PIN_EM2, PIN_LIGHT,
    };
    gpio_config_t cfg = {
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
        .pin_bit_mask = 0,
    };
    for (uint32_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        cfg.pin_bit_mask |= 1ULL << pins[i];
    }
    ESP_ERROR_CHECK(gpio_config(&cfg));
}

/* ── UART init ───────────────────────────────────────────────────────────── */
static void init_cmd_uart(void)
{
    uart_config_t cfg = {
        .baud_rate  = CMD_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(CMD_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_driver_install(CMD_UART_NUM,
                                        CMD_RX_BUF, CMD_TX_BUF, 0, NULL, 0));
}

/* ── Set single output (hw + state struct) ───────────────────────────────── */
static void set_output(int idx, int level)
{
    if (idx < 1 || idx > NUM_OUTPUTS) return;
    gpio_set_level(outputs[idx].pin, level);
    outputs[idx].state = level;
    ESP_LOGI(TAG, "  [%2d] %-8s (%s %-5s) → %s",
             idx, outputs[idx].name, outputs[idx].type, outputs[idx].connector,
             level ? "ON ▶" : "OFF ■");
}

/* ── Drive all outputs LOW and update state ──────────────────────────────── */
static void all_off(void)
{
    for (int i = 1; i <= NUM_OUTPUTS; i++) {
        gpio_set_level(outputs[i].pin, 0);
        outputs[i].state = 0;
    }
    ESP_LOGI(TAG, "ALL OFF");
}

/* ── Print status ────────────────────────────────────────────────────────── */
static void print_status(void)
{
    ESP_LOGI(TAG, "──── OUTPUT STATUS ─────────────────────────────────");
    ESP_LOGI(TAG, "  Auto-sequence: %s", g_auto_run ? "RUNNING" : "PAUSED");
    for (int i = 1; i <= NUM_OUTPUTS; i++) {
        ESP_LOGI(TAG, "  [%2d] GPIO%-3d %-8s %-5s %-5s  %s",
                 i, outputs[i].pin, outputs[i].name,
                 outputs[i].connector, outputs[i].type,
                 outputs[i].state ? "ON  ●" : "OFF ○");
    }
    ESP_LOGI(TAG, "────────────────────────────────────────────────────");
}

/* ── Print help ──────────────────────────────────────────────────────────── */
static void print_help(void)
{
    const char *h =
        "\r\n═══ MOECC-BOARD-1 TEST01 COMMANDS ════════════\r\n"
        "  ON  <idx>          Turn output ON  (1-10)\r\n"
        "  OFF <idx>          Turn output OFF\r\n"
        "  PULSE <idx> <ms>   Pulse output for <ms> ms\r\n"
        "  ALL ON             All outputs ON\r\n"
        "  ALL OFF            All outputs OFF + pause auto\r\n"
        "  AUTO               Resume auto-sequence\r\n"
        "  STATUS             Show all states\r\n"
        "  HELP               This list\r\n"
        "\r\n"
        "  NOTE: any ON/OFF/PULSE cmd pauses auto-sequence.\r\n"
        "        Type AUTO to resume cycling.\r\n"
        "\r\n"
        "  1=Valve1(J4)  2=Valve2(J5)  3=Valve3(J8)\r\n"
        "  4=Valve4(J9)  5=Valve5(J3)  6=Pump1(J1)\r\n"
        "  7=Pump2(J2)   8=EM1(J10)    9=EM2(J7)\r\n"
        " 10=Light(J19)\r\n"
        "═══════════════════════════════════════════════\r\n";
    uart_write_bytes(CMD_UART_NUM, h, strlen(h));
}

/* ── One-shot pulse task (spawned by PULSE cmd) ──────────────────────────── */
typedef struct { int idx; int ms; } pulse_args_t;

static void pulse_task(void *pvParameters)
{
    pulse_args_t *a = (pulse_args_t *)pvParameters;
    ESP_LOGI(TAG, "PULSE [%d] %s  %d ms", a->idx, outputs[a->idx].name, a->ms);
    set_output(a->idx, 1);
    vTaskDelay(pdMS_TO_TICKS(a->ms));
    set_output(a->idx, 0);
    free(a);
    vTaskDelete(NULL);
}

/* ── CMD processor ───────────────────────────────────────────────────────── */
static void process_cmd(const char *line)
{
    char cmd[16] = {0}, arg1[16] = {0}, arg2[16] = {0};
    sscanf(line, "%15s %15s %15s", cmd, arg1, arg2);

    /* Uppercase */
    for (char *p = cmd;  *p; p++) { if (*p >= 'a' && *p <= 'z') *p -= 32; }
    for (char *p = arg1; *p; p++) { if (*p >= 'a' && *p <= 'z') *p -= 32; }

    /* ── HELP ────────────────────────────────────────────────────────────── */
    if (strcmp(cmd, "HELP") == 0)   { print_help();  return; }

    /* ── STATUS ──────────────────────────────────────────────────────────── */
    if (strcmp(cmd, "STATUS") == 0) { print_status(); return; }

    /* ── AUTO – resume sequence ──────────────────────────────────────────── */
    if (strcmp(cmd, "AUTO") == 0) {
        g_auto_run = true;
        ESP_LOGI(TAG, "Auto-sequence RESUMED");
        return;
    }

    /* ── ALL ON / ALL OFF ────────────────────────────────────────────────── */
    if (strcmp(cmd, "ALL") == 0) {
        g_auto_run = false;   /* pause sequence first */
        vTaskDelay(pdMS_TO_TICKS(50)); /* let seq task finish its current tick */
        if (strcmp(arg1, "ON") == 0) {
            ESP_LOGW(TAG, "ALL ON – ensure loads are safe!");
            for (int i = 1; i <= NUM_OUTPUTS; i++) set_output(i, 1);
        } else {
            all_off();
        }
        return;
    }

    /* ── ON / OFF / PULSE – pause auto first ─────────────────────────────── */
    int idx = atoi(arg1);
    if (idx < 1 || idx > NUM_OUTPUTS) {
        ESP_LOGW(TAG, "Bad index '%s'  (valid: 1-%d)", arg1, NUM_OUTPUTS);
        return;
    }

    /* BUG-1 FIX: stop auto-sequence before acting on the output */
    if (g_auto_run) {
        g_auto_run = false;
        vTaskDelay(pdMS_TO_TICKS(50)); /* let seq task finish its current tick */
        ESP_LOGI(TAG, "Auto-sequence paused. Type AUTO to resume.");
    }

    if (strcmp(cmd, "ON") == 0) {
        set_output(idx, 1);
    } else if (strcmp(cmd, "OFF") == 0) {
        set_output(idx, 0);
    } else if (strcmp(cmd, "PULSE") == 0) {
        int ms = atoi(arg2);
        if (ms <= 0 || ms > 10000) { ms = 500; }
        /* BUG-3 FIX: spawn task so cmd_task doesn't block */
        pulse_args_t *a = malloc(sizeof(pulse_args_t));
        if (!a) { ESP_LOGE(TAG, "malloc fail"); return; }
        a->idx = idx;
        a->ms  = ms;
        BaseType_t r = xTaskCreate(pulse_task, "pulse", 2048, a, 5, NULL);
        if (r != pdPASS) { ESP_LOGE(TAG, "pulse task fail"); free(a); }
    } else {
        ESP_LOGW(TAG, "Unknown cmd '%s' – type HELP", cmd);
    }
}

/* ── Auto-sequence task ──────────────────────────────────────────────────── */
static void auto_sequence_task(void *pvParameters)
{
    (void)pvParameters;
    ESP_LOGI(TAG, "Auto-sequence running (cycles 1→%d). Type any cmd to pause.",
             NUM_OUTPUTS);

    int seq = 1;
    for (;;) {
        if (!g_auto_run) {
            /* Paused – make sure all outputs are off, then idle */
            all_off();
            while (!g_auto_run) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
            seq = 1;   /* restart from beginning when resumed */
            ESP_LOGI(TAG, "Auto-sequence resumed from index 1");
        }

        set_output(seq, 1);

        /* Break the ON delay into small slices so g_auto_run is checked
           quickly when a CMD arrives mid-delay.                            */
        for (int t = 0; t < SEQ_ON_MS / 50; t++) {
            if (!g_auto_run) break;
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        if (g_auto_run) {
            set_output(seq, 0);
            for (int t = 0; t < SEQ_OFF_MS / 50; t++) {
                if (!g_auto_run) break;
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }

        if (g_auto_run) {
            seq++;
            if (seq > NUM_OUTPUTS) seq = 1;
        }
    }
}

/* ── CMD task ────────────────────────────────────────────────────────────── */
static void cmd_task(void *pvParameters)
{
    (void)pvParameters;
    char    buf[128];
    int     len = 0;
    uint8_t b;

    for (;;) {
        if (uart_read_bytes(CMD_UART_NUM, &b, 1, pdMS_TO_TICKS(50)) <= 0) continue;
        uart_write_bytes(CMD_UART_NUM, (const char *)&b, 1);
        if (b == '\r' || b == '\n') {
            if (len > 0) {
                buf[len] = '\0';
                uart_write_bytes(CMD_UART_NUM, "\r\n", 2);
                process_cmd(buf);
                len = 0;
            }
        } else if (b == '\b' || b == 127) {
            if (len > 0) len--;
        } else if (len < (int)sizeof(buf) - 1) {
            buf[len++] = (char)b;
        }
    }
}

/* ── App entry ───────────────────────────────────────────────────────────── */
void app_main(void)
{
    init_outputs();
    all_off();
    init_cmd_uart();

    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "MOECC-BOARD-1  TEST 01: DIGITAL OUTPUT TEST  v1.1");
    ESP_LOGI(TAG, "Build: %s %s", __DATE__, __TIME__);
    ESP_LOGI(TAG, "Type HELP for commands  |  Any cmd pauses auto-sequence");
    ESP_LOGI(TAG, "═══════════════════════════════════════════════════════");
    print_help();

    BaseType_t r1 = xTaskCreate(auto_sequence_task, "auto_seq", 3072, NULL, 4, NULL);
    BaseType_t r2 = xTaskCreate(cmd_task,            "cmd",      4096, NULL, 6, NULL);
    if (r1 != pdPASS || r2 != pdPASS) {
        ESP_LOGE(TAG, "FATAL: task create failed");
        for (;;) vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
