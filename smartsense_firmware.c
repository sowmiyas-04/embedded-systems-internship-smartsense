/*
 * =====================================================================
 *  SmartSense Node - Interrupt-Driven Temperature Monitor Firmware
 *  Week 2 Task: Firmware Development and Implementation
 * =====================================================================
 *
 *  README (short form - see README.md for the full report)
 *  -------------------------------------------------------
 *  WHAT IT DOES
 *    Simulates an ATmega328P-class sensor node (LM35 on ADC0, status
 *    LED on PB5, UART at 9600 8N1). Firmware features:
 *      1. 1 kHz timer ISR (Timer0 CTC) = system time base.
 *      2. Temperature sampled every 100 ms, 8-sample moving average.
 *      3. Alarm state machine with hysteresis + sensor-fault detection
 *         (INIT -> NORMAL <-> ALARM, any -> FAULT -> NORMAL).
 *      4. Status LED patterns driven by state (heartbeat / fast blink /
 *         solid) with manual override.
 *      5. UART command shell (interrupt-fed RX ring buffer, throttled
 *         TX ring buffer): HELP STATUS DIAG SET <C> LED <mode>
 *         STREAM <ON|OFF> RESET.
 *
 *  HOW TO COMPILE AND RUN (host simulation, no hardware needed)
 *      gcc -std=c99 -Wall -Wextra -Wpedantic -O2 -DSIMULATION \
 *          smartsense_firmware.c -o smartsense
 *      ./smartsense --selftest        # 22 automated tests, exit 0 = pass
 *      ./smartsense --demo            # 40 s scripted scenario on stdout
 *      ./smartsense --demo --csv out.csv   # also log data for plotting
 *
 *  The "hardware" (ADC, LED pin, UART shift register) is replaced by
 *  the HAL SIMULATION section below. The firmware logic above it never
 *  touches the simulator directly, so porting = rewrite the hal_*()
 *  functions and the three ISR hooks (see PORTING NOTES at the bottom).
 * =====================================================================
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

/* ===================================================================
 *  1. CONFIGURATION
 * =================================================================== */
#define TICK_HZ            1000u   /* Timer ISR rate: 1 tick = 1 ms            */
#define SAMPLE_PERIOD_MS   100u    /* ADC sampling period                      */
#define STREAM_PERIOD_MS   1000u   /* Telemetry period when STREAM ON          */
#define FILTER_LEN         8u      /* Moving-average window (samples)          */

#define ADC_VREF_MV        5000u   /* ADC reference voltage                    */
#define ADC_FULL_SCALE     1023u   /* 10-bit ADC                               */
#define ADC_MIN_VALID      5u      /* below  => open / shorted sensor          */
#define ADC_MAX_VALID      320u    /* above  => ~156 C, beyond LM35 range      */
#define FAULT_DEBOUNCE     3u      /* consecutive bad/good samples to switch   */

#define THRESH_DEFAULT_C   30u     /* default alarm threshold (deg C)          */
#define THRESH_MIN_C       10u
#define THRESH_MAX_C       120u
#define HYSTERESIS_C10     20      /* 2.0 C hysteresis (units of 0.1 C)        */

#define RX_BUF_SIZE        64u     /* must be power of two                     */
#define TX_BUF_SIZE        512u    /* must be power of two                     */
#define CMD_MAX            32u     /* max command line length incl. NUL        */

#define LED_HEARTBEAT_PERIOD_MS 1000u  /* NORMAL: 100 ms pulse every second    */
#define LED_HEARTBEAT_ON_MS     100u
#define LED_ALARM_PERIOD_MS     200u   /* ALARM: 5 Hz blink, 50% duty          */
#define LED_ALARM_ON_MS         100u

/* ===================================================================
 *  2. HARDWARE ABSTRACTION LAYER (interface only)
 *     Implemented for simulation in section 9; on real hardware these
 *     are register accesses.
 * =================================================================== */
static uint16_t hal_adc_read(void);            /* blocking 10-bit conversion  */
static void     hal_led_write(bool on);        /* drive LED pin               */
static bool     hal_uart_tx_ready(void);       /* UDRE: data register empty   */
static void     hal_uart_send_byte(uint8_t b); /* load UART data register     */

/* Critical section. On AVR: save SREG, cli() ... restore SREG.            */
#define ENTER_CRITICAL()  ((void)0)
#define EXIT_CRITICAL()   ((void)0)

/* ===================================================================
 *  3. TYPES AND GLOBAL STATE
 *     "volatile" = written in an ISR and read in main (or vice versa).
 * =================================================================== */
typedef enum { ST_INIT = 0, ST_NORMAL, ST_ALARM, ST_FAULT } sys_state_t;
typedef enum { LED_AUTO = 0, LED_FORCE_ON, LED_FORCE_OFF } led_mode_t;

static const char *const STATE_NAME[] = { "INIT", "NORMAL", "ALARM", "FAULT" };
static const char *const LED_NAME[]   = { "AUTO", "ON", "OFF" };

/* --- shared between ISRs and main loop --- */
static volatile uint32_t g_ms;               /* ms since boot (ISR writes)    */
static volatile bool     g_sample_due;       /* set by timer ISR              */
static volatile bool     g_stream_due;       /* set by timer ISR              */
static volatile uint16_t g_missed_samples;   /* main loop too slow            */

static volatile uint8_t  g_rx_buf[RX_BUF_SIZE];
static volatile uint8_t  g_rx_head;          /* written by RX ISR             */
static volatile uint8_t  g_rx_tail;          /* written by main               */
static volatile uint16_t g_rx_overflow;      /* bytes dropped: RX ring full   */

static volatile uint8_t  g_tx_buf[TX_BUF_SIZE];
static volatile uint16_t g_tx_head;          /* written by main               */
static volatile uint16_t g_tx_tail;          /* written by timer ISR          */
static volatile uint16_t g_tx_dropped;       /* bytes dropped: TX ring full   */

/* --- main-loop private state --- */
static sys_state_t g_state;
static led_mode_t  g_led_mode;
static uint16_t    g_thresh_c;               /* alarm threshold, deg C        */
static int16_t     g_temp_c10;               /* last valid sample, 0.1 C      */
static int16_t     g_avg_c10;                /* filtered temperature, 0.1 C   */
static uint32_t    g_samples;                /* total samples taken           */
static uint16_t    g_alarm_count;            /* ALARM entries since reset     */
static uint8_t     g_bad_run, g_good_run;    /* fault debounce counters       */
static bool        g_stream_on;

static int16_t     f_buf[FILTER_LEN];        /* moving-average ring           */
static uint8_t     f_idx, f_count;
static int32_t     f_sum;

static char        g_line[CMD_MAX];          /* command assembler             */
static uint8_t     g_line_len;
static bool        g_line_overflow;

/* ===================================================================
 *  4. INTERRUPT SERVICE ROUTINES
 *     Rules: short, no printf, no blocking, only touch volatile state.
 * =================================================================== */

/* Timer0 compare-match ISR - fires every 1 ms.
 * Jobs: advance time, raise sample/stream flags, and feed the UART: at
 * most one byte per tick, and only when the UART data register is empty
 * (9600 baud 8N1 = 1.04 ms/byte, i.e. slightly slower than our tick, so
 * the ready-check is REQUIRED or a byte would be overwritten). The main
 * loop therefore never blocks on UART. */
static void timer_isr(void)
{
    g_ms++;

    if ((g_ms % SAMPLE_PERIOD_MS) == 0u) {
        if (g_sample_due) {            /* previous sample not yet consumed */
            g_missed_samples++;
        }
        g_sample_due = true;
    }
    if ((g_ms % STREAM_PERIOD_MS) == 0u) {
        g_stream_due = true;
    }

    if (g_tx_tail != g_tx_head && hal_uart_tx_ready()) {   /* TX service */
        hal_uart_send_byte(g_tx_buf[g_tx_tail]);
        g_tx_tail = (uint16_t)((g_tx_tail + 1u) & (TX_BUF_SIZE - 1u));
    }
}

/* UART RX-complete ISR - called once per received byte.
 * Single-producer ring: only this ISR advances head. If the ring is
 * full the byte is dropped and counted (never overwrite unread data). */
static void uart_rx_isr(uint8_t byte)
{
    uint8_t next = (uint8_t)((g_rx_head + 1u) & (RX_BUF_SIZE - 1u));
    if (next == g_rx_tail) {
        g_rx_overflow++;
        return;
    }
    g_rx_buf[g_rx_head] = byte;
    g_rx_head = next;
}

/* ===================================================================
 *  5. UTILITY FUNCTIONS
 * =================================================================== */

/* Atomic read of the 32-bit ms counter (non-atomic on 8-bit MCUs). */
static uint32_t millis(void)
{
    uint32_t t;
    ENTER_CRITICAL();
    t = g_ms;
    EXIT_CRITICAL();
    return t;
}

/* ADC counts -> 0.1 deg C. LM35 = 10 mV/C, so mV == units of 0.1 C. */
static int16_t adc_to_c10(uint16_t adc)
{
    return (int16_t)(((uint32_t)adc * ADC_VREF_MV) / ADC_FULL_SCALE);
}

/* Queue a string for transmission (non-blocking; drops on overflow). */
static void uart_puts(const char *s)
{
    while (*s) {
        uint16_t next = (uint16_t)((g_tx_head + 1u) & (TX_BUF_SIZE - 1u));
        if (next == g_tx_tail) {
            g_tx_dropped++;
            return;
        }
        g_tx_buf[g_tx_head] = (uint8_t)*s++;
        g_tx_head = next;
    }
}

/* printf-style helper built on uart_puts. */
static void uart_printf(const char *fmt, ...)
{
    char tmp[128];
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    uart_puts(tmp);
}

/* ===================================================================
 *  6. SIGNAL PROCESSING AND STATE MACHINE
 * =================================================================== */

static void filter_reset(void)
{
    f_idx = 0; f_count = 0; f_sum = 0;
    memset(f_buf, 0, sizeof f_buf);
}

/* Push a sample into the moving-average ring (O(1), running sum). */
static void filter_push(int16_t v)
{
    if (f_count < FILTER_LEN) {
        f_count++;
    } else {
        f_sum -= f_buf[f_idx];         /* remove oldest */
    }
    f_buf[f_idx] = v;
    f_sum += v;
    f_idx = (uint8_t)((f_idx + 1u) % FILTER_LEN);
}

static int16_t filter_avg(void)
{
    return f_count ? (int16_t)(f_sum / f_count) : 0;
}

/* One sampling cycle: read ADC, validate, filter, run the state machine. */
static void process_sample(void)
{
    uint16_t adc = hal_adc_read();
    bool valid = (adc >= ADC_MIN_VALID) && (adc <= ADC_MAX_VALID);
    g_samples++;

    if (!valid) {                                  /* ---- bad sample ---- */
        g_good_run = 0;
        if (g_bad_run < 255u) g_bad_run++;
        if (g_bad_run >= FAULT_DEBOUNCE && g_state != ST_FAULT) {
            g_state = ST_FAULT;
            uart_puts("EVT FAULT sensor out of range\r\n");
        }
        return;                                    /* never filter bad data */
    }

    g_bad_run = 0;
    if (g_state == ST_FAULT) {                     /* ---- recovery ---- */
        if (++g_good_run < FAULT_DEBOUNCE) return;
        g_good_run = 0;
        filter_reset();                            /* discard stale history */
        g_state = ST_NORMAL;
        uart_puts("EVT RECOVERED sensor ok\r\n");
    }

    g_temp_c10 = adc_to_c10(adc);
    filter_push(g_temp_c10);
    g_avg_c10 = filter_avg();

    if (g_state == ST_INIT) g_state = ST_NORMAL;

    {   /* alarm decision with hysteresis (Schmitt-trigger behaviour) */
        int16_t trip  = (int16_t)(g_thresh_c * 10);
        int16_t clear = (int16_t)(trip - HYSTERESIS_C10);
        if (g_state == ST_NORMAL && g_avg_c10 >= trip) {
            g_state = ST_ALARM;
            g_alarm_count++;
            uart_printf("EVT ALARM ON  avg=%d.%dC\r\n", g_avg_c10 / 10, g_avg_c10 % 10);
        } else if (g_state == ST_ALARM && g_avg_c10 <= clear) {
            g_state = ST_NORMAL;
            uart_printf("EVT ALARM OFF avg=%d.%dC\r\n", g_avg_c10 / 10, g_avg_c10 % 10);
        }
    }
}

/* ===================================================================
 *  7. LED CONTROLLER (stateless: pattern is a function of time+state)
 * =================================================================== */
static void led_update(void)
{
    uint32_t t = millis();
    bool on = false;

    if (g_led_mode == LED_FORCE_ON)       on = true;
    else if (g_led_mode == LED_FORCE_OFF) on = false;
    else {
        switch (g_state) {
        case ST_ALARM: on = (t % LED_ALARM_PERIOD_MS)     < LED_ALARM_ON_MS;     break;
        case ST_FAULT: on = true;                                                  break;
        default:       on = (t % LED_HEARTBEAT_PERIOD_MS) < LED_HEARTBEAT_ON_MS;  break;
        }
    }
    hal_led_write(on);
}

/* ===================================================================
 *  8. UART COMMAND SHELL
 * =================================================================== */
static void print_status(const char *prefix)
{
    uart_printf("%s T=%d.%dC AVG=%d.%dC STATE=%s THR=%uC LED=%s N=%lu\r\n",
                prefix,
                g_temp_c10 / 10, g_temp_c10 % 10,
                g_avg_c10 / 10,  g_avg_c10 % 10,
                STATE_NAME[g_state], (unsigned)g_thresh_c,
                LED_NAME[g_led_mode], (unsigned long)g_samples);
}

static const char HELP_TEXT[] =
    "CMDS: HELP STATUS DIAG RESET\r\n"
    "      SET <10-120>   alarm threshold C\r\n"
    "      LED AUTO|ON|OFF\r\n"
    "      STREAM ON|OFF\r\n";

/* Execute one complete, upper-cased command line. */
static void execute_command(const char *l)
{
    if (strcmp(l, "HELP") == 0) {
        uart_puts(HELP_TEXT);
    } else if (strcmp(l, "STATUS") == 0) {
        print_status("STATUS:");
    } else if (strcmp(l, "DIAG") == 0) {
        uart_printf("DIAG: up=%lums rx_ovf=%u tx_drop=%u missed=%u alarms=%u\r\n",
                    (unsigned long)millis(), (unsigned)g_rx_overflow,
                    (unsigned)g_tx_dropped, (unsigned)g_missed_samples,
                    (unsigned)g_alarm_count);
    } else if (strcmp(l, "RESET") == 0) {
        g_alarm_count = 0; g_rx_overflow = 0; g_tx_dropped = 0; g_missed_samples = 0;
        uart_puts("OK counters cleared\r\n");
    } else if (strncmp(l, "SET ", 4) == 0) {
        char *end;
        long v = strtol(l + 4, &end, 10);
        if (end == l + 4 || *end != '\0' || v < THRESH_MIN_C || v > THRESH_MAX_C) {
            uart_puts("ERR ARG range 10-120\r\n");
        } else {
            g_thresh_c = (uint16_t)v;
            uart_printf("OK THR=%ldC\r\n", v);
        }
    } else if (strcmp(l, "LED AUTO") == 0) { g_led_mode = LED_AUTO;      uart_puts("OK LED=AUTO\r\n");
    } else if (strcmp(l, "LED ON")   == 0) { g_led_mode = LED_FORCE_ON;  uart_puts("OK LED=ON\r\n");
    } else if (strcmp(l, "LED OFF")  == 0) { g_led_mode = LED_FORCE_OFF; uart_puts("OK LED=OFF\r\n");
    } else if (strcmp(l, "STREAM ON")  == 0) { g_stream_on = true;  uart_puts("OK STREAM=ON\r\n");
    } else if (strcmp(l, "STREAM OFF") == 0) { g_stream_on = false; uart_puts("OK STREAM=OFF\r\n");
    } else {
        uart_puts("ERR UNKNOWN (try HELP)\r\n");
    }
}

/* Feed one received byte to the line assembler. */
static void feed_char(uint8_t c)
{
    if (c == '\r' || c == '\n') {
        if (g_line_overflow) {                    /* line was too long */
            uart_puts("ERR LINE TOO LONG\r\n");
            g_line_overflow = false;
            g_line_len = 0;
        } else if (g_line_len > 0u) {
            g_line[g_line_len] = '\0';
            execute_command(g_line);
            g_line_len = 0;
        }                                          /* empty line: ignore (CRLF) */
        return;
    }
    if (g_line_overflow) return;                   /* discard until newline */
    if (g_line_len >= CMD_MAX - 1u) { g_line_overflow = true; return; }
    if (c >= 'a' && c <= 'z') c = (uint8_t)(c - 'a' + 'A');   /* case-insensitive */
    g_line[g_line_len++] = (char)c;
}

/* Drain the RX ring (main is the only consumer: it alone moves tail). */
static void uart_poll(void)
{
    while (g_rx_tail != g_rx_head) {
        uint8_t c = g_rx_buf[g_rx_tail];
        g_rx_tail = (uint8_t)((g_rx_tail + 1u) & (RX_BUF_SIZE - 1u));
        feed_char(c);
    }
}

/* ===================================================================
 *  FIRMWARE ENTRY POINTS
 *    fw_init()  - one-time setup (also used to reset between tests)
 *    fw_poll()  - one pass of the super-loop; run as often as possible
 * =================================================================== */
static void fw_init(void)
{
    g_ms = 0; g_sample_due = false; g_stream_due = false; g_missed_samples = 0;
    g_rx_head = g_rx_tail = 0; g_rx_overflow = 0;
    g_tx_head = g_tx_tail = 0; g_tx_dropped = 0;
    g_state = ST_INIT; g_led_mode = LED_AUTO;
    g_thresh_c = THRESH_DEFAULT_C;
    g_temp_c10 = 0; g_avg_c10 = 0; g_samples = 0; g_alarm_count = 0;
    g_bad_run = 0; g_good_run = 0; g_stream_on = false;
    g_line_len = 0; g_line_overflow = false;
    filter_reset();
    hal_led_write(false);
}

static void fw_poll(void)
{
    if (g_sample_due) {                  /* flag handshake: ISR sets, main clears */
        g_sample_due = false;
        process_sample();
    }
    if (g_stream_due) {
        g_stream_due = false;
        if (g_stream_on) print_status("TLM:");
    }
    uart_poll();
    led_update();
}

/* ===================================================================
 *  9. HAL SIMULATION  +  TEST / DEMO HARNESS   (host build only)
 * =================================================================== */
#ifdef SIMULATION

/* --- simulated peripherals --- */
static uint16_t g_sim_adc = 51;          /* value the "ADC" will return (~25 C) */
static bool     g_sim_led;
static uint32_t g_sim_led_rises;         /* rising edges seen on LED pin        */
static char     g_sim_tx[8192];          /* everything the "UART" has sent      */
static size_t   g_sim_tx_len;
static bool     g_sim_echo;              /* echo UART bytes to stdout           */
static uint32_t g_sim_tx_credit = 1000u; /* 9600 baud pacing: +960 per ms, 1000 = 1 byte */

static uint16_t hal_adc_read(void) { return g_sim_adc; }

/* UART shift register model: a byte takes 10 bit-times = 1.0417 ms at 9600
 * baud, so the data register frees up 960 times per 1000 ms. */
static bool hal_uart_tx_ready(void) { return g_sim_tx_credit >= 1000u; }

static void hal_led_write(bool on)
{
    if (on && !g_sim_led) g_sim_led_rises++;
    g_sim_led = on;
}

static void hal_uart_send_byte(uint8_t b)
{
    g_sim_tx_credit -= 1000u;
    if (g_sim_tx_len < sizeof g_sim_tx - 1u) {
        g_sim_tx[g_sim_tx_len++] = (char)b;
        g_sim_tx[g_sim_tx_len] = '\0';
    }
    if (g_sim_echo) putchar(b);
}

/* --- simulator helpers --- */
static void sim_reset(void)
{
    g_sim_adc = 51; g_sim_led = false; g_sim_led_rises = 0;
    g_sim_tx_credit = 1000u;
    g_sim_tx_len = 0; g_sim_tx[0] = '\0';
    fw_init();
}
static void sim_clear_tx(void) { g_sim_tx_len = 0; g_sim_tx[0] = '\0'; }

/* One millisecond of "wall time": hardware fires the timer ISR, then the
 * main loop gets a turn. */
static void sim_run_ms(unsigned n)
{
    while (n--) {
        g_sim_tx_credit += 960u;                      /* UART shifts out bits */
        if (g_sim_tx_credit > 1999u) g_sim_tx_credit = 1999u;
        timer_isr();
        fw_poll();
    }
}

/* Deliver a string as if it arrived on the RX pin (calls the RX ISR). */
static void sim_uart_send(const char *s) { while (*s) uart_rx_isr((uint8_t)*s++); }

/* Set sensor "temperature" in 0.1 C (round to nearest ADC count). */
static void sim_set_temp_c10(int c10)
{
    g_sim_adc = (uint16_t)(((long)c10 * (long)ADC_FULL_SCALE + 2500L) / 5000L);
}

static size_t count_substr(const char *hay, const char *needle)
{
    size_t n = 0, len = strlen(needle);
    for (const char *p = hay; (p = strstr(p, needle)) != NULL; p += len) n++;
    return n;
}

/* --- tiny test framework --- */
static int t_pass, t_fail;
static void check(const char *id, const char *desc, bool ok, const char *fmt, ...)
{
    char info[160];
    va_list ap;
    va_start(ap, fmt);
    (void)vsnprintf(info, sizeof info, fmt, ap);
    va_end(ap);
    printf("[%s] %-4s %-58s | %s\n", ok ? "PASS" : "FAIL", id, desc, info);
    if (ok) t_pass++; else t_fail++;
}

static int run_selftest(void)
{
    uint32_t r0;
    printf("SmartSense self-test (host simulation)\n");
    printf("======================================================================\n");

    /* T01 */ sim_reset();
    check("T01", "Init: state INIT, LED off, tick 0, thr 30C",
          g_state == ST_INIT && !g_sim_led && g_ms == 0 && g_thresh_c == 30,
          "state=%s led=%d ms=%lu thr=%u", STATE_NAME[g_state], g_sim_led,
          (unsigned long)g_ms, (unsigned)g_thresh_c);

    /* T02 */ sim_reset(); sim_run_ms(1000);
    check("T02", "Timebase: 1000 ticks -> 10 samples, none missed",
          g_ms == 1000 && g_samples == 10 && g_missed_samples == 0,
          "ms=%lu samples=%lu missed=%u", (unsigned long)g_ms,
          (unsigned long)g_samples, (unsigned)g_missed_samples);

    /* T03 */
    check("T03", "ADC->temp: 0->0.0C, 51->24.9C, 1023->500.0C",
          adc_to_c10(0) == 0 && adc_to_c10(51) == 249 && adc_to_c10(1023) == 5000,
          "c10(0)=%d c10(51)=%d c10(1023)=%d", adc_to_c10(0), adc_to_c10(51), adc_to_c10(1023));

    /* T04 */ sim_reset(); sim_set_temp_c10(250); sim_run_ms(200);
    check("T04", "25C input -> NORMAL, avg 24.9C",
          g_state == ST_NORMAL && g_avg_c10 == 249,
          "state=%s avg=%d", STATE_NAME[g_state], g_avg_c10);

    /* T05 */ sim_reset(); sim_set_temp_c10(250); sim_run_ms(1000);
    sim_set_temp_c10(350); sim_run_ms(300);
    { sys_state_t s3 = g_state; int16_t a3 = g_avg_c10;
      sim_run_ms(100);
      check("T05", "Step 25->35C: filter delays alarm to 4th sample",
            s3 == ST_NORMAL && a3 == 287 && g_state == ST_ALARM && g_avg_c10 == 300,
            "k=3: %s avg=%d | k=4: %s avg=%d", STATE_NAME[s3], a3,
            STATE_NAME[g_state], g_avg_c10); }

    /* T06 (continues from T05: in ALARM at 35C, threshold 30C, hysteresis 2C) */
    sim_set_temp_c10(290); sim_run_ms(1000);         /* 28.8C: above 28.0 clear level */
    { sys_state_t held = g_state;
      sim_set_temp_c10(270); sim_run_ms(1000);       /* 26.8C: below clear level      */
      check("T06", "Hysteresis: holds ALARM at 28.8C, clears at 26.8C, 1 alarm",
            held == ST_ALARM && g_state == ST_NORMAL && g_alarm_count == 1,
            "at28.8=%s at26.8=%s alarms=%u", STATE_NAME[held], STATE_NAME[g_state],
            (unsigned)g_alarm_count); }

    /* T07 */ sim_reset(); sim_set_temp_c10(250); sim_run_ms(1000);
    r0 = g_sim_led_rises; sim_run_ms(1000);
    { uint32_t normal = g_sim_led_rises - r0;
      sim_set_temp_c10(400); sim_run_ms(1500);       /* drive into ALARM */
      r0 = g_sim_led_rises; sim_run_ms(1000);
      { uint32_t alarm = g_sim_led_rises - r0;
        check("T07", "LED: NORMAL=1 pulse/s, ALARM=5 blinks/s",
              normal == 1 && alarm == 5 && g_state == ST_ALARM,
              "normal=%lu alarm=%lu", (unsigned long)normal, (unsigned long)alarm); } }

    /* T08 */ sim_reset(); sim_set_temp_c10(250); sim_run_ms(500);
    g_sim_adc = 0; sim_run_ms(350);
    { bool faulted = (g_state == ST_FAULT);
      r0 = g_sim_led_rises; sim_run_ms(500);
      { bool solid = g_sim_led && (g_sim_led_rises == r0);
        sim_set_temp_c10(250); sim_run_ms(200);
        { sys_state_t mid = g_state; sim_run_ms(100);
          check("T08", "Fault: open sensor -> FAULT (LED solid), 3-sample recovery",
                faulted && solid && mid == ST_FAULT && g_state == ST_NORMAL,
                "fault=%d solid=%d after2good=%s after3good=%s", faulted, solid,
                STATE_NAME[mid], STATE_NAME[g_state]); } } }

    /* T09 */ sim_reset(); sim_set_temp_c10(250); sim_run_ms(500);
    g_sim_adc = 0; sim_run_ms(100);                  /* single glitch  */
    { bool glitch_ok = (g_state != ST_FAULT);        /* checked immediately */
      sim_set_temp_c10(250); sim_run_ms(500);
      glitch_ok = glitch_ok && (g_state != ST_FAULT);
      g_sim_adc = 900; sim_run_ms(350);              /* high = shorted/over-range */
      check("T09", "Debounce: 1 bad sample ignored; ADC=900 -> FAULT",
            glitch_ok && g_state == ST_FAULT, "glitch_ok=%d state=%s",
            glitch_ok, STATE_NAME[g_state]); }

    /* T10 */ sim_reset(); sim_set_temp_c10(250); sim_run_ms(300); sim_clear_tx();
    sim_uart_send("STATUS\r\n"); sim_run_ms(150);
    check("T10", "UART STATUS reports STATE=NORMAL and THR=30C",
          strstr(g_sim_tx, "STATE=NORMAL") && strstr(g_sim_tx, "THR=30C"),
          "reply=\"%.60s\"", g_sim_tx);

    /* T11 */ sim_clear_tx();
    sim_uart_send("SET 40\n");  sim_run_ms(30);
    { uint16_t ok40 = g_thresh_c;
      sim_uart_send("SET 5\n");   sim_run_ms(30);
      sim_uart_send("SET abc\n"); sim_run_ms(30);
      sim_uart_send("SET 40x\n"); sim_run_ms(30);
      sim_uart_send("SET 121\n"); sim_run_ms(30);
      check("T11", "SET: accepts 40; rejects 5, abc, 40x, 121",
            ok40 == 40 && g_thresh_c == 40 && count_substr(g_sim_tx, "ERR ARG") == 4,
            "thr=%u errs=%lu", (unsigned)g_thresh_c,
            (unsigned long)count_substr(g_sim_tx, "ERR ARG")); }

    /* T12 */ sim_reset(); sim_set_temp_c10(250); sim_run_ms(300); sim_clear_tx();
    sim_uart_send("sTaTuS\r\n"); sim_run_ms(150);
    check("T12", "Commands are case-insensitive, CRLF tolerated",
          strstr(g_sim_tx, "STATUS:") != NULL, "reply=\"%.40s\"", g_sim_tx);

    /* T13 */ sim_reset(); sim_set_temp_c10(400); sim_run_ms(1500);   /* ALARM */
    sim_uart_send("LED ON\n"); sim_run_ms(5);
    { bool on = g_sim_led;
      uint32_t rr = g_sim_led_rises; sim_run_ms(400);
      bool still_on = g_sim_led && g_sim_led_rises == rr;
      sim_uart_send("LED OFF\n"); sim_run_ms(5);
      bool off = !g_sim_led;
      sim_uart_send("LED AUTO\n"); sim_run_ms(400);
      check("T13", "LED override ON/OFF beats ALARM pattern; AUTO restores",
            on && still_on && off && g_sim_led_rises > rr, "on=%d steady=%d off=%d",
            on, still_on, off); }

    /* T14 */ sim_reset(); sim_clear_tx();
    sim_uart_send("FOO\n"); sim_run_ms(50);
    check("T14", "Unknown command -> ERR UNKNOWN",
          strstr(g_sim_tx, "ERR UNKNOWN") != NULL, "reply=\"%.30s\"", g_sim_tx);

    /* T15 */ sim_reset(); sim_clear_tx();
    for (int i = 0; i < 100; i++) uart_rx_isr('A');   /* burst, main not polled */
    { uint16_t ovf = g_rx_overflow;
      sim_run_ms(5);
      sim_uart_send("\n"); sim_run_ms(50);
      sim_uart_send("STATUS\n"); sim_run_ms(150);
      check("T15", "RX ring overflow: 37 bytes dropped, recovers, next cmd OK",
            ovf == 37 && strstr(g_sim_tx, "ERR LINE TOO LONG") && strstr(g_sim_tx, "STATUS:"),
            "overflow=%u", (unsigned)ovf); }

    /* T16 */ sim_reset(); sim_clear_tx();
    sim_uart_send("THIS_LINE_IS_MUCH_LONGER_THAN_31_CHARS\n"); sim_run_ms(60);
    sim_uart_send("STATUS\n"); sim_run_ms(150);
    check("T16", "Over-long line rejected, assembler resyncs",
          count_substr(g_sim_tx, "ERR LINE TOO LONG") == 1 && strstr(g_sim_tx, "STATUS:"),
          "errs=%lu", (unsigned long)count_substr(g_sim_tx, "ERR LINE TOO LONG"));

    /* T17 */ sim_reset();
    for (int i = 0; i < 250; i++) timer_isr();        /* main loop stalled 250 ms */
    { uint16_t missed = g_missed_samples;
      fw_poll();
      check("T17", "Stalled main loop: missed sample detected, recovers",
            missed == 1 && !g_sample_due && g_samples == 1,
            "missed=%u samples=%lu", (unsigned)missed, (unsigned long)g_samples); }

    /* T18 */ sim_reset(); sim_clear_tx();
    sim_uart_send("HELP\n"); sim_run_ms(110);
    { size_t at110 = g_sim_tx_len;
      sim_run_ms(300);
      check("T18", "TX paced at 9600 baud (<=106 B in 110 ms); HELP intact, no drops",
            at110 >= 100 && at110 <= 106 && g_sim_tx_len == strlen(HELP_TEXT)
            && strcmp(g_sim_tx, HELP_TEXT) == 0 && g_tx_dropped == 0,
            "sent@110ms=%zu final=%zu/%zu", at110, g_sim_tx_len, strlen(HELP_TEXT)); }

    /* T19 */ sim_reset(); sim_set_temp_c10(250); sim_run_ms(300); sim_clear_tx();
    sim_uart_send("STREAM ON\n"); sim_run_ms(3100);
    { size_t tlm = count_substr(g_sim_tx, "TLM:");
      sim_uart_send("STREAM OFF\n"); sim_clear_tx(); sim_run_ms(2100);
      check("T19", "STREAM ON -> 3 telemetry lines in 3.1 s; OFF silences",
            tlm == 3 && count_substr(g_sim_tx, "TLM:") == 0,
            "on=%zu off=%zu", tlm, count_substr(g_sim_tx, "TLM:")); }

    /* T20 */ sim_reset(); sim_set_temp_c10(400); sim_run_ms(1500);       /* ALARM at 40C */
    { bool in_alarm = (g_state == ST_ALARM);
      sim_uart_send("SET 60\n"); sim_run_ms(200);
      check("T20", "Raising threshold live clears alarm on next sample",
            in_alarm && g_state == ST_NORMAL, "was_alarm=%d now=%s",
            in_alarm, STATE_NAME[g_state]); }

    /* T21 */ sim_reset(); sim_clear_tx();
    sim_uart_send("HELP\nHELP\nHELP\nHELP\nHELP\nHELP\n");   /* 6 x 114 B > 511 B ring */
    sim_run_ms(2000);
    check("T21", "TX ring overflow: bytes dropped+counted, ring stays consistent",
          g_tx_dropped > 0 && g_tx_head == g_tx_tail && g_sim_tx_len < 6u * 114u && g_sim_tx_len >= TX_BUF_SIZE - 1u,
          "dropped=%u sent=%zu head==tail:%d", (unsigned)g_tx_dropped, g_sim_tx_len,
          g_tx_head == g_tx_tail);

    /* T22 */ sim_reset(); sim_set_temp_c10(250); sim_run_ms(1000);
    g_sim_adc = 0; sim_run_ms(350);                   /* -> FAULT */
    sim_set_temp_c10(350); sim_run_ms(300);           /* 3 good samples -> recovery */
    check("T22", "Recovery flushes stale filter history (35C seen at once)",
          g_state == ST_ALARM && g_avg_c10 == 351,
          "state=%s avg=%d", STATE_NAME[g_state], g_avg_c10);

    printf("======================================================================\n");
    printf("RESULT: %d passed, %d failed, %d total\n", t_pass, t_fail, t_pass + t_fail);
    return t_fail == 0 ? 0 : 1;
}

/* --- 40 s scripted scenario: ramp up, hold, cool down, unplug sensor --- */
static int demo_temp_c10(unsigned ms)      /* ms since start -> 0.1 C, smooth ramps */
{
    if (ms < 8000u)  return 250;
    if (ms < 16000u) return 250 + (int)(((ms - 8000u)  * 130u) / 8000u);  /* 25 -> 38 C */
    if (ms < 24000u) return 380;
    if (ms < 32000u) return 380 - (int)(((ms - 24000u) * 130u) / 8000u);  /* 38 -> 25 C */
    return 250;
}

static void demo_cmd(const char *cmd)
{
    printf("\n> %s\n", cmd);
    {   char tmp[64]; snprintf(tmp, sizeof tmp, "%s\r\n", cmd); sim_uart_send(tmp); }
}

static int run_demo(const char *csv_path)
{
    FILE *csv = NULL;
    if (csv_path) {
        csv = fopen(csv_path, "w");
        if (!csv) { perror("csv"); return 1; }
        fprintf(csv, "t_ms,adc,temp_c,avg_c,state,led,thr_c\n");
    }
    sim_reset();
    g_sim_echo = true;
    printf("SmartSense demo: 40 s scenario (UART output below)\n\n");

    for (unsigned ms = 0; ms < 40000u; ms++) {
        unsigned s = ms / 1000u;
        if (ms % 1000u == 0u) {
            if (s == 2)  demo_cmd("STATUS");
            if (s == 5)  demo_cmd("STREAM ON");
            if (s == 12) demo_cmd("SET 35");
            if (s == 34) demo_cmd("STREAM OFF");
            if (s == 36) demo_cmd("LED ON");
            if (s == 37) demo_cmd("LED AUTO");
            if (s == 38) demo_cmd("BOGUS");
            if (s == 39) demo_cmd("DIAG");
        }
        if (s >= 32 && s < 34) g_sim_adc = 0;                 /* sensor unplugged */
        else sim_set_temp_c10(demo_temp_c10(ms));

        sim_run_ms(1);

        if (csv && (ms % 100u) == 99u)
            fprintf(csv, "%u,%u,%.1f,%.1f,%s,%d,%u\n", ms + 1u, (unsigned)g_sim_adc,
                    adc_to_c10(g_sim_adc) / 10.0, g_avg_c10 / 10.0,
                    STATE_NAME[g_state], g_sim_led, (unsigned)g_thresh_c);
    }
    printf("\n--- end of demo ---\n");
    if (csv) fclose(csv);
    return 0;
}

int main(int argc, char **argv)
{
    const char *csv = NULL;
    bool demo = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--demo") == 0) demo = true;
        else if (strcmp(argv[i], "--csv") == 0 && i + 1 < argc) csv = argv[++i];
        else if (strcmp(argv[i], "--selftest") == 0) demo = false;
        else { fprintf(stderr, "usage: %s [--selftest | --demo [--csv file]]\n", argv[0]); return 2; }
    }
    return demo ? run_demo(csv) : run_selftest();
}

#endif /* SIMULATION */

/* ===================================================================
 *  PORTING NOTES - ATmega328P @ 16 MHz (not compiled in this build)
 * ===================================================================
 *  Timer0 CTC, 1 kHz : TCCR0A=(1<<WGM01); TCCR0B=(1<<CS01)|(1<<CS00);
 *                      OCR0A=249; TIMSK0=(1<<OCIE0A);
 *                      ISR(TIMER0_COMPA_vect){ timer_isr(); }
 *  USART0, 9600 8N1  : UBRR0=103; UCSR0B=(1<<RXEN0)|(1<<RXCIE0);
 *                      ISR(USART_RX_vect){ uart_rx_isr(UDR0); }
 *                      hal_uart_tx_ready(){ return UCSR0A&(1<<UDRE0); }
 *                      hal_uart_send_byte(b){ UDR0=b; }
 *                      (or move TX into a UDRE interrupt for full speed).
 *  ADC0 (LM35)       : ADMUX=(1<<REFS0); ADCSRA=(1<<ADEN)|0x07;
 *                      hal_adc_read(): start, wait ADSC, return ADC.
 *  LED on PB5        : DDRB|=(1<<PB5); hal_led_write -> PORTB bit.
 *  ENTER/EXIT_CRITICAL: uint8_t s=SREG; cli(); ... SREG=s;
 *  Super-loop        : fw_init(); sei(); for(;;) fw_poll();
 *  Also: make the 16-bit tx indices atomic (or use uint8_t, 256 B ring).
 * =================================================================== */
