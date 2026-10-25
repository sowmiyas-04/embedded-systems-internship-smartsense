# SmartSense Node: Interrupt-Driven Temperature Monitor Firmware

**Week 2 Task: Firmware Development and Implementation**

A complete embedded firmware project (single C file) for a simulated ATmega328P-class sensor node. It uses a 1 kHz timer interrupt, an interrupt-fed UART command shell, a filtered temperature alarm with hysteresis and sensor-fault detection, and a status LED controller. The firmware runs unmodified on a host PC against a simulated hardware layer, and ships with 22 automated tests, sanitizer runs, mutation testing and a scripted demo.

| | |
|---|---|
| **Code** | `smartsense_firmware.c` (single file, inline-commented, short README embedded at the top) |
| **Run everything** | `bash run_all_tests.sh` |
| **Results** | 22/22 tests pass; 0 compiler warnings (`-Wall -Wextra -Wpedantic`); ASan/UBSan clean; 14/14 mutants detected |
| **Evidence** | `test_evidence/` (logs, CSV) and `docs/` (figures) |

---

## 1. What the firmware does

| # | Requirement | Where implemented |
|---|---|---|
| R1 | 1 ms system time base from a hardware timer interrupt | `timer_isr()` |
| R2 | Sample temperature every 100 ms | `timer_isr()` raises `g_sample_due`, `process_sample()` consumes it |
| R3 | Convert ADC counts to 0.1 °C (LM35, 10 mV/°C, 10-bit, 5 V ref) | `adc_to_c10()` |
| R4 | 8-sample moving-average filter | `filter_push()`, `filter_avg()` |
| R5 | Alarm when filtered temperature ≥ threshold, with 2 °C hysteresis | `process_sample()` |
| R6 | Detect open/shorted/out-of-range sensor; debounce; auto-recover | `process_sample()` |
| R7 | Status LED: heartbeat (NORMAL), 5 Hz blink (ALARM), solid (FAULT), manual override | `led_update()` |
| R8 | UART command shell with input validation | `uart_rx_isr()`, `uart_poll()`, `feed_char()`, `execute_command()` |
| R9 | Robustness: RX/TX buffer overflow, over-long lines, bad input never crash or corrupt state | ring buffers, `feed_char()`, `strtol` checks |
| R10 | Detect a main loop that is too slow (missed sampling deadline) | `g_missed_samples` |
| R11 | Periodic telemetry stream on request | `STREAM ON/OFF` |
| R12 | UART output must never block the main loop and must respect the 9600-baud line rate | TX ring + `hal_uart_tx_ready()` |

**Target hardware model (for the port):** ATmega328P at 16 MHz, LM35 on ADC0, LED on PB5, USART0 at 9600 8N1. Register-level porting notes are at the bottom of the C file.

---

## 2. How to compile and run

Needs only a C99 compiler (gcc/clang/MinGW); no hardware or simulator install is required.

```bash
# build (strict warnings)
gcc -std=c99 -Wall -Wextra -Wpedantic -O2 -DSIMULATION smartsense_firmware.c -o smartsense

./smartsense --selftest            # 22 automated tests; exit code 0 = all pass
./smartsense --demo                # 40-second scripted scenario printed on stdout
./smartsense --demo --csv out.csv  # same, plus a CSV log (used for docs/demo_plot.png)

bash run_all_tests.sh              # build + tests + sanitizers + demo + mutation testing
```

On Windows use MinGW or WSL and the same commands (`smartsense.exe`).

### UART command reference (9600 8N1, commands end with LF or CRLF, case-insensitive)

| Command | Reply | Notes |
|---|---|---|
| `HELP` | command summary | |
| `STATUS` | `STATUS: T=24.9C AVG=24.9C STATE=NORMAL THR=30C LED=AUTO N=20` | raw temp, filtered temp, state, threshold, LED mode, sample count |
| `SET <10-120>` | `OK THR=35C` or `ERR ARG range 10-120` | alarm threshold in °C. Rejects `5`, `abc`, `40x`, `121` |
| `LED AUTO\|ON\|OFF` | `OK LED=...` | ON/OFF override the state-driven pattern |
| `STREAM ON\|OFF` | `OK STREAM=...` | when ON, prints a `TLM:` line every second |
| `DIAG` | `DIAG: up=39001ms rx_ovf=0 tx_drop=0 missed=0 alarms=1` | health counters |
| `RESET` | `OK counters cleared` | clears diagnostic counters |
| anything else | `ERR UNKNOWN (try HELP)` | line > 31 chars: `ERR LINE TOO LONG` |

Asynchronous events are also printed: `EVT ALARM ON/OFF avg=..`, `EVT FAULT ..`, `EVT RECOVERED ..`.

---

## 3. Software architecture

![Architecture](docs/architecture.png)

### 3.1 Layers

* **Layer 1: Hardware via HAL.** The firmware touches hardware only through four functions: `hal_adc_read()`, `hal_led_write()`, `hal_uart_tx_ready()` and `hal_uart_send_byte()`. In this build they are backed by a simulator (variables, a UART pacing model, an edge counter). On the real chip they become register accesses. Nothing above the HAL changes when porting.
* **Layer 2: Interrupt context.** Two entry points:
  * `timer_isr()` (1 kHz) advances `g_ms`, raises `g_sample_due` every 100 ms and `g_stream_due` every 1000 ms, and moves at most one byte from the TX ring to the UART, only if the UART is ready.
  * `uart_rx_isr(byte)` pushes the received byte into the RX ring, or drops and counts it if the ring is full.

  ISRs do no formatting, no parsing and no waiting. All heavy work is deferred to the main loop.
* **Layer 3: Application (main loop).** `fw_poll()` runs forever and does sampling and the state machine, telemetry, command handling and the LED. It owns all application state.

### 3.2 How ISRs and the main loop share data (the concurrency contract)

| Shared object | Writer | Reader | Rule that makes it safe |
|---|---|---|---|
| `g_sample_due`, `g_stream_due` | ISR sets, main clears | both | one-way flag handshake; if ISR sets while still set, `g_missed_samples++` |
| `g_ms` (32-bit) | ISR | main via `millis()` | multi-byte read wrapped in `ENTER/EXIT_CRITICAL` |
| RX ring `head` / `tail` | ISR moves head, main moves tail | both | single producer, single consumer. Each index has exactly one writer. Full ring drops the new byte, never overwrites unread data |
| TX ring `head` / `tail` | main moves head, ISR moves tail | both | same pattern, roles reversed |
| `g_state`, `g_thresh_c`, filter, counters | main only | main only | never touched in an ISR, so no locking needed |

### 3.3 Key design decisions and why

1. **Flags plus super-loop instead of doing work in ISRs.** ISRs stay at a handful of instructions, so interrupt latency is predictable and a slow command can never delay the time base.
2. **Ring buffers on both UART directions.** Reception never depends on main-loop timing (up to 63 bytes buffered), and `uart_puts()` never blocks (up to 511 bytes queued).
3. **TX drained by the timer ISR with a ready-check.** This avoids a second ISR in the simulation while staying correct for 9600 baud (1.04 ms per byte, slower than the 1 ms tick). Without the check a byte would be overwritten. This was found in review and is covered by test T18 and mutant M13 (section 8).
4. **Hysteresis plus filter plus debounce.** Three independent mechanisms prevent false alarms: the moving average rejects noise, hysteresis prevents chatter around the threshold, and 3-sample debounce prevents a single glitch from declaring a fault.
5. **Bad samples never enter the filter, and the filter is flushed on recovery.** Otherwise a short open-circuit would poison the average with stale or garbage data.
6. **Integer arithmetic only.** Temperature is held as 0.1 °C integers. No floating point, which suits an 8-bit MCU without an FPU.
7. **Pattern-as-a-function LED controller.** `led_update()` computes the LED level from `(state, time, override)` and keeps no extra state, so it cannot get out of phase.

### 3.4 Resource estimate (analytical, not measured on hardware)

* **Static RAM:** RX 64 B + TX 512 B + line buffer 32 B + filter 16 B + about 50 B of scalars ≈ **0.67 KB** of the ATmega328P's 2 KB.
* **ISR load:** `timer_isr()` is roughly 40 to 100 AVR cycles of the 16,000 available per millisecond (under 1% CPU). Estimated from the code structure; it should be re-measured on hardware with a GPIO toggle and a logic analyser.

---

## 4. Operating cycle

![Flowchart](docs/flowchart.png)

### 4.1 Step-by-step explanation

1. **Boot.** `fw_init()` zeroes all state, sets the threshold to 30 °C, state `INIT`, LED off. Interrupts are then enabled.
2. **Super-loop.** `fw_poll()` is called continuously and checks four things in a fixed order.
3. **Sample due?** Every 100 ms the timer ISR sets `g_sample_due`. Main clears it and runs `process_sample()`:
   1. read the ADC;
   2. if outside 5…320 counts the sensor is open, shorted or out of range, so count it as bad (3 in a row means `FAULT`) and **stop**, leaving the filter untouched;
   3. otherwise convert to 0.1 °C and push into the moving average;
   4. run the alarm decision (below).
4. **Stream due?** Once per second the ISR sets `g_stream_due`. If `STREAM ON`, a `TLM:` status line is queued.
5. **UART poll.** Bytes received by the RX ISR are drained from the ring, assembled into a line (upper-cased, length-limited), and executed on LF/CR. Replies are queued into the TX ring.
6. **LED update.** The LED level is computed from state, time and override and written to the pin.
7. **Meanwhile, asynchronously:** every 1 ms the timer ISR also sends one queued byte if the UART is ready, and every received byte triggers the RX ISR.

### 4.2 State machine

![State machine](docs/state_machine.png)

| From → To | Condition | Output |
|---|---|---|
| INIT → NORMAL | first valid sample | |
| NORMAL → ALARM | filtered temp ≥ threshold | `EVT ALARM ON`, LED blinks 5 Hz |
| ALARM → NORMAL | filtered temp ≤ threshold − 2.0 °C | `EVT ALARM OFF`, LED heartbeat |
| NORMAL / ALARM → FAULT | 3 consecutive invalid samples | `EVT FAULT`, LED solid |
| FAULT → NORMAL | 3 consecutive valid samples; filter flushed | `EVT RECOVERED` |

Because the alarm uses the *filtered* value, an 8-sample window gives about 0.4 to 0.8 s of detection delay for a step change. This is the deliberate trade-off for noise rejection (see test T05).

### 4.3 Timing summary

| Quantity | Value |
|---|---|
| System tick | 1 ms |
| Sampling period | 100 ms (10 Hz) |
| Filter span | 8 samples = 0.8 s |
| Fault detection | 3 samples = 0.3 s (recovery: 0.3 s) |
| Telemetry | 1 s |
| TX throughput | ≤ 960 B/s (9600 baud); `HELP` (114 B) takes about 119 ms |
| LED | NORMAL: 100 ms pulse every 1 s. ALARM: 100 ms on / 100 ms off |

---

## 5. Key functions

| Function | Role |
|---|---|
| `timer_isr()` | 1 kHz tick: time, flags, TX pacing |
| `uart_rx_isr(b)` | enqueue received byte (drop + count if full) |
| `fw_init()` / `fw_poll()` | one-time setup / one pass of the main loop |
| `process_sample()` | ADC read → validate → filter → state machine |
| `filter_push()` / `filter_avg()` | O(1) moving average with running sum |
| `adc_to_c10()` | counts → 0.1 °C using 32-bit math (no overflow: 1023 × 5000 < 2³²) |
| `led_update()` | LED pattern from state, time, override |
| `uart_poll()` → `feed_char()` → `execute_command()` | RX ring → line assembly → command dispatch |
| `uart_puts()` / `uart_printf()` | non-blocking TX enqueue |
| `millis()` | atomic read of the 32-bit time counter |
| `hal_*()` | hardware boundary (simulated here) |

---

## 6. Testing

### 6.1 Approach

A simulation harness (compiled only with `-DSIMULATION`) replaces the hardware. `sim_run_ms(n)` advances time one millisecond at a time, calling the real `timer_isr()` and `fw_poll()` each tick. Tests set the "sensor" value, inject UART bytes through the real `uart_rx_isr()`, and inspect the LED pin and everything the "UART" transmitted. Because the real firmware functions run, not copies, the results reflect the actual logic. The UART model enforces 9600-baud pacing.

### 6.2 Results: `./smartsense --selftest` (from `test_evidence/selftest_log.txt`)

![Self-test run: 22 passed, 0 failed](test_evidence/screenshot_selftest.png)

*Screenshot of my own run on OnlineGDB (exit code 0).*

| ID | Test | Evidence (actual output) | Result |
|---|---|---|---|
| T01 | Initial state | INIT, LED off, tick 0, thr 30 | PASS |
| T02 | Timebase | 1000 ticks → 10 samples, 0 missed | PASS |
| T03 | ADC → temperature | c10(0)=0, c10(51)=249, c10(1023)=5000 | PASS |
| T04 | 25 °C input | NORMAL, avg 24.9 | PASS |
| T05 | Step 25→35 °C | after 3 new samples NORMAL (avg 28.7); 4th sample ALARM (avg 30.0) | PASS |
| T06 | Hysteresis | holds ALARM at 28.8 °C, clears at 26.8 °C, 1 alarm event | PASS |
| T07 | LED patterns | NORMAL 1 pulse/s, ALARM 5 blinks/s | PASS |
| T08 | Sensor fault | open sensor → FAULT, LED solid, recovers after exactly 3 good samples | PASS |
| T09 | Debounce | 1 bad sample ignored; ADC = 900 → FAULT | PASS |
| T10 | `STATUS` | reply contains `STATE=NORMAL`, `THR=30C` | PASS |
| T11 | `SET` validation | accepts 40; rejects `5`, `abc`, `40x`, `121` | PASS |
| T12 | Case / CRLF | `sTaTuS\r\n` accepted | PASS |
| T13 | LED override | ON/OFF override ALARM pattern, AUTO restores | PASS |
| T14 | Unknown command | `ERR UNKNOWN` | PASS |
| T15 | RX ring overflow | 100-byte burst → 37 dropped (capacity 63), recovers, next command OK | PASS |
| T16 | Over-long line | rejected once, assembler resynchronises | PASS |
| T17 | Stalled main loop | 250 ms stall → 1 missed sample detected, recovers | PASS |
| T18 | TX pacing | 105 B in 110 ms (≤ 960 B/s); `HELP` intact (114/114), 0 dropped | PASS |
| T19 | Telemetry | `STREAM ON` → 3 lines in 3.1 s; `OFF` silences | PASS |
| T20 | Live threshold change | raising threshold clears the alarm on the next sample | PASS |
| T21 | TX ring overflow | 6×`HELP` burst → 2 drops counted, 511 B sent, ring consistent | PASS |
| T22 | Recovery flushes filter | after recovery at 35 °C, avg = 35.1 immediately → ALARM | PASS |

**Total: 22 passed, 0 failed.**

### 6.3 Demo scenario

`./smartsense --demo` runs 40 s: 25 °C, ramp to 38 °C, hold, ramp back, sensor unplugged for 2 s, then recovery, with UART commands sent along the way.

![Demo run on OnlineGDB](test_evidence/screenshot_demo.png)

*Screenshot of my own demo run on OnlineGDB (top of the 40 s output: alarm ON, live `SET 35`, alarm OFF, alarm ON again).*

![Demo plot](docs/demo_plot.png)

Excerpt of the real UART transcript (`test_evidence/demo_uart_log.txt`):

```
> STATUS
STATUS: T=24.9C AVG=24.9C STATE=NORMAL THR=30C LED=AUTO N=20
> STREAM ON
OK STREAM=ON
TLM: T=26.3C AVG=25.9C STATE=NORMAL THR=30C LED=AUTO N=90
TLM: T=29.8C AVG=29.2C STATE=NORMAL THR=30C LED=AUTO N=110
EVT ALARM ON  avg=30.0C
> SET 35
OK THR=35C
EVT ALARM OFF avg=30.9C          <- threshold raised live, alarm clears
EVT ALARM ON  avg=35.0C
...
EVT FAULT sensor out of range    <- sensor unplugged at t = 32 s
EVT RECOVERED sensor ok
```

What the plot shows: the black (filtered) line lags the blue (raw) line, the alarm trips only when the *filtered* value crosses the threshold, and the 2 °C hysteresis prevents re-triggering on the way down.

### 6.4 Additional quality checks

*The warning check, sanitizer run and mutation testing below were run with `run_all_tests.sh` on Linux (gcc 13.3.0); logs are in `test_evidence/`. The self-test and demo screenshots above are from my own run on OnlineGDB.*

| Check | Method | Result |
|---|---|---|
| Static warnings | `gcc -std=c99 -Wall -Wextra -Wpedantic` | 0 warnings (`test_evidence/build_warnings.txt` empty) |
| Memory safety / UB | AddressSanitizer + UBSan, full self-test | clean, 22/22 pass |
| **Mutation testing** | 14 deliberate bugs injected one at a time; every one must make ≥ 1 test fail | **14/14 detected** (table below) |

| Mutant (bug injected) | Caught by |
|---|---|
| M1 Hysteresis removed | T06 |
| M2 Fault debounce 3 → 1 | T08, T09 |
| M3 Filter window 8 → 4 | T05 |
| M4 `>=` changed to `>` in alarm compare | T05 |
| M5 RX overflow check removed | T15 |
| M6 Case-folding removed | T12 |
| M7 Missed-sample counter removed | T17 |
| M8 Alarm LED period 200 → 400 ms | T07 |
| M9 Bad samples fed to filter | T08, T09, T22 |
| M10 TX overflow check removed | T21 |
| M11 Filter not flushed on recovery | T22 |
| M12 `SET` range check removed | T11 |
| M13 UART ready-gate removed | T18 |
| M14 Over-long-line guard removed | T15, T16 |

---

## 7. Verification plan

**Objective:** show with objective evidence that each requirement in section 1 is met and that identified risks are mitigated.

**Methods**

| Method | Used for |
|---|---|
| V1. Automated functional tests on the host simulation (T01–T22) | all functional requirements |
| V2. Fault-injection / abnormal-input tests (T08, T09, T15, T16, T17, T21) | robustness requirements |
| V3. Mutation testing | proves the tests can actually detect defects |
| V4. Sanitizers (ASan, UBSan) | memory errors and undefined behaviour |
| V5. Compiler warnings at `-Wall -Wextra -Wpedantic` | static quality gate |
| V6. Code review against the concurrency contract (section 3.2) | ISR/main data sharing (cannot be fully proven by a single-threaded simulation) |
| V7. Scripted end-to-end demo | integrated behaviour, human-readable evidence |
| V8. *(Planned, not done)* Hardware-in-the-loop on a real ATmega328P | ADC noise, true baud error, real interrupt latency |

**Exit criteria:** all tests pass; zero warnings; sanitizers clean; zero surviving mutants; all risks below have a mitigation and a verifying test or an explicitly stated residual risk. **All criteria are met for the simulated build.**

### Requirements traceability

| Req | Verified by |
|---|---|
| R1 time base | T02 |
| R2 100 ms sampling | T02 |
| R3 conversion | T03, T04 |
| R4 filter | T05 |
| R5 alarm + hysteresis | T05, T06, T20 |
| R6 fault detection / recovery | T08, T09, T22 |
| R7 LED | T07, T13 |
| R8 command shell | T10, T11, T12, T14 |
| R9 robustness | T15, T16, T21 (+ ASan/UBSan) |
| R10 missed-deadline detection | T17 |
| R11 telemetry | T19 |
| R12 non-blocking, baud-correct TX | T18, T21 |

---

## 8. Risk register

Likelihood (L) and Impact (I): Low / Medium / High.

| ID | Risk | L | I | Mitigation in the design | Verified by | Residual |
|---|---|---|---|---|---|---|
| K1 | RX data lost when the main loop is busy | M | M | 64-byte RX ring, ISR-fed; overflow counted, never overwrites | T15, M5 | Burst > 63 B is dropped and visible in `DIAG` |
| K2 | TX queue overflow corrupts the buffer | M | H | full check before every write; drop + count | T21, M10 | Long replies may be truncated (counted) |
| K3 | UART byte overwritten (line-rate mismatch) | H | H | `hal_uart_tx_ready()` gate; 9600-baud pacing model | T18, M13 | Real UDRE behaviour to be confirmed on hardware (V8) |
| K4 | Main loop stalls, samples lost silently | M | M | flag-overrun detection, `missed` counter in `DIAG` | T17, M7 | Detection only; no hardware watchdog yet |
| K5 | Open/shorted/damaged sensor reports bogus temperature | M | H | range check, 3-sample debounce, FAULT state, never filtered | T08, T09, M2, M9 | Failure *within* valid range is undetectable |
| K6 | Noise causes false alarms | H | M | 8-sample average plus hysteresis | T05, T06, M1, M3 | Slow response (0.4–0.8 s) by design |
| K7 | Alarm chatter near threshold | H | M | 2 °C hysteresis | T06, M1 | None observed |
| K8 | Stale data after a sensor fault | M | M | filter flushed on recovery | T22, M11 | None observed |
| K9 | Command buffer overflow | M | H | 32-byte limit, discard until newline | T16, M14, ASan | None observed |
| K10 | Malformed numeric input accepted | M | M | `strtol` with full-string check and range check | T11, M12 | None observed |
| K11 | ISR too long, jitter in time base | L | H | ISRs are a few statements, no blocking | review (V6); estimate in 3.4 | Not measured on hardware |
| K12 | Torn read of 32-bit `g_ms` on 8-bit MCU | M | M | `millis()` uses a critical section | review (V6) | **Cannot be exercised on a PC**; verify on hardware |
| K13 | 16-bit TX indices not atomic on AVR | L | M | single writer per index; port note: use 8-bit indices or critical section | review (V6) | Open until hardware port |
| K14 | Integer overflow in conversion / sums | L | M | 32-bit intermediate math; bounded ranges | UBSan, T03 | None observed |
| K15 | Simulation differs from real hardware | H | M | HAL isolation; all assumptions documented | V8 planned | **Largest residual risk** |

---

## 9. Development log

Issues found and fixed during development, each with its cause and the evidence that now covers it.

| # | Finding | How it was found | Fix |
|---|---|---|---|
| 1 | Test T09 passed even with debounce reduced to 1: it only checked state *after* recovery. | Mutation testing (M2) | Test now checks state immediately after the glitch and again after recovery |
| 2 | No test noticed a missing TX-overflow check or a missing filter flush after recovery. | Mutation testing (M10, M11) | Added T21 and T22 |
| 3 | TX drained at 1 byte/ms = 1000 B/s, but 9600 baud is only 960 B/s: would overrun the UART data register on hardware. | Timing review while writing section 4.3 | Added `hal_uart_tx_ready()`, baud-accurate simulation model, reworked T18, added mutant M13 |
| 4 | Demo temperature and filtered average were identical in the output, which hid the filter's effect. | Reading the demo log critically | Demo now uses a continuous ramp so filter lag is visible (see plot) |

### Effort log (approximate)

| Activity | Hours (approx.) |
|---|---|
| Reading the task and planning | 1 |
| Running and studying the code | 3 |
| Testing and screenshots | 2 |
| README and GitHub upload | 2 |
| **Total** | **8** |

---

## 10. Limitations and future work

* Tested in simulation only. The ADC is an ideal value source (no noise or nonlinearity), and the UART is a pacing model rather than a bit-accurate peripheral. Next step: flash to an ATmega328P (or run in Wokwi/Proteus) using the porting notes at the end of the C file, and measure ISR time with a logic analyser (V8).
* Add a hardware watchdog (`wdt_reset()` in `fw_poll()`) so a hung main loop resets the node rather than only being counted.
* Persist the threshold in EEPROM.
* Replace the timer-driven TX drain with a UDRE interrupt for full-speed UART.
* Add a checksum or framing to the command protocol if used over a noisy link.
