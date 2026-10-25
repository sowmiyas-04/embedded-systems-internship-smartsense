#!/usr/bin/env bash
# Reproduces every verification step reported in README.md.
# Usage: ./run_all_tests.sh        (needs gcc; writes logs to test_evidence/)
set -u
SRC=smartsense_firmware.c; OUT=test_evidence; TMP=$(mktemp -d); mkdir -p "$OUT"
CFLAGS="-std=c99 -Wall -Wextra -Wpedantic -DSIMULATION"

echo "== 1. Build with strict warnings (expect zero warnings) =="
gcc $CFLAGS -O2 $SRC -o "$TMP/ss" 2> "$OUT/build_warnings.txt"; echo "gcc rc=$?, warning lines: $(wc -l < "$OUT/build_warnings.txt")"

echo "== 2. Self-test =="
"$TMP/ss" --selftest | tee "$OUT/selftest_log.txt" | tail -2

echo "== 3. AddressSanitizer + UBSan run =="
gcc -std=c99 -g -O1 -fsanitize=address,undefined -fno-sanitize-recover=all -DSIMULATION $SRC -o "$TMP/ss_san" \
  && "$TMP/ss_san" --selftest | tail -1 | tee "$OUT/sanitizer_log.txt"

echo "== 4. Demo scenario =="
"$TMP/ss" --demo --csv "$OUT/demo.csv" | tr -d '\r' > "$OUT/demo_uart_log.txt"; echo "demo rows: $(($(wc -l < "$OUT/demo.csv")-1))"

echo "== 5. Mutation testing (each mutant must make >=1 test FAIL) =="
mut() { name="$1"; expr="$2"; sed -E "$expr" $SRC > "$TMP/m.c"
  if cmp -s $SRC "$TMP/m.c"; then echo "$name | PATTERN DID NOT APPLY"; return; fi
  gcc -std=c99 -DSIMULATION "$TMP/m.c" -o "$TMP/m" 2>/dev/null || { echo "$name | DID NOT COMPILE"; return; }
  f=$("$TMP/m" --selftest | grep '^\[FAIL\]' | awk '{print $2}' | tr '\n' ',' | sed 's/,$//')
  echo "$name | ${f:-SURVIVED}"; }
{
mut "M1  Hysteresis 2.0 C -> 0"           's/(HYSTERESIS_C10 +)20/\10/'
mut "M2  Fault debounce 3 -> 1"           's/(FAULT_DEBOUNCE +)3u/\11u/'
mut "M3  Filter window 8 -> 4"            's/(FILTER_LEN +)8u/\14u/'
mut "M4  Alarm compare >= becomes >"      's/g_avg_c10 >= trip/g_avg_c10 > trip/'
mut "M5  RX overflow check removed"       's/if \(next == g_rx_tail\) \{/if (0) {/'
mut "M6  Case-folding removed"            's/c >= .a. && c <= .z./0/'
mut "M7  Missed-sample counter removed"   's/g_missed_samples\+\+;/;/'
mut "M8  Alarm LED period 200 -> 400 ms"  's/(LED_ALARM_PERIOD_MS +)200u/\1400u/'
mut "M9  Bad samples fed to filter"       's/return;( +)\/\* never filter bad data \*\//;/'
mut "M10 TX overflow check removed"       's/if \(next == g_tx_tail\) \{/if (0) {/'
mut "M11 Recovery skips filter reset"     's/filter_reset\(\);( +)\/\* discard stale history \*\//;/'
mut "M12 SET range check removed"         's/ \|\| v < THRESH_MIN_C \|\| v > THRESH_MAX_C//'
mut "M13 UART ready-gate removed"         's/ && hal_uart_tx_ready\(\)//'
mut "M14 Over-long-line guard removed"    's/if \(g_line_len >= CMD_MAX - 1u\) \{ g_line_overflow = true; return; \}//'
} | tee "$OUT/mutation_results.txt"
survivors=$(grep -c -E 'SURVIVED|DID NOT' "$OUT/mutation_results.txt")
echo "Survivors: $survivors"; rm -rf "$TMP"
