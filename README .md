# SmartSense: Embedded Systems Engineer Internship (YuvaIntern)

**Intern:** Sowmiya S  
**Internship period:** 19 October 2026 to 16 November 2026  
**Project:** SmartSense, a battery-powered smart home environment monitor

SmartSense is a hypothetical embedded device that measures temperature, humidity,
pressure, ambient light and motion, and sends the data over Wi-Fi to a home MQTT
broker. It is built around an ESP32-C3 module and is designed to run for more than
60 days on one battery charge (estimated 69 days at a 5-minute reporting interval).
The same project is carried through all four weekly tasks.

## Weekly progress

| Week | Task | Status | Folder |
|------|------|--------|--------|
| 1 | System requirements analysis and block diagram design | Done | `report/`, `diagrams/` |
| 2 | Firmware development and implementation | Planned | `week2_firmware/` |
| 3 | Debugging and performance optimisation | Planned | `week3_debugging/` |
| 4 | System integration and final testing report | Planned | `week4_integration/` |

## Week 1: what is included

- `report/Week1_SmartSense_Requirements_and_Block_Diagram.pdf`: the full report (PDF)
- `report/Week1_SmartSense_Requirements_and_Block_Diagram.docx`: the same report (Word)
- `diagrams/`: the figures used in the report (PNG)

### Report contents

1. Introduction: background, problem statement, objectives, scope, stakeholders
2. Methodology: approach, requirement conventions, assumptions, effort
3. System requirements analysis: 54 numbered requirements (functional, performance,
   power, real-time, non-functional), inputs and outputs, communication interfaces
4. System design: hardware block diagram, component selection, pin assignment,
   power budget, software architecture, data model, traceability, risks, verification plan
5. Conclusions and next steps

### Key design choices

| Block | Choice |
|-------|--------|
| Microcontroller | ESP32-C3-MINI-1 (RISC-V, Wi-Fi, Bluetooth LE) |
| Sensors | SHT40 (temperature, humidity), BMP280 (pressure), BH1750 (light), AM312 (PIR motion) |
| Power | 3.7 V 2000 mAh Li-ion, MCP73831 charger, AP2112K-3.3 regulator, USB-C |
| Communication | Wi-Fi, MQTT 3.1.1 over TLS; Bluetooth LE for set-up |
| Firmware | C, ESP-IDF, FreeRTOS, deep-sleep duty cycling |

## How to view

Open the PDF in `report/`. The diagrams in `diagrams/` can be opened directly on GitHub.

## Note

All current and battery-life figures are design estimates based on datasheets. They are
validated in the later weeks by simulation and testing.
