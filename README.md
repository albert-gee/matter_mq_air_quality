# Matter MQ Air Quality Sensor

Target: ESP32-H2
Framework: ESP-IDF 5.3 + ESP-Matter 1.4.2
Sensors: MQ-135, MQ-2, MQ-3, MQ-4, MQ-5, MQ-6, MQ-7, MQ-8, MQ-9
Status: experimental baseline-ratio firmware for one CD74HC4067 mux kit

This project targets one fixed hardware design:

- ESP32-H2 GPIO1 / ADC1_CH0 is ADC logical channel 0.
- CD74HC4067 / 74HC4067-class mux is powered from 3.3 V.
- Mux SIG connects to ESP32-H2 GPIO1 / ADC1_CH0.
- Mux S0/S1/S2/S3 connect to GPIO10/GPIO11/GPIO12/GPIO22.
- Mux EN is tied enabled and unused by firmware.
- Every MQ module A0 line passes through a 10k/10k divider before the mux.
- MQ modules may be powered from 5 V, with shared GND.
- D0 outputs are ignored.

The firmware does not expose ppm, certified smoke detection, certified CO detection, gas-leak alarms, or safety-alarm behavior. MQ-7 and MQ-9 are analog diagnostics only in this design.

```bash
get_idf
get_matter
idf.py set-target esp32h2
idf.py build
idf.py flash monitor
```

## Channel Map

```text
sensor_id 0 = MQ-135 on mux C0  (Matter Air Quality primary)
sensor_id 1 = MQ-2   on mux C1
sensor_id 2 = MQ-3   on mux C2
sensor_id 3 = MQ-4   on mux C3
sensor_id 4 = MQ-5   on mux C4
sensor_id 5 = MQ-6   on mux C5
sensor_id 6 = MQ-7   on mux C6
sensor_id 7 = MQ-8   on mux C7
sensor_id 8 = MQ-9   on mux C8
```

All nine analog sources are mux-backed by default with divider ratio `2.0`.

## Baseline Math

ADC calibration returns the ESP32 ADC-pin voltage as `adc_mv`. The MQ voltage is corrected in the MQ sensor layer:

```text
vrl_mv = adc_mv * divider_ratio
rs_norm = (vc_mv / vrl_mv) - 1
rs_ratio = rs_norm / baseline_rs_norm
baseline_rs_norm = (vc_mv / baseline_vrl_mv) - 1
```

This follows the MQ load-resistor model while avoiding a guessed breakout-board load resistor value. The result is qualitative baseline-ratio logic, not gas concentration.

## Warm-Up

Matter starts immediately and reports Air Quality `UNKNOWN` until MQ-135 is ready. Sensors report:

```text
WARMING -> UNCALIBRATED -> READY
```

Default warm-up/aging times are 48 h for MQ-135, MQ-2, MQ-4, MQ-6, MQ-7, MQ-8, and MQ-9, and 24 h for MQ-3 and MQ-5.

## Setup Flow

1. Wire the kit exactly as listed above.
2. Flash the firmware.
3. In the console, persist the kit defaults:
   `mq use-kit-defaults`
4. Reboot.
5. Validate:
   `mq config-validate`
6. Wait for warm-up/aging to complete.
7. Baseline MQ-135:
   `mq calibrate-baseline 0`
8. Check readings:
   `mq read 0`
   `mq aq-status`
9. Commission/read Matter:
   `matter onboardingcodes`
   `matter esp attribute get <air_quality_endpoint_id> 0x005b 0x0000`

## Console Commands

```text
mq status
mq config
mq config-validate
mq use-kit-defaults
mq read <id|all>
mq source-read <source_id|all>
mq adc <logical_channel>
mq mux-config
mq mux-select <channel>
mq mux-scan [first] [last] [samples]
mq baseline-status
mq calibrate-baseline <id|all> [samples] [delay_ms]
mq calibrate-clean <id|all> [samples] [delay_ms]  # compatibility alias for baseline calibration
mq erase-baseline <id|all>
mq thresholds
mq set-warning-ratio <id> <ratio>
mq set-critical-ratio <id> <ratio>
mq set-primary 0
mq aq-status
mq aq-start
mq aq-stop
mq config-reset all
```

`mq use-kit-defaults` sets mux GPIOs, mux channels, divider ratio `2.0`, all sensor mappings, and MQ-135 as primary. It also erases stored baselines because the measurement path may have changed.

## Matter

The standard Matter Air Quality endpoint uses MQ-135 only. If MQ-135 is disabled, warming, uncalibrated, stale, or faulted, Air Quality is `UNKNOWN`.

Manufacturer-specific diagnostics on cluster `0xFC01` expose sensor id/type, enabled state, raw ADC, `adc_mv`, corrected `vrl_mv`, `baseline_vrl_mv`, `rs_norm_milli`, `rs_ratio_milli`, baseline state, threshold state, fault bitmap, and last update age.
