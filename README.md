# Matter MQ Air Quality Sensor

Target: ESP32-H2
Framework: ESP-IDF 5.3 + ESP-Matter 1.4.2
Sensors: MQ-2, MQ-3, MQ-4, MQ-5, MQ-6, MQ-7, MQ-8, MQ-9, MQ-135
Status: experimental runtime firmware with ADC, calibration, Matter Air Quality, and NVS configuration

This project is an ESP-IDF / ESP-Matter firmware foundation for MQ air quality experiments. It currently provides ESP32-H2 ADC sampling, optional analog mux routing, MQ resistance calculation, clean-air R0 calibration, conservative Matter Air Quality updates, NVS calibration storage, and boot-time NVS configuration. It does not implement ppm estimation, external ADC access, or gas-specific alarm behavior.

```bash
get_idf
get_matter
idf.py set-target esp32h2
idf.py build
idf.py flash monitor
```

Useful development console commands:

```text
mq config
mq config-validate
mq cal-status
mq mux-config
mq mux-set-pins <adc_logical_channel> <s0> <s1> <s2> <s3> [en|-1] [settle_us]
mq mux-enable <0|1>
mq mux-select <channel>
mq mux-scan [first] [last] [samples]
mq mux-use-kit-default
mq source-read <source_id|all>
mq enable <id>
mq disable <id>
mq set-source-divider <source_id> <ratio>
mq config-reset all
```

Mux support:

- Designed for one CD74HC4067 / 74HC4067-class 16-channel analog mux.
- SIG should feed one ESP32-H2 ADC channel.
- CH0..CH8 map to MQ-2, MQ-3, MQ-4, MQ-5, MQ-6, MQ-7, MQ-8, MQ-9, MQ-135.
- Scale every MQ analog output to ESP32-safe voltage before the mux.
- Use `mq mux-set-pins`, `mq mux-enable`, `mq mux-use-kit-default`, then reboot.
- Mux diagnostics read electrical ADC voltage only. They do not mean the MQ sensor is calibrated or safe to interpret.
