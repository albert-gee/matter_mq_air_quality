# Matter MQ Air Quality validation report

Date: 2026-10-24
Target: ESP32-H2 (`/dev/ttyUSB0`)
Toolchain: ESP-IDF 5.3.2; esp-matter `release/v1.4.2` at `4cedd4ad40821933d6cc237e80ab6604f2953fb0`

## Result

The repaired normal image builds and links successfully. It was erased, flashed, hash-verified by esptool, and exercised through the serial console on the ESP32-H2 revision 0.1. Matter, OpenThread, the console, and BLE commissioning started without the former Thread-platform assertion or a reboot.

| Configuration | Size | SHA-256 |
|---|---:|---|
| Normal | 1,650,224 bytes (`0x192e30`) | `4cd713a6ccacb9c2f648a44d4c0ceb9acd0f7a062e2e37dc9b7dd1b5837d9180` |
| OpenThread CLI | 1,707,168 bytes (`0x1a0ca0`) | `1668cccf002271f4d2939d482ee5ef3fd1316aa18fcc20d42546186649cb0bf1` |

## Reproduce

```bash
source "$HOME/.espressif/tools/activate_idf_v5.3.2.sh"
export PATH="$HOME/esp-matter/.host-python/bin:$PATH"
source "$HOME/esp-matter/export.sh"

cd "$HOME/IoT/matter_mq_air_quality"
idf.py --ccache -B build-default \
  -DSDKCONFIG="$PWD/build-default/sdkconfig" \
  -DSDKCONFIG_DEFAULTS="$PWD/sdkconfig.defaults;$PWD/sdkconfig.defaults.esp32h2" \
  build
idf.py --ccache -B build-default \
  -DSDKCONFIG="$PWD/build-default/sdkconfig" \
  -DSDKCONFIG_DEFAULTS="$PWD/sdkconfig.defaults;$PWD/sdkconfig.defaults.esp32h2" \
  -p /dev/ttyUSB0 erase-flash flash monitor
```

Build the OpenThread CLI configuration with:

```bash
idf.py --ccache -B build-optional \
  -DSDKCONFIG="$PWD/build-optional/sdkconfig" \
  -DSDKCONFIG_DEFAULTS="$PWD/sdkconfig.defaults;$PWD/sdkconfig.defaults.esp32h2;$PWD/sdkconfig.defaults.optional" \
  build
```

## Follow-up validation

- `print_sample()` now routes both `rs_norm` and `rs_ratio` through the saturating conversion helper. The largest valid review example (`3,121,052 * 1000`) is represented as `INT32_MAX`, not converted out of range.
- `mq aq-stop` immediately invalidates Matter state before its bounded worker wait. Every enabled sensor is published as `STALE`; disabled sensors remain `DISABLED`; stopped diagnostic ages use the documented unavailable sentinel `4294967295`.
- The worker publishes the stopped state again after its last in-flight sample. A timed-out stop remains pending and can be completed by another `mq aq-stop`; restart is blocked until finalization succeeds.
- On the H2, `mq aq-stop` produced `running=false`, Air Quality `UNKNOWN`, and age `4294967295` for all enabled diagnostics. A repeated `mq aq-stop` returned `ESP_OK`; `mq aq-start` returned `ESP_OK` and resumed sampling.

## Limits

No MQ sensor modules, analog mux, calibrated baseline, Matter controller, or Thread border router were attached. ADC inputs therefore floated and Air Quality correctly remained `UNKNOWN`. Physical sampling accuracy, controller reads of feature/global attributes, final-fabric removal, and calibrated classification remain hardware/controller tests.
