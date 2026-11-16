# ShotGuard firmware (ESP32-S3, ESP-IDF, C)

**Prototype (MDR) goal:** FSRs taped to the outside of the boot, the IMU on the
shin, and the pod tells **touches** apart from **shots**. Every raw sample is
also saved to microSD so thresholds can be tuned on a laptop afterwards.

---

## 1. The four kinds of files

| Kind | Files | Runs where | Who cares |
|---|---|---|---|
| **Firmware (C)** | `main/*.c`, `main/*.h` | On the ESP32 (real or simulated) | Everyone |
| **Build config** | `CMakeLists.txt`, `main/CMakeLists.txt`, `main/Kconfig.projbuild`, `sdkconfig.defaults*` | Read by ESP-IDF on your laptop while compiling | Whoever builds |
| **Simulator** | `diagram.json`, `wokwi.toml` | Read by Wokwi only; ignored by ESP-IDF | Testing without a board |
| **Laptop tools (Python/C tests)** | `tools/sg_decode.py`, `test/*` | On your laptop, never on the ESP32 | App (Neel), detection (Alex) |

There is exactly one copy of the firmware source. Wokwi and the real board run
the same C code, compiled twice with different settings (section 4).

---

## 2. Firmware files, one by one

The firmware is layered. Each layer only calls the layer below it:

```
                main.c                    startup + health printout
                  |
                acq.c                     1 kHz loop: read -> log -> detect
          /     |      \        \
     fsr.c   imu.c   detect.c   logger.c  one file per job
       |       |        |          |
     ESP-IDF drivers  (pure C)   ESP-IDF SD/FAT
     (ADC, I2C)                    |
       |       |                 log_format.h  (file format, shared with Python)
     board.c  config.h           (pins, thresholds - included by everyone)
```

Every module is a `.h`/`.c` pair. The **`.h` is the public interface**: the
few functions other files may call. The **`.c` is the implementation**:
`static` variables and helper functions are private to that file. To
understand what a module does, read its `.h`; to change how it works, edit
its `.c`.

### `config.h` — settings only, no code
Every pin number, the 1 kHz sample rate, the detection thresholds and the
logging sizes. **Change hardware wiring or tuning here, not inside the `.c`
files.** It includes `sdkconfig.h`, which ESP-IDF generates at build time, so
`#if CONFIG_SHOTGUARD_SIM` can select simulator-only values.

### `main.c` — the entry point
ESP-IDF calls `app_main()` after boot (there is no `int main()`). It:
1. calls each module's `*_init()` in order: board, FSR, IMU, logger, detector
2. calls `acq_start()`, which starts the 1 kHz sampling
3. loops forever, printing a health line every 2 s:
   `samples=… overruns=… max_loop=…us imu_err=… touches=… shots=…`

   `overruns` and `max_loop` show whether you really hit 1 kHz (spec S3).

### `board.c / board.h` — Feather-specific setup
- Drives **GPIO7 high**. On the Feather this powers the STEMMA QT port the
  IMU plugs into; without it the IMU is dead. Arduino does this
  automatically; ESP-IDF doesn't. The simulator build skips it.
- Sets up the status LED (it blinks on every touch or shot).
- Creates the **I2C bus** once and shares it through `board_i2c_bus()`. The
  bus is the two wires (SDA, SCL); devices hang off it at different
  addresses. The IMU uses it now, and the Feather's battery gauge can later.
- When the PCB arrives, its differences go here and in `config.h`.

### `fsr.c / fsr.h` — Block 1, reading the pressure sensors
How an FSR becomes a number:
```
3.3 V ──[FSR]──┬──[10 kΩ]── GND
               └──► ADC pin (GPIO 5/6/9/10)
```
Pressing the FSR lowers its resistance, so the voltage at the middle node
rises. The ESP32's **ADC** (analog-to-digital converter) turns 0–3.1 V into a
count from 0 to 4095.
- `fsr_init()` uses `adc_oneshot_io_to_channel()` to map each GPIO to its
  internal ADC1 channel, sets the range (`ADC_ATTEN_DB_12`) and 12-bit
  resolution, and loads Espressif's factory calibration.
- `fsr_read_raw()` reads all 4 channels; it's called 1000× per second.
- `fsr_raw_to_mv()` converts a count to millivolts, for bench testing only.
- Only ADC1 pins are used, because ADC2 is shared with the radio. On the
  Feather, ADC1 pins are D5, D6, D9, D10 and A5.

### `imu.c / imu.h` — Block 2, reading the IMU over I2C
Talking to an I2C sensor means reading and writing its **registers**
(numbered 1-byte memory slots inside the chip, listed in its datasheet):
- *write register*: send `[address][register][value]`, e.g. set the range
  to ±30 g
- *read registers*: send `[register]`, then read N bytes. The chip
  auto-increments, so one 12-byte read starting at `ACCEL_XOUT_H` returns
  ax, ay, az, gx, gy, gz (each a 16-bit, high byte first)

`imu_init()` probes the bus for the chip, resets it, wakes it, checks its ID
register (`WHO_AM_I`), and sets the range: ±30 g, ±4000 °/s, 1125 Hz
internal rate. It also reports the scale factors (1024 counts = 1 g) so the
logs can be converted to real units.

The file contains **two drivers behind one interface**: ICM-20649 for
hardware, and an MPU6050 for Wokwi, which doesn't simulate the ICM-20649.
`#if !CONFIG_SHOTGUARD_SIM` selects which one is compiled. Nothing outside
`imu.c` knows the difference.

### `acq.c / acq.h` — the real-time loop
This is where "1000 samples per second" is enforced:
1. A **hardware timer** (GPTimer) raises an interrupt every 1000 µs.
2. The interrupt handler does one thing: it wakes `acq_task`.
   Interrupt handlers must stay tiny.
3. `acq_task` (pinned to CPU core 1, high priority) then:
   - builds one `sg_sample_t`: timestamp + 4 FSR counts + 6 IMU values
   - appends it to a 100-sample block; full blocks go to the logger
   - passes it to `detect_process()`; a touch or shot becomes an EVENT
     record for the logger, and the LED blinks
   - measures how long all this took (`max_loop_us`)

If one pass ever takes longer than 1 ms, the next wake-up shows it as an
**overrun**.

### `detect.c / detect.h` — Block 3, touch vs. shot
Pure logic, with no hardware calls, so it can be tested on a laptop
(`test/test_detect.c`). For every sample:
1. compute |a| = √(ax² + ay² + az²) in milli-g (≈1000 mg standing still)
2. remember the **peak |a| in the last 50 ms**. The leg swings *before*
   the foot meets the ball, so the detector looks backwards.
3. a **contact** is when the strongest FSR crosses `DETECT_FSR_THRESH_COUNTS`
   going up
4. on contact: peak ≥ `DETECT_SHOT_ACCEL_MG` → **SHOT**; peak ≥
   `DETECT_TOUCH_ACCEL_MG` → **TOUCH**; otherwise it's a step and is ignored
5. ignore new contacts for 300 ms (one ball contact = one event)

It also reports which zone read highest and the peak acceleration. All
thresholds are placeholders in `config.h`; tune them from real logged kicks.

### `logger.c / logger.h` — Block 4 (device side), microSD
A single SD write can freeze for 10–50 ms. Sampling can't wait that long,
so writing runs in its own task on the other core:
```
acq_task:    r = logger_get_free(); fill r; logger_submit(r);   (never waits)
logger_task: wait for r -> fwrite to SD -> give r back to the pool
```
The pool holds 8 buffers. If the card falls so far behind that none is free,
the record is dropped and counted (`dropped_records`); sampling continues.
- On boot it mounts the card over SPI and opens the next free file
  (`SG0001.BIN`, `SG0002.BIN`, …), then writes the file header.
- Every 10 records (about 1 s) it calls `fsync`. If the battery dies, you
  lose at most about 1 s of data.
- **No card (e.g. in the simulator)?** It prints to the serial console
  instead: one data line per second, and a line like
  `>>> SHOT #3 zone=laces peak=14.2 g` for each event.

### `log_format.h` — the SD file format (firmware ↔ laptop contract)
Defines the exact bytes on the card:
```
[file header 36 B][record][record][record]...
record = [header 20 B] + n × [sample 28 B]
   DATA  record: n = 100 raw samples (100 ms)
   EVENT record: n = 0, carries kind (touch/shot), zone, peak g, time
```
`__attribute__((packed))` removes padding so Python can read it with
`struct`. The `_Static_assert`s refuse to compile if someone changes a struct
size by accident. Change this file only together with `sg_decode.py`, and
bump the version.

---

## 3. Build and simulator files

| File | What it does |
|---|---|
| `CMakeLists.txt` (top) | Tells ESP-IDF "this folder is a project called shotguard". Three lines; rarely touched. |
| `main/CMakeLists.txt` | Lists every `.c` file to compile. **Add new `.c` files here** or they won't be built. |
| `main/Kconfig.projbuild` | Adds a "ShotGuard" page to `idf.py menuconfig` with one checkbox, *Build for the Wokwi simulator*, which becomes `CONFIG_SHOTGUARD_SIM` in C. |
| `sdkconfig.defaults` | Settings for every build: ESP32-S3 target, 8 MB flash, 240 MHz, 1 kHz RTOS tick. |
| `sdkconfig.defaults.feather` | Real board only: console over the S3's native USB (the Feather has no USB-serial chip). |
| `sdkconfig.defaults.sim` | Simulator only: turns on `CONFIG_SHOTGUARD_SIM`, console on UART0. |
| `sdkconfig` (generated) | ESP-IDF writes the final merged settings here during a build. Don't commit it. |
| `diagram.json` | The virtual circuit Wokwi draws: ESP32-S3 DevKit, MPU6050, 4 potentiometers standing in for the FSRs, and a microSD card, wired to the same GPIOs as the real board. |
| `wokwi.toml` | Tells Wokwi where the compiled firmware is (`build-sim/`). |

---

## 4. One codebase, two builds: Wokwi and the real board

```
              same C source (main/)
               /                    \
  sdkconfig.defaults + .sim      sdkconfig.defaults + .feather
               |                          |
        idf.py -B build-sim         idf.py -B build
               |                          |
        build-sim/*.bin             build/*.bin
               |                          |
     Wokwi runs it (MPU6050,      idf.py flash -> Feather over USB-C
     pots, virtual SD)            (ICM-20649, real FSRs, real SD)
```

**Wokwi doesn't compile your code.** You compile on your laptop with
ESP-IDF, and Wokwi runs the resulting binary on a simulated ESP32-S3. It's
the same machine code the real chip would run. That's why Wokwi works for
ESP-IDF projects and Cirkit Designer doesn't yet.

What differs between the two builds is only what's behind
`#if CONFIG_SHOTGUARD_SIM`:
| | Simulator build | Feather build |
|---|---|---|
| IMU driver | MPU6050 (±16 g) | ICM-20649 (±30 g) |
| GPIO7 STEMMA power | skipped | driven high |
| Detection thresholds | low (touch 1.3 g, shot 1.8 g) so you can trigger them with sliders | placeholders (touch 3 g, shot 8 g) |
| Console | UART0 | native USB |

Timing numbers (`overruns`, `max_loop`) only mean something on real hardware.

---

## 5. Laptop tools

### `tools/sg_decode.py`
Runs on your laptop after a session, never on the ESP32. Pull the microSD
card, then:
```
python3 tools/sg_decode.py SG0001.BIN samples.csv events.csv
```
- prints the session: number of samples, duration, and every touch/shot
  with zone and peak g
- `samples.csv`: every sample in real units (g, °/s) for plotting in
  Excel, MATLAB or pandas
- `events.csv`: one row per detected touch/shot

This is where the threshold tuning loop happens. Plot `samples.csv`, mark
where the real kicks and touches were, and adjust the numbers in
`config.h`. It's also the seed for Neel's analysis app.

### `test/test_detect.c` (+ `test/sdkconfig.h` stub)
Unit tests for `detect.c` that compile with plain `gcc` on a laptop.
Scenarios: standing, a step, a touch, a shot, a double contact, and a swing
that came too early. Run it after every change to the detector:
```
gcc -std=c11 -Wall -Itest -Imain main/detect.c test/test_detect.c -lm -o test/test_detect
./test/test_detect
```
`test/sdkconfig.h` is an empty stand-in for the file ESP-IDF normally
generates, so `config.h` compiles outside ESP-IDF.

---

## 6. Setup, build, flash, simulate

1. **Install ESP-IDF v6.0.x** via the *ESP-IDF* VS Code extension (or
   Espressif's EIM installer), following the ESP32-S3 Get Started guide.
2. **Simulator build**, then run it in Wokwi:
   ```
   idf.py -B build-sim -D SDKCONFIG=build-sim/sdkconfig \
          -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.sim" build
   ```
   - *VS Code (easiest):* install **Wokwi for VS Code**, get the free
     license (F1 → "Wokwi: Request a new License"), then F1 → "Wokwi: Start
     Simulator".
   - *wokwi.com in the browser:* create a new ESP32-S3 project and paste
     `diagram.json` into its diagram tab. Run
     `idf.py -B build-sim -D SDKCONFIG=build-sim/sdkconfig uf2`, then in
     the code editor press F1 → "Load HEX File and Start Simulation" and
     pick `build-sim/uf2.bin`.
   - *Trigger events in the sim:* set the MPU6050's acceleration high
     (above 1.8 g total for a shot, above 1.3 g for a touch), then turn any
     potentiometer past about 40%. Turn it back down before the next one.
3. **Real board:**
   ```
   idf.py -B build -D SDKCONFIG=build/sdkconfig \
          -D SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.defaults.feather" build
   idf.py -B build -D SDKCONFIG=build/sdkconfig -p PORT flash monitor
   ```
   PORT is `COMx` on Windows, `/dev/cu.usbmodem*` on Mac, or `/dev/ttyACM0`
   on Linux. If it won't connect, hold BOOT, tap RESET, release BOOT, flash,
   then press RESET.

## 7. Wiring (Feather ESP32-S3 #5323)

| Signal | Feather pin | GPIO | Notes |
|---|---|---|---|
| FSR instep / laces / outside / toe | D5 / D6 / D9 / D10 | 5 / 6 / 9 / 10 | Each FSR from 3V3 to its pin; a 10 kΩ from the pin to GND |
| IMU SDA / SCL | STEMMA QT | 3 / 4 | Port power = GPIO7 (handled in `board.c`) |
| SD SCK / MOSI / MISO / CS | SCK / MO / MI / D12 | 36 / 35 / 37 / 12 | Check against Adafruit's PrettyPins PDF |
| Status LED | D13 | 13 | Blinks on each touch/shot |

## 8. Who owns what (from the PDR roles)

| Files | Owner | First tasks |
|---|---|---|
| `acq.c`, `main.c` | Aamir | First hardware run: confirm `overruns = 0` and `max_loop` well under 1000 µs |
| `fsr.c` + wiring | Nimon | Check each zone's raw counts through the boot; choose the divider resistor |
| `imu.c` | Aamir + Nimon | Confirm WHO_AM_I = 0xE1 and I2C address 0x69 on the real breakout |
| `detect.c`, `test/` | Alex | Tune touch/shot thresholds from logged kicks; add a test per change |
| `logger.c`, `log_format.h`, `tools/` | Neel | Plot `samples.csv` with event markers; grow it into the app |
| `config.h`, `board.c` | Diego | Keep pins in sync with the PCB schematic |

## 9. References
- ESP-IDF Programming Guide, ESP32-S3: https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/
  The API pages used here are *ADC Oneshot*, *I2C*, *GPTimer*, *SD SPI Host*,
  *FAT Filesystem*, and *FreeRTOS*.
- ESP-IDF examples in `$IDF_PATH/examples/peripherals/`: `adc/oneshot_read`,
  `i2c/`, `timer_group/gptimer`, `storage/sd_card/sdspi`
- Adafruit ESP32-S3 Feather guide: https://learn.adafruit.com/adafruit-esp32-s3-feather
- ICM-20649 datasheet: https://invensense.tdk.com/?p=6996
- Adafruit_ICM20X library (reference init sequence): https://github.com/adafruit/Adafruit_ICM20X
- Wokwi + ESP-IDF: https://docs.wokwi.com/vscode/esp-idf

**Status:** the detector and file-format code is tested on a laptop. The
ESP-IDF build has not been compiled yet, so expect small fixes on the first
`idf.py build`.
