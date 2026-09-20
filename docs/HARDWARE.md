# Hardware Additions — LSM6DS3 IMU & Telemetry

Everything that must change on the physical robot before the planned software
(gyro-based yaw feedback, PID heading control, wireless telemetry) will run.

Target board: **ESP32 DOIT DevKit v1** (ESP32-WROOM-32, 30-pin header).
Firmware: ESP-IDF 6.1 via PlatformIO.

---

## 1. Current pin allocation (already wired — do not disturb)

| GPIO | Function | Direction | Notes |
|-----:|----------|-----------|-------|
| 2  | Status LED | Out | On-board blue LED. Strapping pin. |
| 14 | Motor A PWM | Out | LEDC ch0, 20 kHz, 8-bit |
| 18 | Right ToF `XSHUT` | Out | Releases right VL53L0X, re-addressed to `0x30` |
| 19 | Left ToF `XSHUT` | Out | Releases left VL53L0X, re-addressed to `0x31` |
| 21 | I²C0 SDA | Bidir | 400 kHz |
| 22 | I²C0 SCL | Out | 400 kHz |
| 23 | Start button | In | Currently `GPIO_FLOATING` |
| 25 | Motor B IN1 | Out | |
| 26 | Motor A IN2 | Out | |
| 27 | Motor A IN1 | Out | |
| 32 | Motor B PWM | Out | LEDC ch1 |
| 33 | Motor B IN2 | Out | |

---

## 2. NEW: LSM6DS3 IMU wiring

The IMU joins the **existing I²C bus**. No new bus, no bit-banging — two signal
wires, power, and one interrupt line.

| LSM6DS3 pin | Connect to | Required? | Why |
|-------------|-----------|:---------:|-----|
| `VDD` / `VIN` | **3V3** | Yes | Part is 1.71–3.6 V. If your breakout has a regulator and level shifters (e.g. SparkFun SEN-13339) 5 V also works, but 3V3 keeps logic levels native. |
| `GND` | **GND** | Yes | Must share ground with the ESP32 **and** the motor supply. |
| `SCL` / `SPC` | **GPIO 22** | Yes | Shared with both VL53L0X sensors. |
| `SDA` / `SDI` | **GPIO 21** | Yes | Shared. |
| `CS` | **3V3** | **Yes — critical** | `CS` high selects **I²C mode**, low selects SPI. Most breakouts pull this up already, but a bare module sits in SPI mode and appears completely dead on the bus. **Check this first if the IMU does not enumerate.** |
| `SA0` / `SDO` | **GND** *(recommended)* | Yes | Sets the I²C address. GND → **`0x6A`**, 3V3 → `0x6B`. Either is conflict-free; pick one and hard-code it. SparkFun's breakout defaults to `0x6B`, Adafruit's to `0x6A`. |
| `INT1` | **GPIO 4** | Recommended | Data-ready / FIFO-threshold interrupt. See §3. |
| `INT2` | *leave unconnected* | No | Reserve **GPIO 16** if a second interrupt source is ever needed. |

### Resulting I²C bus map

| Address | Device |
|--------:|--------|
| `0x29` | VL53L0X default — transient, during boot re-addressing only |
| `0x30` | Right VL53L0X |
| `0x31` | Left VL53L0X |
| `0x6A` | **LSM6DS3 (new)** |

No collisions. Verify the part with `WHO_AM_I` (register `0x0F`):
**`0x69`** = LSM6DS3, **`0x6A`** = LSM6DS3TR-C / LSM6DS3-C. (The TR-C's `WHO_AM_I`
value happens to equal the other variant's bus address — unrelated, don't confuse
the two.)

---

## 3. Why GPIO 4 for INT1

GPIO 4 is the best remaining choice on this board:

- **Not a strapping pin.** The ESP32's strapping pins are GPIO 0, 2, 5, 12 (MTDI) and 15 (MTDO). GPIO 4 is none of them, so it has no boot-time level requirement.
- **No boot-time glitching.** Unlike GPIO 5 and 15, it does not emit a PWM burst at reset.
- **Full capability** — input, interrupt-capable, internal pull-up/pull-down.
- **Spending an ADC2 pin costs nothing here.** GPIO 4 is on ADC2, and **ADC2 is unusable while WiFi is active** — which it will be, for telemetry. It has no future as an analog input anyway, so it is better spent on a digital interrupt than left idle.
- Broken out on the 30-pin DOIT header (silkscreen `D4`).

**Is INT1 strictly necessary?** No — you can poll `STATUS_REG`. But the interrupt
gives you a hardware-timed sample edge, which is exactly what the PID loop needs:
a *constant* dt. Polling from a task that also talks to the ToF sensors gives a
jittery dt, and jittery dt poisons the derivative term. **Wire it.**

---

## 4. Mounting the IMU (this matters more than the wiring)

A gyro wired perfectly but mounted badly produces useless yaw data.

1. **Z axis must be vertical** — pointing up or down, perpendicular to the ring floor. Yaw rate comes from the gyro's Z channel. Mounted on its side you would be integrating pitch or roll instead.
2. **Mount as close to the robot's centre of rotation as practical.** Off-centre, the accelerometer picks up centripetal acceleration during turns (`a = ω²r`), which contaminates the forward-acceleration reading you want for impact detection. The gyro is insensitive to position; the accelerometer is not.
3. **Rigid mount — no foam, no double-sided tape alone.** Drivetrain and gearbox vibration aliases straight into the gyro passband. Bolt or screw it to the chassis. If vibration is still visible in the data, the fix is the LSM6DS3's built-in low-pass filter, not a softer mount.
4. **Record the orientation in code** as a sign/axis-mapping constant, so "positive yaw rate = counter-clockwise" is written down once and never re-derived at 2 a.m.
5. Keep it clear of the motors and their leads — not for magnetic reasons (there is no magnetometer) but to keep the I²C wires out of the PWM switching noise.

---

## 5. Power — the most likely thing to bite you

Highest-risk item on this list, because the failure mode looks like a software bug.

- **Separate the motor supply from the ESP32 supply**, sharing only ground. When the bot is shoving the opponent the motors *stall* — that is the entire point of a sumo match — and stall current is several times running current. On a shared rail this browns out the 3V3 line and resets the ESP32 mid-match.
- **Bulk capacitance across the motor supply:** 470–1000 µF electrolytic, as close to the motor driver as possible.
- **Decoupling at the IMU:** 100 nF ceramic across `VDD`–`GND` at the module. The LSM6DS3 draws only ~1.25 mA in combined accel+gyro mode, so this is about noise, not current.
- **Budget for WiFi.** The radio draws ~250 mA in TX bursts. The DOIT board's AMS1117 can supply that but runs hot — make sure whatever feeds `VIN`/`5V` can deliver ≥ 1 A with headroom.
- If random reboots appear once telemetry is enabled, read `esp_reset_reason()` before suspecting the code. `ESP_RST_BROWNOUT` names the problem immediately.

---

## 6. I²C bus integrity

The firmware currently enables the ESP32's **internal** pull-ups
(`flags.enable_internal_pullup = true` in `configure_sensor_bus`, `src/main.c`).
Those are weak — roughly 45 kΩ — and already marginal at 400 kHz. You are about to
have **three** devices on the bus plus the extra wiring capacitance.

- Fit **external 4.7 kΩ pull-ups** to 3V3 on SDA and SCL, then disable the internal ones.
- **Count what is already there first.** Most VL53L0X and LSM6DS3 breakouts ship with their own on-board pull-ups, typically 4.7 kΩ or 10 kΩ. Three breakouts at 4.7 kΩ in parallel is ~1.6 kΩ, which is *too strong* — the bus may fail to pull down to a valid logic low. Measure SDA-to-3V3 resistance with the board unpowered and remove redundant resistors (or cut the solder jumpers) until the total sits in the 2.2–4.7 kΩ range.
- Keep the I²C runs short, and twist each signal with a ground return if you can.

---

## 7. Config changes that are really hardware constraints

These live in `sdkconfig` but exist because of the physical board:

| Setting | Current | Change to | Why |
|---------|---------|-----------|-----|
| `CONFIG_ESPTOOLPY_FLASHSIZE` | `"2MB"` | `"4MB"` | The WROOM-32 on this board has 4 MB. You are currently addressing half of it. |
| `CONFIG_PARTITION_TABLE_SINGLE_APP` | `y` | `SINGLE_APP_LARGE` | At 2 MB single-app the app partition is **1 MB**. The WiFi stack, HTTP server and WebSocket support will push the binary near that ceiling, and the link fails late and confusingly. |
| `CONFIG_FREERTOS_HZ` | `100` | `1000` | 10 ms tick granularity means `vTaskDelay(pdMS_TO_TICKS(2))` rounds to **zero ticks** — a bare yield, not a delay. This already affects the VL53L0X driver's busy-wait loops, and makes an accurate 100 Hz+ IMU sample rate impossible to schedule. |
| `CONFIG_ESP_CONSOLE_UART_BAUDRATE` | `115200` | `921600` | Only if you use serial CSV logging — 115200 cannot carry 100 Hz of telemetry. |

---

## 8. Telemetry — hardware implications

No new components required; the ESP32-WROOM-32 has the radio on-module. Two
physical points only:

- **Antenna clearance.** The WROOM's PCB antenna is the exposed keep-out zone at the end of the module. Do not mount it flat against a metal chassis plate, do not run motor leads across it, and do not bury it under the ToF wiring. A metal sumo chassis is an effective RF shield — overhang the module past the chassis edge if possible.
- **Nothing else.** If you later want guaranteed-complete logs rather than streamed ones, a microSD-over-SPI module needs 4 pins; **GPIO 5, 13, 16, 17** are reserved below and would cover it.

---

## 9. Pins still free after these additions

| GPIO | Status | Suitable for |
|-----:|--------|--------------|
| 5  | Free — strapping pin, emits PWM at boot, internal pull-up | Outputs tolerant of a boot glitch |
| 13 | **Free and clean** | Anything |
| 15 | Free — strapping pin (MTDO) | Inputs that idle high |
| 16 | **Free and clean** — reserved for IMU `INT2` or SD | Anything¹ |
| 17 | **Free and clean** — reserved for SD | Anything¹ |
| 34, 35 | Input only, no pull-ups, **ADC1** | Analog line/edge sensors |
| 36 (VP), 39 (VN) | Input only, no pull-ups, **ADC1** | Analog line/edge sensors |
| 12 | Avoid — strapping (MTDI), must be low at boot | — |
| 1, 3 | Avoid — UART0 console and flashing | — |
| 0, 6–11 | Not available — flash, or not broken out | — |

¹ GPIO 16 and 17 are consumed by PSRAM on WROVER modules. This board is a
**WROOM-32 with no PSRAM**, so they are genuinely free — but if the board is ever
swapped for a WROVER, they are the first thing to break.

> **Ring-edge sensors:** a sumo bot normally needs downward IR reflectance sensors
> so it does not drive itself out of the ring. When you add them they **must** go on
> **GPIO 34, 35, 36, 39** — the only ADC1 pins left, and ADC2 is unusable whenever
> WiFi is on. Plan for this before spending those four pins on anything else.

---

## 10. Build checklist

- [ ] LSM6DS3 `CS` tied to 3V3 (I²C mode) — verify with a meter, not by eye
- [ ] `SA0` tied to GND → address `0x6A` (or 3V3 → `0x6B`; record which)
- [ ] SDA → GPIO 21, SCL → GPIO 22
- [ ] `INT1` → GPIO 4
- [ ] 100 nF decoupling cap at IMU `VDD`
- [ ] I²C pull-up total measured, within 2.2–4.7 kΩ
- [ ] IMU Z axis vertical, near centre of rotation, rigidly mounted
- [ ] Motor supply separate from ESP32 supply, grounds common
- [ ] 470–1000 µF bulk cap across motor supply
- [ ] WROOM antenna not against metal or under a wire loom
- [ ] `WHO_AM_I` (`0x0F`) reads `0x69` or `0x6A` before any control code is written
- [ ] Bus scan shows `0x30`, `0x31` and `0x6A` — all three, together
