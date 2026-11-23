# Project 80120 – DIY Pallet Electric Go-Kart Firmware (ESP32-S3)

Firmware for an **ESP32-S3** microcontroller (running on the [RoboticsBrno Saturn](https://github.com/RoboticsBrno/Saturn) board) acting as the central Vehicle Control Unit (VCU) for a custom electric go-kart built on a wooden Europallet chassis.

The kart uses **four 500W hoverboard BLDC hub motors** with reflashed hoverboard control boards, **phase-shorting electromagnetic braking on the front motors**, a mechanical rack-and-pinion steering assembly, addressable LED lighting with dynamic turn signals and emergency strobes, relay-controlled power/braking stages, a Hall-effect speedometer, and a 20×4 I2C LCD dashboard.

---

## 🛠️ Hardware Overview

* **Chassis:** Modified wooden Europallet frame (~10 kg) with a mechanical rack-and-pinion steering assembly ([AliExpress Steering Assembly](https://www.aliexpress.com/item/1005006120546015.html)).
* **Drivetrain & Braking:**
  * **4× BLDC Hoverboard Hub Motors** (6.5" wheels) driven by reflashed hoverboard motherboards (unlocked and flashed via ST-Link & STM32CubeProgrammer following [this guide](https://www.youtube.com/watch?v=77bMH5Ql1Xo) [00:03:43]).
  * **Front-Motor Phase-Short Braking:** The front hub motors are used as dynamic/regenerative-style brakes by **shorting their motor phases** (controlled via the stepped brake relay stages), providing braking torque without relying solely on mechanical friction.
* **Main Controller:** [RoboticsBrno Saturn](https://github.com/RoboticsBrno/Saturn) (ESP32-S3).
* **Display:** 20×4 Character LCD over I2C (`0x3F`) with custom CGRAM large-font speed digits and status icons.
* **Lighting & Signals:**
  * **Front LED Strip (46 LEDs):** Low/high beams, dynamic turn signals, emergency strobes.
  * **Rear LED Strip (46 LEDs):** Tail lights, strobing brake lights, reverse lights, dynamic turn signals, emergency strobes.
  * **Side LED Strips (16 LEDs total: `0–7` right, `8–15` left):** Turn signals with incandescent filament warm-up/cool-down curve & side strobes.
  * **Onboard Status LED:** Boot/system status indicator.
* **Relays (Active-LOW):**
  * `1×` BLDC driver power pulse relay (`IO10`)
  * `1×` Acoustic / buzzer relay (`IO02`)
  * `6×` Stepped braking / front-motor phase-short relays (on display header GPIOs)

---

## 📌 Pinout & Wiring

> **Note:** PMOD header pins on the Saturn board are numbered from the GND side (e.g., `PMOD1_4` = `IO16`). All switches use `INPUT_PULLUP` (active-LOW), and all relays are active-LOW.

### Controls, Sensors & Bus
| Function | ESP32-S3 Pin | Details |
| :--- | :--- | :--- |
| **Throttle Pedal** | `IO17` | 12-bit ADC (`0–4095`), potentiometer |
| **Brake Pedal** | `IO18` | 12-bit ADC (`0–4095`), potentiometer |
| **Throttle PWM Out** | `IO16` (`PMOD1_4`) | 1 kHz, 8-bit (`0–255`), LEDC Ch. 0 (RC-filtered analog signal for hoverboard ECU) |
| **Hall Speed Sensor** | `IO21` | Rising-edge interrupt (`hallISR`), 15 pulses/rev |
| **I2C SDA / SCL** | `IO47` / `IO42` | 400 kHz bus for 20×4 LCD (`0x3F`) |

### Relays (Active-LOW)
| Function | ESP32-S3 Pin | Details |
| :--- | :--- | :--- |
| **BLDC Power Pulse** | `IO10` | ~300 ms pulse toggling the hoverboard motor driver state |
| **Acoustic / Buzzer Relay** | `IO02` | Boot beep, mode/light toggle feedback, and blinker clicker |
| **Brake / Phase-Short Relays (1–6)** | Display `D0–D3`, `CS`, `CK` | Stepped control (`6` → `0` active relays as brake % increases) driving front-motor phase shorting / braking stages |

### Switches & LEDs
* **Switches (`INPUT_PULLUP`, Active-LOW):** Forward/Reverse, Left/Right turn signals, Hazard lights, Lights OFF / Low / High beam, Emergency beacons.
* **Addressable LED Strips:** Front (46), Rear (46), Side (16), plus onboard status LED.

---

## ⚙️ Firmware Architecture

Single-threaded main loop (`~100 Hz`, paced with `delay(10)`, no FreeRTOS tasks) paired with one hardware interrupt (`hallISR`) for wheel speed.

### 1. Boot Sequence (`setup`)
1. Logs the last ESP32 reset reason (`poweron`, `brownout`, `panic`, etc.) and sets the onboard status LED to **orange**.
2. Configures GPIOs, sets throttle PWM to `0`, and sets all relays to `HIGH` (inactive).
3. Runs `loadingStart()` and `runOnSequence()` concurrently: renders **"Project 80120"** on the LCD with a 70-step progress bar and non-repeating randomized status messages while sweeping the front/rear LEDs and flashing the dynamic turn signals.
4. Shows `"Success!"`, sets the status LED to **green**, and switches to the driving dashboard.

### 2. Main Loop Execution Order
`Direction/Neutral` → `Switches` → `Throttle & Brake` → `BLDC Relay Control` → `Lighting` → `Speed Calculation` → `LCD Update`

* **Direction & Neutral (`masterSmerAManagement`):**
  * Handles **Forward**, **Reverse**, and **Neutral**.
  * Shifting to Neutral zeroes PWM immediately and delays BLDC driver cutoff by **500 ms** (`ZPOZDENI_NEUTRALU`) to avoid unnecessary power cycling during quick shifts.
* **Throttle, Front-Motor Braking & Safety (`zpracujPlynABrzdu`):**
  * **Throttle & Brake Blending:** Brake pedal input above `5%` scales down throttle by `(100 - brake%)` down to `0` at full braking.
  * **Front-Motor Phase-Short Braking:** Braking engages the front motors as electromagnetic brakes by shorting motor phases via the stepped relay bank (`map(brake, 0..100 → 6..0)`), working alongside the rear brake light strobe.
  * **Forward / Reverse PWM:** Forward maps `0–100%` linearly to `0–255` duty; Reverse applies a `5%` deadzone and maps `5–100%` to `81 → 63` duty to match the reflashed hoverboard ECU calibration.
  * **Emergency Reboot:** Holding **Hazards ON** + **Throttle ≥ 95%** + **Brake ≥ 95%** for **> 2 s** zeroes PWM and calls `ESP.restart()`.
* **BLDC Driver Pulse (`kontrolaBLDCDriveru`):**
  * Sends a ~300 ms `LOW` pulse on `IO10` (with a 2 s minimum interval between pulses and PWM zeroed during switching) to toggle the hoverboard driver like a push-button.
* **Lighting (`aktualizujSvetla`):**
  * **Dynamic Blinkers:** 700 ms cycle (`450 ms` outward sweep across 10-LED zones + `250 ms` off), synced with the acoustic relay and LCD indicators; side strips use a filament-like curve (`60 ms` ramp-up, `220 ms` fade-out, quadratic brightness).
  * **Emergency Beacons:** 600 ms alternating red (left) / blue (right) triple-flash strobe across front, rear, and side LEDs with smooth transition back to normal lighting.
  * **Rear Strip Priority:** `Beacons` → `Reverse (White)` → `Brake (60 ms Red Strobe)` → `Tail Lights`.
* **Speedometer (`aktualizujRychlost`):**
  * Reads pulse intervals on `IO21` inside a `portMUX` critical section (ignoring `< 1000 µs` noise spikes), computes speed from a 6.5" wheel (`~0.52 m`, `3.47 cm/pulse`), smooths with an EMA filter (`α = 0.3`), and resets to `0 km/h` after `0.5 s` without pulses.
* **20×4 LCD Dashboard:**
  * Shares CGRAM slots between the boot screen and the dashboard (3×2 large speed digits + blinker, light, and neutral/park icons) and updates only changed cells (`lcdPrepisDiff`) every `500 ms` to keep I2C overhead low.

---

## 📊 Dashboard Layout (20×4 LCD)

```text
+--------------------+
| [0][0]  <  L  P  > |  <- Large 2-row speed digits + Blinkers, Lights, Neutral/Park
| [0][0]  km/h  FWD  |  <- Unit + Drive Mode (FWD / RWD / blank in Neutral)
| TRK 36.0V PAL 12.4V|  <- Traction & Auxiliary battery voltages (telemetry hooks)
| I  12.5A  DOJ 15km |  <- Current draw & Estimated range (telemetry hooks)
+--------------------+
