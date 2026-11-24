# GridGuard: Master Engineering & Codex Implementation Specification

**Target Controller:** Raspberry Pi Pico 2 W (RP2350A, Cortex-M33)
**Firmware RTOS:** Zephyr RTOS (`rpi_pico2/rp2350a/m33/w`)
**Grid Standard:** India / IEC Single-Phase Nominal (`230 V RMS`, `50 Hz`)
**Transport:** Native USB CDC-ACM (`/dev/ttyACM0` on Linux, `COMx` on Windows) with COBS + CRC-16-CCITT binary framing
**Host Backend:** Python 3.11+ / FastAPI / pyserial / WebSocket
**Host Frontend:** React 18 / TypeScript / Vite / Tailwind CSS / Apache ECharts
**Hardware Scope:** Internal Physics Plant HAL + Low-Voltage DC Physical Protection (MOSFET/LED desk mode + Lab Relay mode) + Independent ADC Disconnect Verification

---

## 0. Codex Anti-Hallucination Contract (Read First)

This document is an exact, self-contained build specification for an autonomous coding agent. Follow every rule below literally:

1. **Zero Prohibited Hardware/Software:**
   * **DO NOT** include ZMPT101B, ACS712, SCT-013, or any external AC/mains voltage/current sensor driver.
   * **DO NOT** implement a PWM-to-RC-to-ADC analog loopback. All grid waveforms are synthesized in the Pico firmware plant module and injected into the exact same Sensor HAL ring buffer that an ADC driver uses.
   * **DO NOT** require an external USB-to-UART adapter. Use Zephyr's native USB CDC-ACM (`zephyr,cdc-acm-uart`).
   * **DO NOT** use `DT_ALIAS(led0)` on `rpi_pico2/rp2350a/m33/w`. On Pico 2 W, the onboard LED is wired to the CYW43439 Wi-Fi chip, not RP2350 GP25 ([Zephyr Debugprobe](https://github.com/vjardin/zephyr-debugprobe)). Drive an external status LED on `GP15`.
   * **DO NOT** enable internal pull-down resistors on RP2350 GPIO inputs. Early A2-stepping RP2350 silicon exhibits erratum **RP2350-E9** (~120 uA leakage latching weakly pulled-down inputs near ~2.2 V) ([Wikipedia](https://en.wikipedia.org/wiki/RP2350), [Tinkered](https://www.tinkered.ai/components/raspberry-pi-pico-2)). Use internal pull-ups (active-low switch to GND) or external pull-downs `<= 8.2 kOhm`.
   * **DO NOT** use TensorFlow Lite Micro, CMSIS-NN, or Streamlit. NILM inference must be 100% pure C99 using weights exported by `scripts/train_nilm.py`.
2. **Zero Stubs or Fake Success:**
   * Every C function, Python parser, and React component must be fully implemented. No `TODO`, no `pass`, no hardcoded dummy telemetry in live mode.
   * Never report `PHYSICAL_DISCONNECT_CONFIRMED` when commanding the relay GPIO low. Confirmation requires the feedback checks in Section 3.4 to pass.
3. **Single Source of Truth for Binary Framing:**
   * C packed structs in `firmware/lib/gridguard_core/include/gg_protocol.h` and Python `struct` unpackers in `host/backend/app/protocol.py` must match byte-for-byte (little-endian, explicit fixed-width types, zero implicit compiler padding).
4. **Build & Test Gate:**
   * Before finishing, verify: (a) pure-C core unit tests pass via CMake/CTest, (b) `scripts/train_nilm.py` runs and generates `nilm_weights.h` with >= 95% synthetic test accuracy, (c) FastAPI `pytest` suite passes including COBS round-trip and replay tests, (d) React `npm run build` succeeds with zero TypeScript errors, and (e) Zephyr build configuration is valid for `rpi_pico2/rp2350a/m33/w`.

---

## 1. System Architecture & Safety Boundary

GridGuard separates electrical grid simulation/metrology from low-voltage physical disconnect verification while keeping the entire real-time protection and NILM pipeline executing on the Pico 2 W:

```text
+===========================================================================+
|                  RASPBERRY PI PICO 2 W (ZEPHYR RTOS)                      |
|                                                                           |
|  [Physics Plant 3.2 kHz] --(12-bit quantized frames)--> [Sensor HAL Buf]  |
|                                                               |           |
|                                                               v           |
|  [Metrology & Goertzel DSP (50 Hz cycles)] -------------------+           |
|         |                                     |                           |
|         v                                     v                           |
|  [IEC 60255 Protection Engine]        [Event-Driven Physics NILM]         |
|         |                               (CUSUM + 10-feat MLP +            |
|         v                                Mahalanobis Unknown Gate)        |
|  [10-State Relay FSM]                                                     |
|         |--> GP14 (MOSFET / Relay Driver)                                 |
|         |<-- GP26 / ADC0 (Source-Side Divider)                            |
|         |<-- GP27 / ADC1 (Load-Side Divider)                              |
|         |<-- GP16 (Optional Aux Contact, Active-Low Pull-Up)              |
|         |                                                                 |
|  [Zephyr NVS Flash Fault Log] <------ (Trip / Verify / Reset Events)      |
|  [USB CDC-ACM COBS + CRC16] <===========================================> |
+===========================================================================+
                                      | USB Cable (/dev/ttyACM0 or COMx)
                                      v
+===========================================================================+
|                   HOST (ENDEAVOUROS LINUX / WINDOWS 11)                   |
|  [FastAPI Serial Bridge + Replay Engine] <--WebSocket--> [React + ECharts]|
+===========================================================================+
```

**Mains Prohibition:** Never connect household AC mains (230 V AC) to this prototype. The physical switching stage operates strictly on an isolated, current-limited **3.3 V to 12 V DC** supply with a known low-voltage DC load.

---

## 2. Hardware Pin Map & Dual-Stage Physical Wiring

Because the Pico 2 W exposes exactly **3 external ADC pins** on its header (`GP26/ADC0`, `GP27/ADC1`, `GP28/ADC2`) while `GP29/ADC3` is wired internally to `VSYS/3` ([Robocraze](https://robocraze.com/blogs/post/latest-raspberry-pi-pico-2-features-and-improvements), [Tinkered](https://www.tinkered.ai/components/raspberry-pi-pico-2)), eliminating the PWM-RC loopback leaves all 3 ADC pins available with zero conflict.

### 2.1 Pin Assignment Table

| Pico 2 W Pin | Signal Name | Zephyr Device / Role | Electrical Notes |
| :--- | :--- | :--- | :--- |
| GP14 (Pin 19) | `RELAY_DRV_OUT` | GPIO Output (default LOW at boot) | Drives logic-level N-MOSFET gate (via 220 Ohm series + 10 kOhm gate-to-GND pulldown) or 3.3 V relay module IN |
| GP15 (Pin 20) | `STATUS_LED_OUT` | GPIO Output | External breadboard status LED + 330 Ohm resistor to GND (replaces unavailable RP2350 onboard LED) |
| GP16 (Pin 21) | `AUX_CONTACT_IN` | GPIO Input, `GPIO_PULL_UP` | Optional mechanically linked relay auxiliary contact wired to GND (active-low; avoids RP2350-E9 pull-down errata) |
| GP26 (Pin 31) | `ADC_SOURCE` | `adc` Channel 0 (12-bit, 0-3.3 V) | Source-side voltage divider output (strictly <= 3.3 V; **not 5 V tolerant**) |
| GP27 (Pin 32) | `ADC_LOAD` | `adc` Channel 1 (12-bit, 0-3.3 V) | Load-side voltage divider output (strictly <= 3.3 V; **not 5 V tolerant**) |
| AGND / GND | `GND` | Common Reference | Shared ground between Pico 2 W and low-voltage DC supply negative |
| USB Micro-B | `USB_CDC` | `zephyr_udc0` / `cdc_acm_uart0` | Power, UF2 flashing, and binary COBS telemetry/command transport |

### 2.2 Stage A: Tonight's Breadboard Test Setup (MOSFET + LED Load)

Use parts already on the desk (Pico 2 W, breadboard, logic-level N-MOSFET, LED, resistors) to verify the full physical disconnect loop immediately:

* **Supply:** Pico `VBUS` (5 V) or external 5 V DC pack.
* **High-Side/Load Check Note:** For an N-MOSFET low-side switch (`5 V -> R_load (330 Ohm) -> LED -> MOSFET Drain -> Source -> GND`), the Drain node is LOW (~0.1 V) when ON and HIGH (~5 V) when OFF. To keep polarity identical to a high-side NO relay (`LOAD_NODE = HIGH` when energized, `0 V` when disconnected), either:
  * **Option 1 (Recommended for tonight):** Use `VIRTUAL_FEEDBACK` mode first to validate firmware/dashboard, or wire a **P-MOSFET high-side switch** / **NPN + PNP pair** so `LOAD+` drops to `0 V` when tripped.
  * **Option 2 (N-MOSFET low-side desk mode):** Set `CONFIG_GRIDGUARD_LOWSIDE_INVERT_LOAD_ADC=y` (or runtime `CMD_SET_FEEDBACK_MODE` = `MODE_LOWSIDE_MOSFET`), which maps `V_load_eff = V_source - V_drain` so the exact same disconnect verification algorithm runs on an N-MOSFET + LED breadboard!

### 2.3 Stage B: Lab Relay Setup (5 V - 12 V DC Physical Load)

```text
+V_DC (5-12 V, current-limited) --[FUSE]--+--> [RELAY COM] ---(NO)--- [RELAY NO] --+--> [DC LOAD] --> GND
                                          |                                         |
                                    [R_TOP_S]                                 [R_TOP_L]      [R_BLEED]
                                          |-----> GP26 (ADC0)                       |-----> GP27 (ADC1)
                                    [R_BOT_S]                                 [R_BOT_L]         |
                                          |                                         |           v
                                         GND                                       GND         GND
```

**Divider & Protection Sizing (Strict 3.3 V ADC Ceiling):**
GP26 and GP27 have internal clamping diodes to the 3.3 V rail and are **not 5 V tolerant** ([Tinkered](https://www.tinkered.ai/components/raspberry-pi-pico-2)). Size dividers for $V_{\max} = 13.2\text{ V}$ (12 V nominal + 10% tolerance) so $V_{\text{ADC}} \le 3.0\text{ V}$:

$$V_{\text{ADC}} = V_{\text{node}} \cdot \frac{R_{\text{bot}}}{R_{\text{top}} + R_{\text{bot}}}$$

* **Recommended E24 values for 5-12 V range:** $R_{\text{top}} = 33\text{ k}\Omega$, $R_{\text{bot}} = 10\text{ k}\Omega$ (ratio $K_{\text{div}} = \frac{10}{43} = 0.23256$, scale factor $S_{\text{div}} = 4.30$).
  * At $V_{\text{node}} = 12.0\text{ V} \implies V_{\text{ADC}} = 2.79\text{ V}$ (safe margin below 3.3 V).
  * At $V_{\text{node}} = 5.0\text{ V} \implies V_{\text{ADC}} = 1.16\text{ V}$ (~1440 ADC counts out of 4095, high SNR).
  * Effective source impedance $R_{\text{th}} = R_{\text{top}} \parallel R_{\text{bot}} \approx 7.67\text{ k}\Omega$ (well within RP2350 SAR ADC settling limits; add a `10 nF` ceramic cap from each ADC pin to AGND for anti-aliasing and sampling-cap charge reservoir).
* **Bleeder Resistor ($R_{\text{bleed}}$):** Connect $R_{\text{bleed}} = 1\text{ k}\Omega$ ($\ge 0.25\text{ W}$) from `LOAD+` to GND. At 12 V, $I_{\text{bleed}} = 12\text{ mA}$ and $P_{\text{bleed}} = \frac{12^2}{1000} = 0.144\text{ W}$, ensuring `LOAD+` discharges to `< 0.1 V` within milliseconds when the NO contact opens.
* **Coil Driver (if using bare relay):** Logic-level N-MOSFET (e.g., 2N7000/IRLZ44N/BS170 rated above coil current) + `1N4007`/`1N4148` flyback diode across relay coil (cathode to `+V_coil`, anode to MOSFET drain) + `220 Ohm` gate resistor + `10 kOhm` gate-to-GND pulldown.

---

## 3. Mathematical Theory & Paper-Derived Algorithms

All algorithms in this section live in `firmware/lib/gridguard_core/` as pure, portable C99 (`float` single-precision utilizing the RP2350 Cortex-M33 FPU).

### 3.1 Physics-Based 50 Hz Grid Plant Simulator

* **Sampling Frequency:** $f_s = 3200\text{ Hz}$
* **Nominal Fundamental:** $f_0 = 50.0\text{ Hz}$
* **Samples per Nominal Cycle:** $N = \frac{f_s}{f_0} = 64$ samples/cycle (exact integer power of 2, zero spectral leakage at 50 Hz, Nyquist = 1600 Hz covering up to the 31st harmonic).
* **Nominal Voltage:** $V_{\text{nom}} = 230.0\text{ V}_{\text{rms}}$ ($V_{\text{peak}} = 230\sqrt{2} \approx 325.27\text{ V}$).

At discrete sample index $n$ with instantaneous phase accumulator $\theta[n] = \theta[n-1] + 2\pi \frac{f_{\text{grid}}[n]}{f_s} \pmod{2\pi}$:

$$v[n] = \sqrt{2} V_{\text{grid}}[n] \left(\sin(\theta[n]) + 0.015\sin(3\theta[n] + 0.2) + 0.010\sin(5\theta[n] - 0.1)\right) + \eta_v[n]$$

For each active appliance $a \in \{1, 2, 3, 4\}$ switched on at time $t_{0,a}$ with elapsed time $\Delta t_a = t - t_{0,a} \ge 0$, its current contribution includes an exponential inrush envelope plus odd harmonics:

$$i_a[n] = \left(1 + (\kappa_a - 1) e^{-\Delta t_a / \tau_a}\right) \sum_{h \in \{1,3,5,7,9\}} \sqrt{2} I_{a,h} \sin(h\theta[n] - \phi_{a,h})$$

Total plant current plus optional fault injection $i_{\text{fault}}[n]$ and Gaussian measurement noise $\eta_i[n]$:

$$i[n] = \sum_{a \in \text{ON}} i_a[n] + i_{\text{fault}}[n] + \eta_i[n]$$

**12-Bit Sensor Quantization Model (Simulated ADC HAL):**
Before entering metrology, $v[n]$ and $i[n]$ are scaled to a virtual 12-bit bipolar-biased ADC (`V_FS = ±400 V`, `I_FS = ±40 A`, mid-code `2048`):

$$c_v[n] = \text{clamp}_{[0,4095]}\left(\left\lfloor 2048 + \frac{v[n]}{400.0} \cdot 2047 \right\rceil\right), \quad c_i[n] = \text{clamp}_{[0,4095]}\left(\left\lfloor 2048 + \frac{i[n]}{40.0} \cdot 2047 \right\rceil\right)$$

Metrology receives **only** $(c_v[n], c_i[n])$ via the HAL interface and converts back to physical engineering units.

**Appliance Signature Parameters (India 230 V / 50 Hz):**

| ID | Appliance | $P_{\text{nom}}$ (W) | $I_1$ (A rms) | $\phi_1$ (deg) | $I_3/I_1$ | $I_5/I_1$ | $I_7/I_1$ | $I_9/I_1$ | Inrush $\kappa_a$ | Decay $\tau_a$ (s) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| 0 | Resistive Kettle / Heater | 1000 W | 4.35 A | 0.0 deg | 0.00 | 0.00 | 0.00 | 0.00 | 1.05 | 0.05 s |
| 1 | Ceiling Fan (Induction) | 75 W | 0.41 A | 37.0 deg | 0.04 | 0.02 | 0.01 | 0.00 | 2.20 | 0.25 s |
| 2 | Laptop / LED SMPS | 65 W | 0.38 A | -12.0 deg | 0.72 | 0.48 | 0.26 | 0.12 | 1.80 | 0.04 s |
| 3 | Refrigerator Compressor | 250 W | 1.36 A | 36.9 deg | 0.08 | 0.05 | 0.02 | 0.01 | 4.50 | 0.35 s |

### 3.2 Non-Sinusoidal Metrology & Goertzel Harmonic Engine

Derived from **IEEE Std 1459-2010 / 1459-2025** ([IEEE SA](https://standards.ieee.org/ieee/1459/7578/)) and **Goertzel (1958)** ([Wikipedia](https://en.wikipedia.org/wiki/Goertzel_algorithm)), computed over every $N = 64$-sample window (50 Hz update rate = 20 ms per window):

1. **DC Offset Removal:**
   $$\bar{v} = \frac{1}{N}\sum_{n=0}^{N-1} v[n], \quad \tilde{v}[n] = v[n] - \bar{v}, \quad \tilde{i}[n] = i[n] - \bar{i}$$
2. **True RMS Voltage & Current:**
   $$V_{\text{rms}} = \sqrt{\frac{1}{N}\sum_{n=0}^{N-1} \tilde{v}[n]^2}, \quad I_{\text{rms}} = \sqrt{\frac{1}{N}\sum_{n=0}^{N-1} \tilde{i}[n]^2}$$
3. **Total Active Power ($P$), Apparent Power ($S$), and True Power Factor ($\text{PF}$):**
   $$P = \frac{1}{N}\sum_{n=0}^{N-1} \tilde{v}[n]\tilde{i}[n], \quad S = V_{\text{rms}} I_{\text{rms}}, \quad \text{PF} = \frac{P}{\max(S, \epsilon)}$$
4. **Goertzel Single-Bin Harmonic Extraction ($h \in \{1, 3, 5, 7, 9, 11\}$):**
   For harmonic index $k = h$ over $N = 64$ samples, define $\omega_k = \frac{2\pi k}{N}$ and $c_k = 2\cos(\omega_k)$. Initialize $s[-1] = 0, s[-2] = 0$ and iterate for $n = 0 \dots N-1$:
   $$s[n] = x[n] + c_k s[n-1] - s[n-2]$$
   Extract complex DFT phasor at window end:
   $$\text{Re}\{X_k\} = s[N-1] - \cos(\omega_k)s[N-2], \quad \text{Im}\{X_k\} = \sin(\omega_k)s[N-2]$$
   Convert to RMS harmonic magnitude:
   $$X_{k,\text{rms}} = \frac{\sqrt{2}}{N}\sqrt{\text{Re}\{X_k\}^2 + \text{Im}\{X_k\}^2}$$
5. **IEEE 1459 Fundamental Active ($P_1$), Reactive ($Q_1$), Non-Fundamental Distortion ($D$), and THD:**
   Using fundamental phasors $V_1 = V_{1r} + jV_{1i}$ and $I_1 = I_{1r} + jI_{1i}$ (scaled by $\frac{\sqrt{2}}{N}$):
   $$P_1 = V_{1r}I_{1r} + V_{1i}I_{1i}, \quad Q_1 = V_{1i}I_{1r} - V_{1r}I_{1i}$$
   $$D = \sqrt{\max(0, S^2 - P_1^2 - Q_1^2)}, \quad \text{THD}_I = \frac{\sqrt{\sum_{h \in \{3,5,7,9,11\}} I_{h,\text{rms}}^2}}{\max(I_{1,\text{rms}}, \epsilon)} \times 100\%$$
6. **Sub-Sample Linear-Interpolated Zero-Crossing Frequency Estimator:**
   Detect rising zero-crossings ($\tilde{v}[n-1] < 0 \le \tilde{v}[n]$) with hysteresis band $\pm 15\text{ V}$ to reject noise. Fractional crossing index:
   $$n_{\text{zc}} = (n - 1) + \frac{-\tilde{v}[n-1]}{\tilde{v}[n] - \tilde{v}[n-1]}$$
   Instantaneous frequency $f_{\text{est}} = \frac{f_s}{n_{\text{zc}}^{(m)} - n_{\text{zc}}^{(m-1)}}$, smoothed with a 5-cycle median/EMA filter ($\alpha = 0.25$).
7. **Energy Accumulator (Wh):**
   $$E_{\text{Wh}}[m] = E_{\text{Wh}}[m-1] + P[m] \cdot \frac{\Delta t_{\text{win}}}{3600}, \quad \Delta t_{\text{win}} = 0.02\text{ s}$$

### 3.3 IEC 60255-151 IDMT Protection & Fault Detection Engine

Derived from **IEC 60255-151:2009** ([GridAPM](https://www.gridapm.com/blog/iec-60255-151-inverse-time-curve-constants/), [CalcEngineer](https://calcengineer.com/electrical/relay-coordination-curve/)):

For any overcurrent ratio $M = \frac{I_{\text{rms}}}{I_p} > 1.05$ (pickup current $I_p = 8.0\text{ A}$, time multiplier $\text{TMS} = 0.15$):

$$t_{\text{op}}(M) = \text{TMS} \cdot \frac{k}{M^{\alpha} - 1}$$

| Curve Type | $k$ | $\alpha$ | Role in GridGuard |
| :--- | :--- | :--- | :--- |
| Standard Inverse (SI) | 0.14 | 0.02 | Configurable option |
| Very Inverse (VI) | 13.5 | 1.0 | Configurable option |
| **Extremely Inverse (EI, Default)** | **80.0** | **2.0** | Matches $I^2t$ thermal fuse/cable heating |
| Long-Time Inverse (LTI) | 120.0 | 1.0 | Configurable option |

**Discrete Thermal Integrator (Executed Every 20 ms Cycle):**
To handle time-varying overloads accurately instead of a naive timer, maintain normalized thermal state $\Theta[m] \in [0, 1.0]$:
* When $M[m] > 1.05$: $\Theta[m] = \Theta[m-1] + \frac{0.02}{t_{\text{op}}(M[m])}$. Trip `FAULT_IDMT_OVERLOAD` when $\Theta[m] \ge 1.0$.
* When $M[m] \le 1.05$ (thermal cooling): $\Theta[m] = \max\left(0, \Theta[m-1] - \frac{0.02}{\tau_{\text{cool}}}\right)$ with $\tau_{\text{cool}} = 10.0\text{ s}$.

**Complete Protection Threshold Matrix (230 V / 50 Hz):**

| Fault Code | Enum Name | Condition | Persistence | Action |
| :--- | :--- | :--- | :--- | :--- |
| `0x01` | `FAULT_SHORT_CIRCUIT` | $I_{\text{rms}} \ge 25.0\text{ A}$ | 1 cycle (20 ms) | Immediate Trip |
| `0x02` | `FAULT_OVERCURRENT` | $I_{\text{rms}} \ge 12.0\text{ A}$ | 3 cycles (60 ms) | Immediate Trip |
| `0x03` | `FAULT_IDMT_OVERLOAD` | $\Theta[m] \ge 1.0$ ($I_p = 8.0\text{ A}$, EI curve) | Integrated | Inverse-Time Trip |
| `0x04` | `FAULT_UNDERVOLTAGE` | $V_{\text{rms}} < 195.0\text{ V}$ (-15%) | 10 cycles (200 ms) | Trip |
| `0x05` | `FAULT_OVERVOLTAGE` | $V_{\text{rms}} > 255.0\text{ V}$ (+11%) | 10 cycles (200 ms) | Trip |
| `0x06` | `FAULT_FREQ_DRIFT` | $f_{\text{est}} < 48.5\text{ Hz}$ or $f_{\text{est}} > 51.5\text{ Hz}$ | 15 cycles (300 ms) | Trip |
| `0x07` | `FAULT_HIGH_THD_WARN` | $\text{THD}_I > 35.0\%$ and $I_{\text{rms}} > 0.3\text{ A}$ | 25 cycles (500 ms) | Warning Only (No Trip) |
| `0x08` | `FAULT_ADC_HEALTH` | Clipped (`0` or `4095` > 8 samples/cycle) or stuck/stale | 5 cycles (100 ms) | Safe Trip |
| `0x09` | `FAULT_RELAY_VERIFY` | Physical disconnect or reclose verification failed | Per FSM (§3.4) | Latched `TRIP_FAILURE` |

### 3.4 Physical Relay Disconnect & Reclose Verification FSM

**10-State Machine:**
`BOOT_SAFE_OPEN` -> `SELF_TEST` -> `NORMAL_CLOSED` -> `TRIP_REQUESTED` -> `COIL_DEENERGIZED` -> `VERIFY_OPEN` -> (`PHYSICAL_DISCONNECT_CONFIRMED` | `TRIP_FAILURE` | `DISCONNECT_UNVERIFIED`) -> `RESET_CHECK` -> `VERIFY_CLOSED` -> (`NORMAL_CLOSED` | `CLOSE_FAILURE`).

**Disconnect Verification Algorithm (Runs at 50 Hz after `COIL_DEENERGIZED`):**
1. Wait settling/mechanical opening delay `T_SETTLE_MS = 100 ms` (configurable 20-500 ms).
2. Sample `V_src` (GP26/ADC0) and `V_load` (GP27/ADC1) every 20 ms up to timeout `T_VERIFY_MAX_MS = 600 ms`.
3. Evaluate every sample against:
   * `source_present`: $V_{\text{src}} \ge V_{\text{src,min}}$ (default `3.0 V`).
   * `load_deenergized`: $V_{\text{load}} < \min(0.10 \cdot V_{\text{src}}, 0.5\text{ V})$.
   * `aux_ok`: `true` if `AUX_CONTACT_ENABLED == 0`, else `GP16 == HIGH` (switch open with internal pull-up).
4. **Truth Table:**
   * If `!source_present` for 5 consecutive samples -> transition to `DISCONNECT_UNVERIFIED` (cannot prove relay opened if supply died).
   * If `source_present && load_deenergized && aux_ok` holds for **5 consecutive samples (100 ms)** -> transition to `PHYSICAL_DISCONNECT_CONFIRMED`.
   * If timeout expires with `source_present && (!load_deenergized || !aux_ok)` -> transition to `TRIP_FAILURE` (welded contact, bypass, or wiring fault).
5. **Reclose Verification (`CMD_RESET_RELAY`):**
   * Reject if fault condition is still active or `!source_present`.
   * Energize `GP14`, wait `T_SETTLE_MS`, verify `V_load >= 0.80 * V_src` for 5 consecutive samples -> `NORMAL_CLOSED`; otherwise de-energize `GP14` immediately and enter `CLOSE_FAILURE`.

**Feedback Modes (Runtime Switchable via `CMD_SET_FEEDBACK_MODE`):**
* `0 = FEEDBACK_HARDWARE_HIGHSIDE`: Real ADC0/ADC1 readings (Lab Relay or High-Side MOSFET).
* `1 = FEEDBACK_HARDWARE_LOWSIDE`: Real ADC0/ADC1 with $V_{\text{load,eff}} = \max(0, V_{\text{src}} - V_{\text{adc1}})$ (Tonight's N-MOSFET + LED desk test).
* `2 = FEEDBACK_VIRTUAL_OK`: Software-emulated healthy 12 V supply + 25 ms relay opening (zero-wiring software test).
* `3 = FEEDBACK_VIRTUAL_WELDED`: Software-emulated welded contact ($V_{\text{load}}$ stays at 12 V) to safely test `TRIP_FAILURE`.
* `4 = FEEDBACK_VIRTUAL_NO_SRC`: Software-emulated lost supply ($V_{\text{src}} = 0\text{ V}$) to safely test `DISCONNECT_UNVERIFIED`.

### 3.5 Event-Driven Physics-Guided NILM (Pure C99, Sub-1-Second)

Grounded in **Hart (1992)** ([DOI:10.1109/5.192069](https://www.google.com/search?q=https%3A%2F%2Fdoi.org%2F10.1109%2F5.192069)), **Azzini et al. (2018)** ([Semantic Scholar](https://www.semanticscholar.org/paper/Event-detection-methods-for-nonintrusive-load-Azzini-Torquato/b2ee5df5dff1799888c4582f73401114f2271d12)), **Tabanelli et al. (2020)**, **Athanasiadis et al. (2021)** ([DOI:10.3390/en14030767](https://www.google.com/search?q=https%3A%2F%2Fdoi.org%2F10.3390%2Fen14030767)), and **Explainable Real-Time Event-Based NILM (2025)** ([arXiv:2501.16841](https://arxiv.org/abs/2501.16841)):

**Stage 1: Two-Sided CUSUM Event Detector (50 Hz cycle stream)**
Track exponentially weighted steady-state baseline $\mu_P[m], \mu_Q[m]$ (frozen during active event capture) and composite power step $e[m] = (P[m] - \mu_P) + 0.5(Q_1[m] - \mu_Q)$:

$$g^+[m] = \max(0, g^+[m-1] + e[m] - \delta), \quad g^-[m] = \max(0, g^-[m-1] - e[m] - \delta)$$

With drift $\delta = 15.0\text{ W}$ and threshold $h = 45.0\text{ W}$:
* $g^+[m] > h \implies$ **TURN-ON event** detected; capture 25 cycles (**0.50 s**) post-event window, then apply 15-cycle lockout.
* $g^-[m] > h \implies$ **TURN-OFF event** detected; capture 15 cycles (**0.30 s**) post-event window, match against currently ON appliances by closest $(\Delta P, \Delta Q_1)$ signature.

**Stage 2: 10-Feature Physics Vector ($\mathbf{x} \in \mathbb{R}^{10}$) on TURN-ON**
Compute pre-event averages (last 10 cycles before trigger) and post-event steady averages (cycles 18..25 after trigger), plus peak current over cycles 1..15:
1. $x_0 = |\Delta P|$ (W)
2. $x_1 = \Delta Q_1$ (var)
3. $x_2 = |\Delta S|$ (VA)
4. $x_3 = \text{PF}_{\Delta} = \frac{\Delta P}{\max(\sqrt{\Delta P^2 + \Delta Q_1^2}, 1.0)}$
5. $x_4 = \Delta I_3 / \max(\Delta I_1, 0.05)$ (3rd harmonic current ratio)
6. $x_5 = \Delta I_5 / \max(\Delta I_1, 0.05)$ (5th harmonic current ratio)
7. $x_6 = \Delta I_7 / \max(\Delta I_1, 0.05)$ (7th harmonic current ratio)
8. $x_7 = \Delta \text{THD}_I$ (fractional)
9. $x_8 = I_{\text{inrush,peak}} / \max(\Delta I_{\text{ss}}, 0.05)$ (inrush peak-to-steady ratio)
10. $x_9 = \text{sign}(\Delta Q_1) \cdot \frac{|\Delta Q_1|}{\max(|\Delta P|, 1.0)}$ (reactive-to-active ratio)

Standardize: $z_j = \frac{x_j - \mu_j}{\sigma_j}$ using training-set mean/std constants in `nilm_weights.h`.

**Stage 3: Pure-C 2-Layer MLP + Mahalanobis Unknown-Load Gate**
* **Network:** `Input(10) -> Dense(16, ReLU) -> Dense(4, Softmax)` in pure C99 `float` (~244 parameters = **< 1 KB flash**, **< 0.1 ms** inference on RP2350 Cortex-M33 FPU).
* **Unknown-Load Rejection Gate:** For predicted class $c = \arg\max_k p_k$, compute diagonal Mahalanobis distance to class centroid $(\boldsymbol{\mu}_c, \boldsymbol{\sigma}_c)$:
  $$d_M(\mathbf{z}, c) = \sqrt{\sum_{j=0}^{9} \left(\frac{z_j - \mu_{c,j}}{\sigma_{c,j} + 0.05}\right)^2}$$
  If $p_c < 0.70$ **or** $d_M(\mathbf{z}, c) > 4.5$, classify as `APPLIANCE_UNKNOWN` instead of forcing a false match.
* **Deterministic State Tracker & Residual:** Maintain `bool state[4]`, `float est_power_w[4]`, `float est_energy_wh[4]`, and compute unexplained residual:
  $$P_{\text{unknown}} = \max\left(0, P_{\text{meas}} - \sum_{a=0}^{3} \hat{P}_a \cdot \text{state}[a]\right)$$

**Offline Training Script (`scripts/train_nilm.py`):**
Codex must write a deterministic (`seed=42`) Python script (using `numpy` only — zero PyTorch/TF bloat) that calls the exact plant equations over 400 randomized switching events per appliance (varying $V_{\text{grid}} \in [215, 245]\text{ V}$, $\pm 8\%$ load tolerance, random background combinations, and ADC noise), trains the `10 -> 16 -> 4` MLP with Adam/SGD, computes per-class centroids/stds, validates >= 95% held-out accuracy, and emits `firmware/lib/gridguard_core/include/nilm_weights.h`.

---

## 4. Zephyr RTOS Firmware Blueprint

### 4.1 Board Target & Devicetree Overlay

* **Zephyr Version:** `v4.3.0+` (first release with mainline `rpi_pico2/rp2350a/m33/w` support — [Golioth Blog](https://blog.golioth.io/building-zephyr-for-the-raspberry-pi-pico2-w/)).
* **Build Command:** `west build -p -b rpi_pico2/rp2350a/m33/w firmware`
* **Host Simulation Build:** `west build -p -b native_sim firmware` (for CI/host testing).

**`firmware/boards/rpi_pico2_rp2350a_m33_w.overlay`:**
```dts
/ {
    chosen {
        zephyr,console = &cdc_acm_uart0;
        zephyr,shell-uart = &cdc_acm_uart0;
    };

    gridguard_gpios {
        compatible = "gpio-keys";
        relay_drv: relay_drv {
            gpios = <&gpio0 14 GPIO_ACTIVE_HIGH>;
            label = "RELAY_DRV_OUT";
        };
        status_led: status_led {
            gpios = <&gpio0 15 GPIO_ACTIVE_HIGH>;
            label = "STATUS_LED_OUT";
        };
        aux_contact: aux_contact {
            gpios = <&gpio0 16 (GPIO_PULL_UP | GPIO_ACTIVE_LOW)>;
            label = "AUX_CONTACT_IN";
        };
    };

    zephyr,user {
        io-channels = <&adc 0>, <&adc 1>;
    };
};

&zephyr_udc0 {
    cdc_acm_uart0: cdc_acm_uart0 {
        compatible = "zephyr,cdc-acm-uart";
    };
};

&adc {
    status = "okay";
    #address-cells = <1>;
    #size-cells = <0>;

    channel@0 {
        reg = <0>;
        zephyr,gain = "ADC_GAIN_1";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,resolution = <12>;
    };

    channel@1 {
        reg = <1>;
        zephyr,gain = "ADC_GAIN_1";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,resolution = <12>;
    };
};

&flash0 {
    partitions {
        compatible = "fixed-partitions";
        #address-cells = <1>;
        #size-cells = <1>;

        /* Reserve top 64 KB of 4 MB Pico 2 W flash for NVS fault log */
        storage_partition: partition@3f0000 {
            label = "storage";
            reg = <0x003f0000 0x00010000>;
        };
    };
};
```

**`firmware/prj.conf`:**
```ini
# FPU & Math
CONFIG_FPU=y
CONFIG_NEWLIB_LIBC=y
CONFIG_NEWLIB_LIBC_FLOAT_PRINTF=y

# USB CDC-ACM Serial Transport
CONFIG_USB_DEVICE_STACK=y
CONFIG_USB_DEVICE_PRODUCT="GridGuard Pico 2 W"
CONFIG_USB_DEVICE_INITIALIZE_AT_BOOT=n
CONFIG_USB_CDC_ACM=y
CONFIG_SERIAL=y
CONFIG_UART_INTERRUPT_DRIVEN=y
CONFIG_UART_LINE_CTRL=y

# Peripherals: GPIO, ADC, Flash + NVS
CONFIG_GPIO=y
CONFIG_ADC=y
CONFIG_FLASH=y
CONFIG_FLASH_MAP=y
CONFIG_FLASH_PAGE_LAYOUT=y
CONFIG_NVS=y
CONFIG_CRC=y

# Threading & Diagnostics
CONFIG_THREAD_NAME=y
CONFIG_THREAD_STACK_INFO=y
CONFIG_INIT_STACKS=y
CONFIG_MAIN_STACK_SIZE=2048
CONFIG_SYSTEM_WORKQUEUE_STACK_SIZE=2048
```

### 4.2 5-Thread Deterministic RTOS Architecture

| Thread Name | Priority | Stack | Period / Trigger | Responsibility |
| :--- | :--- | :--- | :--- | :--- |
| `gg_dsp_prot` | 2 (Highest) | 4096 B | 20 ms `k_timer` / `k_sem` | Steps plant 64 samples (3.2 kHz), runs IEEE 1459 + Goertzel metrology, evaluates IEC 60255 protection, triggers relay trip immediately |
| `gg_relay_ver` | 3 | 2048 B | 20 ms | Reads `ADC0`/`ADC1` + `GP16` (or virtual feedback), steps 10-state relay FSM, enforces disconnect/reclose verification |
| `gg_nilm` | 5 | 4096 B | `k_msgq` (50 Hz cycles) | Runs CUSUM event detector, extracts 10-feature vector, runs pure-C MLP + Mahalanobis gate, updates appliance state tracker |
| `gg_comms` | 6 | 4096 B | RX IRQ + 50 ms TX tick | Handles COBS + CRC-16-CCITT framing over USB CDC-ACM, dispatches commands, streams telemetry/waveforms |
| `gg_nvs_log` | 8 (Lowest) | 2048 B | `k_msgq` (Event-only) | Commits `gg_fault_record_t` structs to Zephyr NVS on trip/verify/reset; never writes per sample |

### 4.3 Persistent NVS Fault Record (`gg_fault_record_t`, 48 bytes packed)

Store a monotonic ring counter (`NVS_KEY_META = 0x0001`) and up to 64 fault records (`NVS_KEY_LOG_BASE = 0x1000` .. `0x103F`). At boot, reload the latest record; if its `relay_state` was `TRIP_FAILURE` or unacknowledged trip, keep relay **open** (`BOOT_SAFE_OPEN`) until an explicit `CMD_RESET_RELAY` passes all checks.

---

## 5. Binary Wire Protocol (COBS + CRC-16-CCITT)

Grounded in **Cheshire & Baker (1999)** ([DOI:10.1109/90.769765](https://www.computer.org/csdl/journal/nt/1999/02/00769765/13rRUyft7we)):
* **Frame Format (Pre-COBS):** `[version:u8=1][msg_type:u8][seq:u16][payload_len:u16][payload:N bytes][crc16:u16]`
* **CRC-16-CCITT:** Polynomial `0x1021`, Init `0xFFFF`, no final XOR, computed over `[version .. end of payload]`.
* **On-Wire:** `COBS_Encode(frame)` followed by single `0x00` delimiter byte. Max unencoded frame = 512 bytes.

### Message Type Registry (Little-Endian Packed Structs)

| ID | Name | Direction | Rate / Trigger | Payload Fields (Exact Order) |
| :--- | :--- | :--- | :--- | :--- |
| `0x01` | `MSG_HELLO` | Pico -> Host | On connect / 2 s | `fw_major:u8, fw_minor:u8, board_id:u8, feedback_mode:u8, uptime_ms:u32` |
| `0x02` | `MSG_TELEMETRY` | Pico -> Host | 20 Hz | `ts_ms:u32, v_rms:f32, i_rms:f32, p_w:f32, q1_var:f32, s_va:f32, pf:f32, freq_hz:f32, thd_i_pct:f32, thd_v_pct:f32, energy_wh:f32, thermal_pct:f32, harm_i_rms[5]:f32` (h=1,3,5,7,9) |
| `0x03` | `MSG_WAVEFORM` | Pico -> Host | 5 Hz | `ts_ms:u32, v_samples[64]:i16` (scaled x10 V), `i_samples[64]:i16` (scaled x100 A) |
| `0x04` | `MSG_NILM_STATE` | Pico -> Host | On event / 2 Hz | `ts_ms:u32, active_mask:u8, last_event_id:u8, last_event_type:u8, reserved:u8, est_power_w[4]:f32, confidence[4]:f32, unknown_power_w:f32, last_mahalanobis:f32, inference_us:u32` |
| `0x05` | `MSG_PROTECTION` | Pico -> Host | On change / 10 Hz | `ts_ms:u32, relay_state:u8, active_fault:u8, coil_cmd:u8, aux_state:u8, feedback_mode:u8, verify_samples:u8, reserved:u16, v_src_v:f32, v_load_v:f32, trip_latency_us:u32` |
| `0x06` | `MSG_FAULT_RECORD` | Pico -> Host | On trip / query | `record_id:u32, ts_ms:u32, fault_code:u8, relay_result:u8, coil_cmd:u8, aux_state:u8, trigger_val:f32, threshold_val:f32, v_src_v:f32, v_load_v:f32, verify_ms:u16, crc32:u32` |
| `0x07` | `MSG_HEALTH` | Pico -> Host | 1 Hz | `ts_ms:u32, dsp_exec_us:u32, max_dsp_us:u32, dropped_frames:u32, uart_crc_err:u32, nvs_records:u16, free_stack_min:u16` |
| `0x10` | `CMD_SET_APPLIANCE` | Host -> Pico | User click | `appliance_id:u8 (0..3), target_state:u8 (0/1)` |
| `0x11` | `CMD_INJECT_FAULT` | Host -> Pico | User click | `fault_scenario:u8 (0=CLEAR, 1..7), magnitude:f32` |
| `0x12` | `CMD_RESET_RELAY` | Host -> Pico | User click | `force_clear_latch:u8 (0/1)` (still runs full `RESET_CHECK` & `VERIFY_CLOSED`) |
| `0x13` | `CMD_SET_FEEDBACK_MODE` | Host -> Pico | User click | `feedback_mode:u8 (0..4)` |
| `0x14` | `CMD_READ_FAULTS` | Host -> Pico | On load | `start_index:u16, count:u16` |
| `0x15` | `MSG_ACK_NACK` | Pico -> Host | Per CMD | `cmd_seq:u16, cmd_type:u8, status:u8 (0=OK, 1=REJECTED, 2=INVALID)` |

---

## 6. Host Backend (FastAPI) & Frontend (React + ECharts)

### 6.1 FastAPI Backend (`host/backend/`)
* **Serial Manager:** Auto-detects Pico USB CDC-ACM port (`/dev/ttyACM*` on Linux, `COM*` matching Raspberry Pi/Zephyr VID/PID on Windows) with automatic reconnect.
* **Built-In Replay / Demo Mode:** Launching `uvicorn app.main:app` with `GRIDGUARD_REPLAY=1` (or `--replay`) runs an honest Python replay generator emitting identical Pydantic schemas with `mode: "REPLAY"` clearly flagged in the WebSocket header.
* **Endpoints:**
  * `WS /ws/live`: Pushes JSON-serialized telemetry, waveforms, NILM, protection, and health frames at up to 20 Hz.
  * `POST /api/cmd/appliance`, `POST /api/cmd/fault`, `POST /api/cmd/reset-relay`, `POST /api/cmd/feedback-mode`.
  * `GET /api/faults` & `GET /api/faults/export.csv` (retrieves Pico NVS records and exports CSV/JSON).

### 6.2 React Engineering Console (`host/frontend/`) — 4 Tight Views
Dark, restrained industrial instrumentation aesthetic (`bg-slate-950`, monospace tabular numerals `font-mono tabular-nums`, zero marketing fluff). Status color contract: **Green** = verified connected/closed, **Amber** = `VERIFYING` / warning / `REPLAY MODE`, **Red** = active fault / `TRIP_FAILURE`, **Grey** = `DISCONNECT_UNVERIFIED` / disconnected.

1. **Overview & Physical Protection:** Big unambiguous Physical Relay Verification banner (`NORMAL_CLOSED` / `VERIFYING` / `PHYSICAL_DISCONNECT_CONFIRMED` / `TRIP_FAILURE` / `DISCONNECT_UNVERIFIED`), live `V_src` vs `V_load` bar gauges, auxiliary contact badge, 6 compact IEEE 1459 KPI cards (`V_rms`, `I_rms`, `P`, `Q1`, `PF`, `Freq` + `THD_I`), and IEC 60255 thermal $\Theta\%$ progress bar.
2. **Waveforms & Harmonics:** Real-time ECharts oscilloscope plot (64-sample 50 Hz `v(t)` and `i(t)` dual-axis) + ECharts bar chart of odd current harmonics ($h=1,3,5,7,9$) + live `V_src`/`V_load` trend chart.
3. **NILM Disaggregation:** 4 appliance state cards (Kettle, Fan, SMPS, Fridge) with live ON/OFF badge, estimated power/energy, confidence score, Mahalanobis distance indicator, and Unknown Load Residual gauge.
4. **Fault History & Scenario Lab:** Interactive scenario controls (toggle simulated appliances, inject any of the 7 faults, switch relay feedback mode between Hardware High-Side / Low-Side MOSFET / Virtual Welded / Virtual No-Source, request Safe Reset) + Pico NVS Fault Log table with CSV export and System Health footer (DSP latency, CRC errors, stack headroom).

---

## 7. GitHub Repository Structure & Cross-Platform Setup

### 7.1 Repository Layout

```text
GridGuard/
├── .github/workflows/ci.yml
├── .gitignore
├── README.md
├── docs/
│   ├── SETUP_ENDEAVOUROS.md
│   ├── SETUP_WINDOWS.md
│   └── WIRING_AND_SAFETY.md
├── scripts/
│   ├── train_nilm.py
│   ├── setup_endeavouros.sh
│   └── setup_windows.ps1
├── firmware/
│   ├── CMakeLists.txt
│   ├── prj.conf
│   ├── boards/rpi_pico2_rp2350a_m33_w.overlay
│   ├── lib/gridguard_core/
│   │   ├── CMakeLists.txt
│   │   ├── include/
│   │   │   ├── gg_plant.h
│   │   │   ├── gg_metrology.h
│   │   │   ├── gg_protection.h
│   │   │   ├── gg_relay_fsm.h
│   │   │   ├── gg_nilm.h
│   │   │   ├── gg_protocol.h
│   │   │   └── nilm_weights.h
│   │   ├── src/*.c
│   │   └── tests/test_core.c
│   └── src/
│       ├── main.c
│       ├── hal_adc_relay.c
│       ├── nvs_store.c
│       └── usb_comms.c
└── host/
    ├── backend/
    │   ├── requirements.txt
    │   ├── app/{main.py,protocol.py,serial_bridge.py,replay.py,schemas.py}
    │   └── tests/test_protocol.py
    └── frontend/
        ├── package.json
        ├── vite.config.ts
        ├── tailwind.config.ts
        └── src/{App.tsx,types.ts,hooks/useGridGuardWs.ts,pages/*.tsx}
```

### 7.2 Setup on EndeavourOS (Arch Linux)

Codex must generate `scripts/setup_endeavouros.sh` and `docs/SETUP_ENDEAVOUROS.md` covering:
1. **System packages:** `sudo pacman -S --needed git cmake ninja gperf ccache dfu-util dtc wget python python-pip python-virtualenv nodejs npm base-devel`
2. **Serial permissions (USB CDC-ACM):** `sudo usermod -aG uucp $USER` (Arch/EndeavourOS uses `uucp` for `/dev/ttyACM0`, not `dialout`) + udev rule for Pico BOOTSEL / USB-CDC.
3. **Zephyr workspace (`~/zephyrproject`):**
   ```bash
   python -m venv ~/zephyrproject/.venv && source ~/zephyrproject/.venv/bin/activate
   pip install west pyelftools
   west init ~/zephyrproject --mr v4.3.0
   cd ~/zephyrproject && west update && west zephyr-export
   pip install -r ~/zephyrproject/zephyr/scripts/requirements.txt
   west sdk install -t arm-zephyr-eabi
   ```
4. **Build & Flash Pico 2 W (UF2 drag-and-drop, no debug probe required):**
   ```bash
   west build -p -b rpi_pico2/rp2350a/m33/w firmware
   # Hold BOOTSEL while plugging Pico 2 W in, then copy:
   cp build/zephyr/zephyr.uf2 /run/media/$USER/RP2350/
   ```
5. **Run Host Stack:**
   ```bash
   cd host/backend && python -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt
   uvicorn app.main:app --reload --port 8000
   # Second terminal:
   cd host/frontend && npm install && npm run dev
   ```

### 7.3 Setup on Windows 11

Codex must generate `scripts/setup_windows.ps1` and `docs/SETUP_WINDOWS.md` covering:
1. **Prerequisites via `winget`:** `winget install Kitware.CMake Ninja-build.Ninja Git.Git Python.Python.3.12 OpenJS.NodeJS.LTS 7zip.7zip` (plus `dtc` / `gperf` via Chocolatey or MSYS2, or `west sdk install`).
2. **Zephyr workspace (`%USERPROFILE%\zephyrproject`):**
   ```powershell
   python -m venv $env:USERPROFILE\zephyrproject\.venv
   & $env:USERPROFILE\zephyrproject\.venv\Scripts\Activate.ps1
   pip install west pyelftools
   west init $env:USERPROFILE\zephyrproject --mr v4.3.0
   cd $env:USERPROFILE\zephyrproject; west update; west zephyr-export
   pip install -r zephyr\scripts\requirements.txt
   west sdk install -t arm-zephyr-eabi
   ```
3. **Build & Flash Pico 2 W:**
   ```powershell
   west build -p -b rpi_pico2/rp2350a/m33/w firmware
   # Hold BOOTSEL while plugging Pico 2 W in, then copy build\zephyr\zephyr.uf2 to the RP2350 USB drive.
   ```
4. **Run Host Stack:** Auto-detects `COMx` via `pyserial` (`serial.tools.list_ports`), or pass `--port COM5` / `GRIDGUARD_REPLAY=1`.

---

## 8. Acceptance Tests & Verification Checklist

Codex must implement automated tests verifying every item below:

| Test ID | Target | Exact Pass Criterion |
| :--- | :--- | :--- |
| `T1` | Metrology (`test_core.c`) | Pure 230 V / 4.35 A resistive sine -> $V_{\text{rms}} = 230.0 \pm 0.5\text{ V}$, $P = 1000 \pm 3\text{ W}$, $\text{PF} > 0.995$, $f = 50.00 \pm 0.05\text{ Hz}$, $\text{THD}_I < 1\%$ |
| `T2` | Goertzel Harmonics (`test_core.c`) | SMPS load -> $I_3/I_1 = 0.72 \pm 0.03$, $I_5/I_1 = 0.48 \pm 0.03$, $\text{THD}_I$ matches analytical $\sqrt{0.72^2+0.48^2+0.26^2+0.12^2} \approx 91.1\% \pm 2\%$ |
| `T3` | IEC 60255 IDMT (`test_core.c`) | At $M = 2.0$ ($16\text{ A}$, EI curve, $\text{TMS}=0.15$), theoretical $t_{\text{op}} = 0.15 \times \frac{80}{2^2 - 1} = 4.00\text{ s}$; integrator trips within $4.00 \pm 0.06\text{ s}$ |
| `T4` | NILM (`train_nilm.py` + `test_core.c`) | >= 95% classification accuracy across all 4 appliances on held-out test set; an unmodeled 600 W pure-capacitive load triggers `APPLIANCE_UNKNOWN` via Mahalanobis gate |
| `T5` | Relay Verification FSM (`test_core.c`) | (a) Healthy open -> `PHYSICAL_DISCONNECT_CONFIRMED` only after 5 samples below threshold; (b) Welded (`V_load = 12 V`) -> `TRIP_FAILURE`; (c) Lost source (`V_src = 0 V`) -> `DISCONNECT_UNVERIFIED` |
| `T6` | Wire Protocol (`test_protocol.py`) | COBS encode/decode round-trip on payloads containing `0x00` and `0xFF` bytes + 1-bit flip rejected by CRC-16-CCITT |
| `T7` | Physical Bench & Reboot Test (User) | Trigger fault -> physical LED/relay opens -> `PHYSICAL_DISCONNECT_CONFIRMED` -> power-cycle Pico -> fault record persists from Zephyr NVS |

---

## 9. Academic & Technical References

1. G. W. Hart, "Nonintrusive Appliance Load Monitoring," *Proceedings of the IEEE*, vol. 80, no. 12, pp. 1870-1891, 1992. [https://doi.org/10.1109/5.192069](https://www.google.com/search?q=https%3A%2F%2Fdoi.org%2F10.1109%2F5.192069)
2. IEEE Std 1459-2010 / 1459-2025, *IEEE Standard Definitions for the Measurement of Electric Power Quantities Under Sinusoidal, Nonsinusoidal, Balanced, or Unbalanced Conditions*. [https://standards.ieee.org/ieee/1459/7578/](https://standards.ieee.org/ieee/1459/7578/)
3. IEC 60255-151:2009, *Measuring relays and protection equipment - Functional requirements for over/under current protection* (IDMT Curve Constants). [https://www.gridapm.com/blog/iec-60255-151-inverse-time-curve-constants/](https://www.gridapm.com/blog/iec-60255-151-inverse-time-curve-constants/)
4. G. Goertzel, "An Algorithm for the Evaluation of Finite Trigonometric Series," *The American Mathematical Monthly*, vol. 65, no. 1, pp. 34-35, 1958. [https://en.wikipedia.org/wiki/Goertzel_algorithm](https://en.wikipedia.org/wiki/Goertzel_algorithm)
5. E. Tabanelli, D. Brunelli, and L. Benini, "A Feature Reduction Strategy for Enabling Lightweight Non-Intrusive Load Monitoring on Edge Devices," *IEEE ISIE*, 2020.
6. C. Athanasiadis et al., "A Scalable Real-Time Non-Intrusive Load Monitoring System for the Estimation of Household Appliance Power Consumption," *Energies*, vol. 14, no. 3, 767, 2021. [https://doi.org/10.3390/en14030767](https://www.google.com/search?q=https%3A%2F%2Fdoi.org%2F10.3390%2Fen14030767)
7. H. Azzini et al., "Event detection methods for nonintrusive load monitoring," *IEEE ICS*, 2018. [https://www.semanticscholar.org/paper/Event-detection-methods-for-nonintrusive-load-Azzini-Torquato/b2ee5df5dff1799888c4582f73401114f2271d12](https://www.semanticscholar.org/paper/Event-detection-methods-for-nonintrusive-load-Azzini-Torquato/b2ee5df5dff1799888c4582f73401114f2271d12)
8. "Toward Explainable NILM: Real-Time Event-Based NILM Framework for High-Frequency Data," *arXiv:2501.16841*, 2025. [https://arxiv.org/abs/2501.16841](https://arxiv.org/abs/2501.16841)
9. S. Cheshire and M. Baker, "Consistent Overhead Byte Stuffing," *IEEE/ACM Transactions on Networking*, vol. 7, no. 2, pp. 159-172, 1999. [https://www.computer.org/csdl/journal/nt/1999/02/00769765/13rRUyft7we](https://www.computer.org/csdl/journal/nt/1999/02/00769765/13rRUyft7we)
10. Raspberry Pi Pico 2 / RP2350 Pinout, ADC & Erratum RP2350-E9 Technical Notes. [https://www.tinkered.ai/components/raspberry-pi-pico-2](https://www.tinkered.ai/components/raspberry-pi-pico-2) and [https://en.wikipedia.org/wiki/RP2350](https://en.wikipedia.org/wiki/RP2350)
11. Building Zephyr RTOS for Raspberry Pi Pico 2 W (`rpi_pico2/rp2350a/m33/w` & USB-CDC). [https://blog.golioth.io/building-zephyr-for-the-raspberry-pi-pico2-w/](https://blog.golioth.io/building-zephyr-for-the-raspberry-pi-pico2-w/)
