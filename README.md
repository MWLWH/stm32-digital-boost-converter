# STM32F446 Digital 1 kW DC-DC Boost Converter Firmware

Embedded C firmware developed on an **STM32F446RET6 (LQFP64)** microcontroller for a **1 kW (24–48 V input, 52.5 V regulated output, up to 20 A)** digital DC-DC boost converter, designed to regulate fuel cell power in a hydrogen-electric vehicle.

> **Hardware Documentation:** [View DC-DC Converter Schematic (`docs/Eys_sch.pdf`)](docs/Eys_sch.pdf)

---

## Key Features

- **Center-Aligned PWM & Triggered ADC-DMA Acquisition:**
  - Generates a high-frequency center-aligned PWM gate drive signal on `TIM8_CH2` (`PC7`) and triggers synchronized 5-channel `ADC1` conversions via `TIM8_TRGO` and `DMA2`.
- **Dual-Loop CC/CV PI Control with Mode Switching:**
  - Implements separate Constant-Voltage (`PI`), Constant-Current (`PI_A`), and No-Load (`PI_NO_LOAD`) Proportional-Integral controllers with anti-windup clamping and minimum-select arbitration for smooth CC/CV crossover.
- **Digital Signal Conditioning & `VREFINT` Compensation:**
  - Dynamically compensates ADC measurements using the factory-calibrated internal voltage reference (`VREFINT`), combined with a 5-point moving median filter on current feedback and Exponential Moving Average (EMA) filters.
- **Soft-Start, Slew-Rate Limiting & Fault Protection:**
  - Limits inrush current during startup via incremental duty-cycle ramping (`EYS_Soft_Start`) and bounds duty-cycle transients (`MAX_D_STEP = 5`).
  - Continuously monitors over-voltage (`> 59 V`), over-current (`> 20 A`), and input under-voltage (`< 24 V`), immediately halting PWM output upon fault detection.
- **CAN-Bus Telemetry & Remote Current Setpoint:**
  - Broadcasts scaled `V_IN`, `V_OUT`, `I_OUT`, and `I_REF` telemetry frames over `CAN1` (`ExtId: 0x00000011`) and accepts remote current reference commands from the Vehicle Control Unit (VCU) with automatic timeout fallback to the analog `INADC` setpoint input.

---

## Hardware & Software Stack

| Component / Tool | Specification |
| :--- | :--- |
| **Microcontroller** | STM32F446RETx (`LQFP64`, ARM Cortex-M4 @ 180 MHz) |
| **Power Stage Ratings** | Input: `24–48 V DC` \| Output Setpoint: `52.5 V DC` (Cutoff: `59 V / 20 A`) |
| **Analog Sensing** | Voltage Dividers (`VOUT`, `VIN`), Shunt Resistor (`5 mΩ`) + `20 V/V` Current Sense Amp (`CSENSE`) |
| **Peripherals** | `TIM8` (Center-Aligned PWM + TRGO), `ADC1` + `DMA2` (5 Channels), `CAN1` (Extended ID) |
| **Toolchain** | STM32CubeIDE, STM32 HAL |

---

## Pin Mapping (STM32F446RETx — LQFP64)

| Peripheral | Signal Label | STM32 Pin | Function |
| :--- | :--- | :--- | :--- |
| **TIM8_CH2** | `PWM` | `PC7` | Center-aligned PWM gate drive output |
| **ADC1_IN1** | `VOUT` | `PA1` | Output voltage feedback (`250 kΩ / 9.91 kΩ` divider) |
| **ADC1_IN4** | `VIN` | `PA4` | Input voltage feedback (`169 kΩ / 9.76 kΩ` divider) |
| **ADC1_IN5** | `INADC` | `PA5` | Analog current limit / setpoint reference input |
| **ADC1_IN6** | `CSENSE` | `PA6` | Output current sense feedback (`5 mΩ` shunt, `20x` gain) |
| **CAN1** | `CAN1_RX / CAN1_TX` | `PA11 / PA12` | VCU telemetry broadcast & remote current reference command |
| **GPIO Out** | `user_led` | `PB15` | Control loop heartbeat / status indicator LED |
| **SYS (SWD)** | `SWDIO / SWCLK` | `PA13 / PA14` | Serial Wire Debug programming & live watch interface |

---

## Control Loop Architecture

```text
┌───────────────────────────┐      ADC1 + DMA2 (5-Ch)      ┌──────────────────────────────┐
│  Boost Power Stage        │ ───(PA1, PA4, PA5, PA6)────▶ │       STM32F446RET6          │
│  (24–48V In → 52.5V Out)  │ ◀─────────(PC7 PWM)───────── │  Median/EMA + CC/CV PI Loop  │
└───────────────────────────┘            TIM8_CH2          └──────────────┬───────────────┘
                                                                          │ CAN1 (PA11/PA12)
                                                                          ▼
                                                           ┌──────────────────────────────┐
                                                           │  Vehicle Control Unit (VCU)  │
                                                           └──────────────────────────────┘