
## 1. Overview
This repository contains a deterministic, non-blocking embedded firmware written in C for the **STM32F407VG (ARM Cortex-M4F)** microcontroller. It operates as the low-level real-time controller for a differential drive mobile robot, handling dual DC motor velocity ramping, hardware quadrature encoder tracking, filtered analog power telemetry (ADC), and 4-channel industrial relay control.

To eliminate CPU bottlenecks and timing jitter common in polling-based (`HAL_Delay`) loops, the firmware utilizes **Direct Memory Access (DMA)**, **UART IDLE Line Detection**, **Hardware Timer Offloading**, and **Object-Oriented C (`struct`)** design patterns.

---

## 2. System Block Diagram

```text
+-----------------------------------------------------------------------------------+
|                        STM32F407VG @ 168 MHz (Cortex-M4F)                         |
|                                                                                   |
|  +-----------------------+      +----------------------------------------------+  |
|  |   USART2 (DMA+IDLE)   |<---->| Host PC / ROS 2 Bridge (115200 8N1)          |  |
|  |   uart2_ros           |      | RX: Motor Duty Cmds | TX: 100 Hz Odometry    |  |
|  +-----------------------+      +----------------------------------------------+  |
|                                                                                   |
|  +-----------------------+      +----------------------------------------------+  |
|  |   USART3 (DMA+IDLE)   |<---->| Relay Control Interface (115200 8N1)         |  |
|  |   uart3_relay         |      | RX: ON/OFF/ALL Cmds | TX: State Feedback     |  |
|  +-----------------------+      +----------------------------------------------+  |
|                                                                                   |
|  +-----------------------+      +----------------------------------------------+  |
|  |   UART4 (DMA TX)      |----->| Power Telemetry Stream (115200 8N1 @ 10 Hz)  |  |
|  |   uart4_adc           |      | TX: Filtered Battery Current, Charge & Volt  |  |
|  +-----------------------+      +----------------------------------------------+  |
|                                                                                   |
|  +-----------------------+      +----------------------------------------------+  |
|  | TIM6 Real-Time ISR    |      | Executed Deterministically Every 10 ms:      |  |
|  | 100 Hz (10 ms)        |----->| 1. Sample TIM2/TIM3 Hardware Encoders (dL/dR)|  |
|  |                       |      | 2. Update Motor Slew-Rate Ramps (L & R)      |  |
|  |                       |      | 3. Trigger 3-Ch ADC1 DMA Conversion          |  |
|  +-----------------------+      +----------------------------------------------+  |
+-----------------------------------------------------------------------------------+
```

---

## 3. Firmware Architecture & Core Modules

### 3.1. Non-Blocking Multi-UART DMA Engine (`UartComm.h` / `UartComm.c`)
All serial ports (`USART2`, `USART3`, `UART4`) operate asynchronously via DMA reception paired with hardware **IDLE Line Interrupts (`UART_IT_IDLE`)**. The CPU is never blocked waiting for incoming bytes; an interrupt is generated only when a complete data frame finishes arriving on the bus.

* **Encapsulated State (`volatile UartComm_t`):** Groups hardware handles (`huart`, `hdma_rx`), isolated `rx_buff` and `tx_buff` arrays, and synchronization flags (`rx_ready`, `tx_busy`, `tx_ready`) into a single structure per peripheral (`uart2_ros`, `uart3_relay`, `uart4_adc`).
* **`Uart_Init()`:** Attaches UART/DMA handles, enables `UART_IT_IDLE`, and arms DMA reception with `UART_BUFFER_SIZE - 1` to guarantee space for a null-terminator (`\0`).
* **`Uart_RxIdleCallback()`:** Invoked inside the UART ISR upon line idle detection. Halts DMA, computes exact payload length via `__HAL_DMA_GET_COUNTER`, appends `\0`, sets `rx_ready = 1`, and immediately rearms DMA.
* **`Uart_SendMsg()`:** Copies outgoing strings into the port's isolated `tx_buff` using `memcpy` and initiates non-blocking `HAL_UART_Transmit_DMA()` transfers with a 10 ms timeout guard (`tx_busy` cleared inside `HAL_UART_TxCpltCallback`).

### 3.2. Object-Oriented Motor Driver & Slew-Rate Limiter (`motor.h` / `motor.c`)
Controls dual H-Bridge DC motor drivers using 16 kHz hardware PWM (`TIM1_CH1` & `TIM8_CH1`, Period: `10499`) with deterministic trapezoidal velocity ramping.

* **Encapsulated State (`Motor_t`):** Stores timer handles, PWM channels, 16-bit GPIO direction pin masks (`dir1_pin_forward`, `dir2_pin_back`), normalized duty cycle states (`-1.0f` to `1.0f`), and ramp step parameters.
* **`Motor_SetDutyWithRamp()`:** Computes the per-tick duty increment (`ramp_val`) required to transition from `current_duty` to `target_duty` over `ramp_time_ms` (configured to `1000 ms`).
* **`Motor_UpdateRamp()`:** Called at **100 Hz (10 ms)** inside the `TIM6` ISR. Smoothly steps motor output toward the target duty while applying deadzone suppression (`DEADZONE = 0.005f`).
* **H-Bridge Shoot-Through Protection:** When a direction reversal is requested (`current_duty * target_duty < 0`), the driver immediately forces PWM compare registers to `0`, puts direction pins into brake/stop state, and enforces a **20 ms (`zero_delay = 2`)** dead-time before energizing the opposite direction.

### 3.3. Hardware Quadrature Encoder Odometry (`main.c`)
High-frequency quadrature A/B pulses are counted directly in hardware by `TIM2` (Left) and `TIM3` (Right) configured in **Encoder Interface Mode (`TIM_ENCODERMODE_TI12`)**, imposing zero interrupt load on the CPU per pulse.

* Every **10 ms** inside `HAL_TIM_PeriodElapsedCallback` (`TIM6`), 16-bit counter registers are sampled to compute signed delta ticks (`dL` and `dR`). Using 16-bit signed integer arithmetic (`int16_t`) inherently handles counter wrap-around.
* `Uart_SendEncoder()` transmits `dR` and `dL` telemetry over `USART2` every **10 ms (100 Hz)**.

### 3.4. Timer-Paced ADC Telemetry & Trimmed Mean Filter (`main.c`)
Monitors three analog power channels via `ADC1` (`CH0`: Battery Discharge Current, `CH1`: Charging Current, `CH4`: Battery Voltage).

* **Controlled Sampling Rate:** Rather than running the ADC in continuous free-running mode (which causes DMA interrupt flooding), `HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, 3)` is triggered deterministically every **10 ms (100 Hz)** by `TIM6`.
* **Sliding Window & `trimmed_mean_filter()`:** `HAL_ADC_ConvCpltCallback()` collects 10 samples per channel (`buffSize = 10`). Once full (every **100 ms / 10 Hz**), the main loop copies each buffer, sorts the 10 readings via Bubble Sort, discards the 3 lowest and 3 highest noise spikes (`trimSize = 3`), and averages the middle 4 samples before converting to physical Ampere/Volt units (`Uart_SendAnalog()`).

### 3.5. 4-Channel Industrial Relay Controller (`relay.h` / `relay.c`)
Manages 4 digital output channels on `GPIOD` (`PD12`, `PD13`, `PD14`, `PD15`) mapped via the `RelayPin` struct array.

* **`Relay_ProcessCommand()`:** Parses incoming ASCII strings over `USART3` using reentrant `strtok_r` tokenization. Supports broadcast commands (`ALL_ON`, `ALL_OFF`) and multi-target comma-separated commands (e.g., `ON 1,3` or `OFF 2,4`).
* **`Relay_Set()`:** Drives the physical GPIO pin and dispatches real-time state confirmation strings (`Relay X ON/OFF`) over `USART3`.

---

## 4. Hardware Pinout & Peripheral Mapping

| Subsystem | STM32 Peripheral | Pin Mapping | Configuration / Description |
| :--- | :--- | :--- | :--- |
| **Left Motor PWM** | `TIM8_CH1` | `PC6` | PWM Output (~16 kHz, `ARR = 10499`) |
| **Left Motor DIR** | `GPIOC` | `PC4`, `PC5` | `MOTOR_L_DIR1_Pin`, `MOTOR_L_DIR2_Pin` (Open-Drain w/ Pull-Up) |
| **Right Motor PWM**| `TIM1_CH1` | `PE9` / `PA8` | PWM Output (~16 kHz, `ARR = 10499`) |
| **Right Motor DIR**| `GPIOB` | `PB0`, `PB1` | `MOTOR_R_DIR1_Pin`, `MOTOR_R_DIR2_Pin` (Open-Drain w/ Pull-Up) |
| **Left Encoder** | `TIM2` | `PA0`, `PA1` | Quadrature Encoder `TI12` (`ARR = 65535`) |
| **Right Encoder** | `TIM3` | `PA6`, `PA7` | Quadrature Encoder `TI12` (`ARR = 65535`) |
| **ROS 2 / Host UART**| `USART2` | `PA2`, `PA3` | 115200 bps, 8N1, `DMA1_Stream5` (RX) / `DMA1_Stream6` (TX) |
| **Relay UART** | `USART3` | `PB10`, `PB11` | 115200 bps, 8N1, `DMA1_Stream1` (RX) / `DMA1_Stream3` (TX) |
| **Telemetry UART** | `UART4` | `PC10`, `PC11` | 115200 bps, 8N1, `DMA1_Stream2` (RX) / `DMA1_Stream4` (TX) |
| **Power Sense ADC**| `ADC1` | `PA0/PA1/PA4` | `CH0` (Batt A), `CH1` (Charge A), `CH4` (Batt V) via `DMA2_Stream0` |
| **Relay Outputs** | `GPIOD` | `PD12` .. `PD15`| 4-Channel Push-Pull Digital Outputs (Relays 1–4) |
| **Emergency Stop** | `GPIO` | `A_STOP_Pin` | Digital Input with Pull-Down |
| **Real-Time Tick** | `TIM6` | Internal | 100 Hz (10 ms) Base Timer Interrupt (`PSC = 8399`, `ARR = 99`) |

---

## 5. Serial Communication Protocol (115200 8N1)

| Port | Direction | Frequency | Payload Format | Description |
| :--- | :---: | :---: | :--- | :--- |
| `USART2` | **RX** | Asynchronous | `M_R 0.65 M_L -0.50\r\n` | Sets target duty cycle (`-1.0` to `1.0`) for Right and Left motors |
| `USART2` | **TX** | 100 Hz (10 ms)| `dR 142 dL 140\r\n` | Signed delta encoder counts accumulated over the last 10 ms |
| `USART3` | **RX** | Asynchronous | `ALL_ON\r\n` or `ON 1,3\r\n`| Activates/deactivates all relays or comma-separated channels (`1..4`) |
| `USART3` | **TX** | Event-Driven | `Relay 1 ON \r\n` | Immediate hardware state confirmation per relay channel |
| `UART4` | **TX** | 10 Hz (100 ms)| `A0: 4.2A, A1: 0.0A, A2: 24.1V\r\n` | Trimmed-mean filtered battery current, charge current, and voltage |

---

## 6. Repository Structure

```text
├── Core
│   ├── Inc
│   │   ├── main.h              # CubeMX pin definitions & global macros
│   │   ├── UartComm.h          # Non-blocking DMA+IDLE UART struct & API
│   │   ├── motor.h             # Object-oriented motor control struct & API
│   │   └── relay.h             # 4-channel relay controller definitions
│   └── Src
│       ├── main.c              # System init, super-loop, TIM6 ISR & ADC filter
│       ├── stm32f4xx_it.c      # Hardware IRQ handlers (USART IDLE, DMA, TIM6)
│       ├── UartComm.c        # DMA RX/TX & buffer management implementation
│       ├── motor.c             # Velocity ramp generator & H-bridge logic
│       └── relay.c             # ASCII command tokenizer & GPIO switching
├── Drivers                     # STM32F4xx CMSIS & HAL standard peripheral drivers
└── .gitignore                  # Excludes Debug/Release build artifacts
```
