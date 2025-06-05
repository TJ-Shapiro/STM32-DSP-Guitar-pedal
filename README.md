# STM32 Digital Guitar Effects Pedal 🎸

A real-time digital signal processing guitar effects pedal built on an STM32 microcontroller. This project combines embedded systems, analog/digital electronics, and audio engineering to deliver high-quality guitar effects in a compact, custom PCB-powered unit.

[💻 View on My Site](www.tjshapiro.com/projects/guitar)

---

## 🛠 Overview

This pedal processes guitar audio signals in real time using a variety of effects, including:

- **Distortion**
- **Delay**
- **Reverb**

Built with both analog front-end input conditioning and digital signal processing pipelines, the system delivers **<1ms latency**, making it suitable for live performance environments.

---

## 🧠 Key Features

- ✅ Real-time audio processing with <1ms latency  
- 🎛️ Multiple DSP effects algorithms implemented in C  
- 🖨️ Custom PCB with isolated analog/digital domains  
- 🎧 I2S audio interface with 24-bit resolution  
- 🔌 USB serial interface for effect configuration  

## 🧩 Technologies Used

- **C / C++**  
- **STM32 (ARM Cortex-M)**  
- **I2C, I2S, SPI**  
- **CMSIS DSP / Custom FX Algorithms**  
- **RTOS (Custom Scheduler)**  

---

## ⚙️ System Architecture

- **Input Stage:** Analog buffering, anti-aliasing filters  
- **Core DSP:** STM32 processes effects in real-time  
- **Output Stage:** DAC and analog reconstruction filters  
- **Control Interface:** USB commands for patch switching

---

## 🧪 Development Highlights

### Latency Optimization
- Used **DMA** with double-buffering for seamless audio streaming  
- Hand-tuned DSP routines for cycle efficiency

### Noise Reduction
- Careful PCB layout with **ground planes**, **shielding**, and **separate analog/digital domains**

### RTOS Control
- Wrote a lightweight task scheduler to manage user I/O and DSP pipelines in real time

---

## 📁 Repository Structure

```bash
.
├── Core/                 # STM32 project files
├── Drivers/              # Peripheral drivers
├── Inc/                  # Headers
├── Src/                  # Main source files and DSP logic
├── PCB/                  # KiCad files
├── media/                # Images, videos, diagrams
└── README.md             # This file
