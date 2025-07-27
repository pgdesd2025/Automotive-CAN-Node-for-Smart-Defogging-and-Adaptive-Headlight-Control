# Automotive CAN Node: Smart Defogging and Adaptive Headlight Control
🚗 Automotive CAN Node for Smart Defogging and Adaptive Headlight Control

This project delivers an advanced automotive control node, enhancing safety, comfort, and efficiency. It integrates multiple sensors with an RTOS for automated headlight control and proactive defogging, all communicating via CAN bus.


✨ Project Vision
To create a highly responsive automotive subsystem that autonomously adapts to changing environmental conditions, optimizing driver visibility and cabin clarity for safer, more comfortable driving.


💡 Key Features & Functionalities
🚦 Adaptive Headlight Control:
Ambient Light (LDR): Auto high/low beam switching based on day ☀️, evening 🌆, or night 🌙.

Forward Vehicle (Ultrasonic): Adjusts headlights to prevent dazzling 🚫🔆 and enhance following visibility.

Road Surface (LIS3DSH): Detects rough roads 🚧, dynamically activating deeper beams for improved hazard illumination.

Dynamic Beam Management: Seamless transitions based on integrated light, distance, and road analysis.


🌬️ Smart Defogging System:
Environmental Monitoring (DHT11): Continuously tracks cabin temperature 🌡️ and humidity 💧.

Intelligent Fog Detection: Calculates dew point to predict and detect windshield fogging 🌫️.

Automated Activation: Proactively activates defogging for clear visibility.


🔗 CAN Bus Communication:
Robust Data Exchange: Reliably transmits all sensor data and control commands.

Seamless Integration: Ensures low-latency, error-checked communication with vehicle ECUs.


🛠️ Core Technologies & Components
Microcontroller: STM32F407VGT6 (ARM Cortex-M4)

RTOS: FreeRTOS

Sensors: LDR 💡, Ultrasonic 📏, LIS3DSH 🧭, DHT11 🌡️💧

Communication: CAN Bus 🚌

Language: Embedded C

Environment: STM32CubeIDE


⭐ Benefits & Impact
Enhanced Driver Safety: Automated adjustments reduce fatigue, prevent visibility loss. 🛡️

Increased Comfort: Minimizes manual intervention for lights and defogging. 😊

Optimized Performance: Prevents unnecessary high beam usage, saves power. ⚡

Advanced Diagnostics: Real-time CAN data for vehicle health analysis. 📊

Scalability: CAN architecture allows future module integration. ➕
