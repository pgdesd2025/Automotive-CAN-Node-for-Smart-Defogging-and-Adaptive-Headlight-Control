# Automotive CAN Node: Smart Defogging and Adaptive Headlight Control
🚗 Automotive CAN Node for Smart Defogging and Adaptive Headlight Control
This project presents an advanced automotive control node designed to enhance vehicle safety, driver comfort, and operational efficiency through intelligent environmental sensing and adaptive system management. Leveraging a Real-Time Operating System (RTOS) for robust multitasking, this system integrates multiple sensors to provide automated headlight control and proactive defogging capabilities, all communicating seamlessly over the Controller Area Network (CAN) bus.

✨ Project Vision
To develop a highly responsive and integrated automotive subsystem that autonomously adapts to changing environmental conditions, optimizing driver visibility and cabin clarity, thereby contributing to safer and more comfortable driving experiences.

💡 Key Features & Functionalities
🚦 Adaptive Headlight Control:
Ambient Light Sensing (LDR): A Light Dependent Resistor (LDR) precisely measures ambient light intensity. This enables automatic switching between high beam (upper) and low beam (deeper) headlights based on real-time conditions (e.g., day ☀️ to evening 🌆 or night 🌙), ensuring optimal illumination without manual intervention.

Forward Vehicle Detection (Ultrasonic Sensor): An ultrasonic sensor continuously monitors the distance to vehicles ahead. The system intelligently adjusts headlight intensity or beam pattern to prevent dazzling oncoming drivers 🚫🔆 while simultaneously enhancing visibility when following another vehicle.

Road Surface Visibility (LIS3DSH Accelerometer): The onboard LIS3DSH 3-axis accelerometer detects vibrations and sudden movements indicative of rough or uneven road surfaces 🚧. In challenging conditions, this data dynamically triggers the activation of deeper beams, providing improved illumination of immediate road hazards and enhancing driver awareness.

Dynamic Beam Management: The control logic prioritizes safety and visibility, seamlessly transitioning between beam modes based on a composite analysis of light, distance, and road conditions.

🌬️ Smart Defogging System:
Environmental Monitoring (DHT11 Sensor): A DHT11 temperature and humidity sensor continuously monitors the cabin's internal environment 🌡️💧.

Intelligent Fog Detection: By accurately calculating the dew point based on the collected temperature and humidity data, the system intelligently predicts and detects potential windshield fogging 🌫️ before it significantly impairs visibility.

Automated Defogging Activation: Upon detecting conditions conducive to fogging, the system automatically activates the vehicle's defogging mechanism, ensuring a clear windshield and maintaining optimal driver visibility.

🔗 CAN Bus Communication:
Robust Data Exchange: All sensor data (LDR readings, ultrasonic distances, accelerometer values, DHT11 temperature/humidity), control commands (headlight states, defogging activation), and system status updates are transmitted reliably via the CAN (Controller Area Network) bus protocol.

Seamless Integration: The use of CAN ensures low-latency, error-checked communication with other vehicle Electronic Control Units (ECUs), facilitating seamless integration into a larger automotive ecosystem. This allows other vehicle systems to react to or utilize the environmental data provided by this node.

🛠️ Core Technologies & Components
Microcontroller: STM32F407VGT6 (ARM Cortex-M4) Discovery Board

Real-Time Operating System (RTOS): FreeRTOS for efficient task scheduling, resource management, and concurrent operation.

Sensors:

LDR (Light Dependent Resistor) 💡

Ultrasonic Sensor (e.g., HC-SR04) 📏

LIS3DSH Accelerometer 🧭

DHT11 Temperature & Humidity Sensor 🌡️💧

Communication Protocol: CAN Bus (Controller Area Network) 🚌

Programming Language: Embedded C

Development Environment: STM32CubeIDE with HAL libraries

⭐ Benefits & Impact
Enhanced Driver Safety: Automated headlight adjustments reduce driver fatigue and improve visibility in varying conditions, while proactive defogging prevents sudden visibility loss. 🛡️

Increased Comfort: Reduces the need for manual intervention for headlights and defogging, allowing the driver to focus more on the road. 😊

Optimized Performance: Intelligent control prevents unnecessary high beam usage, potentially extending headlight lifespan and reducing power consumption. ⚡

Advanced Diagnostics & Data: Real-time sensor data transmitted over CAN can be logged and analyzed for vehicle diagnostics, performance optimization, and future feature development. 📊

Scalability & Modularity: The CAN bus architecture allows for easy integration of additional sensors or control modules in the future. ➕
