# Wiring Diagram

This document illustrates the hardware connections for the KRSBI-B Robot based on the implemented software stack.
The system uses an NVIDIA Jetson/NUC as the main computer, an Arduino Mega as the low-level controller, and a standard PC power supply or LiPo.

## 🔌 System Overview

```mermaid
graph TD
    LiPo[LiPo Battery 12V 5000mAh] --> PDB[Power Distribution Board]
    PDB --> NUC[Main PC (Jetson/NUC)]
    PDB --> MotorDriver1[MD10C Driver - M1]
    PDB --> MotorDriver2[MD10C Driver - M2]
    PDB --> MotorDriver3[MD10C Driver - M3]
    PDB --> StepDown[Buck Converter 5V 3A]
    StepDown --> Arduino[Arduino Mega 2560]

    NUC -- USB (Serial) --> Arduino
    NUC -- USB --> Camera[Webcam/RealSense]
    NUC -- USB --> Lidar[Scan Lidar (Optional)]

    Arduino -- PWM/DIR --> MotorDriver1
    Arduino -- PWM/DIR --> MotorDriver2
    Arduino -- PWM/DIR --> MotorDriver3

    MotorDriver1 --> Motor1[DC Motor w/ Encoder 1]
    MotorDriver2 --> Motor2[DC Motor w/ Encoder 2]
    MotorDriver3 --> Motor3[DC Motor w/ Encoder 3]

    Motor1 -- Encoder Phase A/B --> Arduino
    Motor2 -- Encoder Phase A/B --> Arduino
    Motor3 -- Encoder Phase A/B --> Arduino

    Arduino -- I2C (SDA/SCL) --> IMU[MPU6050]
    Arduino -- Serial/I2C --> Distance[Ultrasonic SR04 / ToF]
```

## 📍 Pin Connections (Arduino Mega 2560)

### Motor Drivers (Cytron MD10C)

| Component                | Pin Function    | Arduino Pin | Notes          |
| :----------------------- | :-------------- | :---------- | :------------- |
| **Motor 1 (Front)**      | PWM (Speed)     | D2          | Timer specific |
|                          | DIR (Direction) | D22         | Digital        |
| **Motor 2 (Rear-Right)** | PWM             | D3          | Timer specific |
|                          | DIR             | D23         | Digital        |
| **Motor 3 (Rear-Left)**  | PWM             | D4          | Timer specific |
|                          | DIR             | D24         | Digital        |

### Encoders (Quadrature)

| Component      | Signal  | Arduino Pin | External Interrupt                                                 |
| :------------- | :------ | :---------- | :----------------------------------------------------------------- |
| **Encoder M1** | Phase A | D18 (INT5)  | Required for precise counting                                      |
|                | Phase B | D19 (INT4)  |                                                                    |
| **Encoder M2** | Phase A | D20 (INT3)  |                                                                    |
|                | Phase B | D21 (INT2)  |                                                                    |
| **Encoder M3** | Phase A | D2 (INT0)   | Conflict with PWM? Use D2/D3 carefully. Mega has specific INT pins |
|                | Phase B | D3 (INT1)   |                                                                    |

_Note: Arduino Mega Interrupt pins: 2, 3, 18, 19, 20, 21. If using PWM on 2/3/4, ensure to use alternate INT pins or pin change interrupts if necessary. Re-map as needed in firmware `krsbi_comm/firmware/arduino_code.ino`._

### Sensors

| Component         | Signal  | Arduino Pin | Protocol         |
| :---------------- | :------ | :---------- | :--------------- |
| **IMU (MPU6050)** | SDA     | D20 (SDA)   | I2C (Shared bus) |
|                   | SCL     | D21 (SCL)   | I2C              |
|                   | VCC     | 5V          | Regulated        |
|                   | GND     | GND         | Common Ground    |
| **Ultrasonic**    | Trigger | D30         | Digital Out      |
|                   | Echo    | D31         | Digital In       |

## 🔋 Power Management

### Battery Specs

- **Type**: LiPo 3S (11.1V Nominal, 12.6V Max) or 4S (14.8V).
- **Capacity**: > 5000mAh recommended for match duration (10-15 mins active).
- **Safety**: Use a Battery Alarm/Buzzer set to 3.5V/cell.

### Voltage Levels

- **12V Line**: Motors, PC (via PicoPSU or DC jack if supported).
- **5V Line**: Arduino, Sensors, Logic level shifters.
- **Logic Level**: Arduino Mega is 5V logic. Encoder signals are usually 5V compatible. PC USB is 5V.
- **Grounding**: Connect ALL Grounds (GND) together (Common Ground) to prevent communication noise.

## ⚠️ Safety Notes

1.  **Fuse**: Install a 20A Fuse on the main positive terminal from the battery.
2.  **Emergency Stop**: Install a physical latching E-Stop button that cuts power to the Motor Drivers (but keeps logic/PC running if possible, or cuts main power entirely).
3.  **Isolation**: Ideally use Opto-isolators for PWM/DIR lines to protect the Arduino from motor noise.
