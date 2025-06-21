# Robot-Assisted Camera Arm — Bachelor Thesis

This repository contains the code, documentation, and resources developed as part of the Bachelor thesis *Design and Control of a Robot-Assisted Camera Arm with Intuitive Control Methods* at the Management Center Innsbruck (MCI).

## 📑 Overview

The aim of this project was to develop a compact robotic camera arm that completes the existing **Desktop Teleoperated Training System** at MCI. This system combines:

- A **double-parallelogram mechanism** manipulated by two identical belt systems
- An additional Z-axis using a lead-screw for up-and-down camera motion
- An intuitive control concept based on joystick input with two modes:
  - **Absolute Mode:** Real-time tracking of joystick movements
  - **Incremental Mode:** Stepwise positioning with defined angles

## 🛠️ Repository Structure


## ⚙️ Hardware

- **ESP32-WROOM-32E** microcontroller
- Stepper motors with integrated encoders
- DC motor with trapezoidal lead screw
- Potentiometers for pitch and roll angle feedback
- Power supply: 12V laboratory power supply unit

## 💻 Software

- Arduino code for motor control loop and sensor reading
- MATLAB scripts for kinematics, data processing, and plotting
- Serial communication between ESP32 and control PC

## 🚀 Getting Started

1. Clone this repository:
   ```bash
   git clone https://github.com/YourUsername/YourRepositoryName.git
