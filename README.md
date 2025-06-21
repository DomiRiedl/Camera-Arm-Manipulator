# Camera Arm Manipulator — Bachelor Thesis

This repository contains the code, documentation, and resources developed as part of the Bachelor thesis *Development and evaluation of a robotic
arm for endoscopic cameras with intuitive manipulation techniques* at the Management Center Innsbruck (MCI).

##  Overview

The aim of this project was to develop a compact robotic camera arm that completes the existing **Desktop Teleoperated Training System** at MCI. This system combines:

- A **double-parallelogram mechanism** manipulated by two identical belt systems
- An additional Z-axis using a lead-screw for up-and-down camera motion
- An intuitive control concept based on joystick input with two modes:
  - **Absolute Mode:** Real-time tracking of joystick movements
  - **Incremental Mode:** Stepwise positioning with defined angles

## 🛠️ Repository Structure

- Robotic_arm_Control.ino -> Main code to control the System
- Evaluation_Setup.ino -> Code for Measurements and Evaluation
- GUI_Bachelorthesis.py -> Python code for User Interface

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
   git clone https://github.com/DomiRiedl/Camera-Arm-Manipulator.git

2. Upload Robotic_arm_control.ino to your Microcontroller
3. Start the User Interface using the Code GUI_Bachelorthesis.py
4. Connect to your System
5. "Start" or "Skip" the Calibration process
6. Choose control Method
7. System is read for Operation

## 📬 Contact & Contribution

Found a bug? Have an idea to improve the system?  
- Open an **Issue**  
- Submit a **Pull Request**  
- Or reach out directly via **GitHub**
   
