# Three-level-SVPWM-TI-AM2634

This project implements a real-time control algorithm for a three-level neutral-point clamped inverter using vector-based PWM.
The modulation is executed on the TI AM2634 microcontroller using TI's Code Composer Studio and SysConfig tools.
The project includes a method for decomposing three-level SVPWM into equivalent two-level logic for simplified sector mapping and gate signal generation.

Key features:
- Inverse Clarke transform + sector detection logic
- Vector mapping and projection using α-β space
- Gate signals generation using calculated duty cycles and ePWM module
- Dead-time configuration to avoid shoot-through
- Experimental verification using oscilloscope waveform analysis

Tools used:
- C programming language
- TI Code Composer Studio
- TI System Configuration Tool (SysConfig)
- AM263x real-time MCU (Sitara family)
- Oscilloscope for waveform validation

Author:
Leonard Mikša
MSc Electrical Engineering student
Email: leonardmiksa@gmail.com
