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

> 📎 Full project report (in Croatian) is available in the provided PDF file.

--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

The second part of this two-phase project involved implementing a different method for three-level SVPWM modulation.
Unlike the previous approach, this method directly determines switching sequences based on the reference vector position, while taking into account various practical constraints, including:
- Minimal on-time enforcement to ensure safe switching
- Neutral point voltage balancing to stabilize capacitor voltages
NOTE: this work was done with postdoc. Nikola Turk to whom I owe gratitude for mentorship and guidance. I contributed to the laboratory implementation and testing phases, although I did not continue with hands-on hardware verification.

Key features:
- generation of switching sequences in MATLAB
- PLECS simulation
- PI control of neutral point voltage (including anti wind-up)
- implementation of minimum on-time
- frequency analysis of the phase voltages
- Experimental verification using oscilloscope waveform analysis

Tools used:
- MATLAB
- PLECS
- C programming language
- TI Code Composer Studio
- TI System Configuration Tool (SysConfig)
- AM263x real-time MCU (Sitara family)
- Oscilloscope for waveform validation

> 📎 Full project report (in Croatian) is available in the provided PDF file.


Author:
Leonard Mikša
MSc Electrical Engineering student
Email: leonardmiksa@gmail.com
