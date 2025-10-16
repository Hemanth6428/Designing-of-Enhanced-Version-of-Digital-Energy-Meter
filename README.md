# Designing-of-Enhanced-Version-of-Digital-Energy-Meter

**1. Objective:**
To design, simulate, and implement analog voltage and current sensing circuits for integration with the TI TMS320F28335 DSP microcontroller for real-time measurement and processing.

**2. Design and Simulation**

Designed and simulated the analog voltage and current conditioning circuits in Proteus.

The complete circuit consists of four major parts:

1) Voltage conditioning circuit

2) Current conditioning circuit

3) Power supply section for IC TL082

4) Power supply section for aurdino/microcontroller

**3. Circuit Description**

Op-Amp Used: TL082 (Dual Op-Amp IC)

First op-amp: Configured as a differential amplifier for voltage sensing.

Second op-amp: Used as a buffer (voltage follower) to isolate the signal and prevent loading.

Current Sensing:

Sensor Used: LA55P Hall Effect current sensor.

The sensor output is processed using an inverting op-amp configuration followed by a buffer stage for stable output.

Power Supply:

IC 7815 and IC 7915 are used to provide regulated +15 V and –15 V supply to the TL082 op-amp.

The circuit operates with three ground terminals (analog, digital, and power ground) for noise isolation.

**4. PCB Design and Fabrication**

Created a custom PCB in Proteus using the project file “pcb_inverting_4.pdsprj”.

The PCB was fabricated through etching and drilling processes.

The design includes reduced jumper connections (only one jumper) and a single common ground point to improve signal integrity.

**5. Microcontroller and Programming**
Initially, the circuit was tested with Arduino as the microcontroller for the smart energy meter prototype.

Later, migrated to TI TMS320F28335 DSP for:

Real-time data acquisition

ADC conversion and processing

LED output display of measured parameters

Ongoing work involves achieving higher accuracy and noise reduction with the DSP-based setup.

**6. Files and Documentation**

Detailed modeling and explanation of voltage and current conditioning circuits are documented.

Separate Proteus simulation files were created for each module.

The final PCB design file: pcb_inverting_4.pdsprj.

