# bioprintam
BioPrintAM - Senior Design Project @ GMU -- Affiliated with **GMU's Bioengineering Program**

**Advisors:**
- Dr. Remi Veneziano
- Dr. Quentin Sanders
- Mr. Randolph Warren (Randy)

---

## Project Structure

This repository contains the complete BioPrintAM firmware, libraries, and supporting scripts. The codebase has been refactored to use a modular library structure with centralized configuration and minimal code duplication.

### Directory Organization

#### `config/`
Centralized configuration and calibration constants used across all firmware and scripts:
- **`config.h`** - Motor calibration (STEPS_PER_ML, MAX_VOLUME), thermistor calibration (beta coefficients, nominal values), I2C addresses, ADC resolution (12-bit), and system constants

#### `libraries/`
Shared libraries used by firmware sketches:
- **`thermistor_sensor.h`** - Unified thermistor reading using Steinhart-Hart formula; supports HEAT_MAT_SENSOR and SYSTEM_SENSOR types
- **`motor_controller.h`** - Motor control helpers: movement, positioning, speed conversion (mL↔steps, mm/s↔TIC units)
- **`pid_controller.h`** - PID-based temperature control for dual-zone heating system
- **`ui_system.h`** - All UI drawing functions (25+ pages) and touch handlers for GigaDisplay
- **`state_machine.h`** - System state definitions, enums, and configuration structures
- **`extrusion_helpers.h`** - Extrusion validation with boost logic for slow motor speeds

#### `firmware/`
Canonical Arduino sketches (use these for compilation and deployment):
- **`bioprint_integrated_canonical.ino`** - Main integrated firmware with motor control, temperature monitoring, and UI
  - **Variants:**
    - Standard (default): Full temperature validation required
    - NO_TEMP_REQ: Uncomment `#define SKIP_TEMP_VALIDATION` at line ~32 for temperature bypass
- **`motor_controls_canonical.ino`** - Standalone motor control CLI interface for testing and calibration

#### Support Scripts
- **`heatchamber.ino`** - Heat chamber testing (reads 4 thermistors, outputs CSV)
- **`insulationread.ino`** - Insulation testing (reads 2 system thermistors, outputs CSV)
- **`heatchamber.m`** - MATLAB script for real-time thermistor visualization
- **`insulationtest.m`** - MATLAB script for insulation test data visualization
- **`Raspi-HomeScreen.py`** - Python/Kivy UI for Raspberry Pi (separate from Arduino)

---

## Hardware

- **Microcontroller:** ARDUINO GIGA R1 WIFI
- **Display:** ARDUINO GIGA DISPLAY SHIELD (touch-enabled)
- **Motor Drivers:** TIC T825 Stepper Motor Drivers (2×, I2C addresses 0x0E and 0x0F)
- **Motors:** ACTUONIX P8-ST 100MM Linear Stepper Motors (2×)
- **Heating:** MOSFET-controlled heat mat on Pin 9 (PWM)
- **Temperature Sensors:** 4× 10K NTC Thermistors (Steinhart-Hart calibrated)

---

## Compilation & Deployment

### Arduino Setup
1. Install Arduino IDE
2. Select Board: **Arduino Giga R1 WiFi**
3. Install required libraries via Library Manager:
   - Tic Stepper Motor Driver Arduino Library (Pololu)
   - Arduino_GigaDisplay_GFX
   - Arduino_GigaDisplayTouch

### Build Standard Firmware
```
Open: firmware/bioprint_integrated_canonical.ino
Compile and upload
```

### Build NO_TEMP_REQ Variant
```
Open: firmware/bioprint_integrated_canonical.ino
Uncomment line ~32: #define SKIP_TEMP_VALIDATION
Compile and upload
```

### Motor Control Standalone
```
Open: firmware/motor_controls_canonical.ino
Compile and upload
Access via Serial Monitor (115200 baud)
Commands: HOME, SETUP, PRIME, EXTRUDE, STATUS, TERMINATE, STOP, HELP
```

---

## Thermistor Calibration

All thermistor readings are centralized in `libraries/thermistor_sensor.h` using the Steinhart-Hart formula:

- **Heat Mat Sensors** (A0, A1): β = 3435K, R₀ = 10K @ 25°C
- **System Sensors** (A2, A3): β = 3950K, R₀ = 10K @ 25°C
- **Series Resistance:** 10K ohms
- **ADC Resolution:** 12-bit (0-4095)

Calibration constants are defined in `config/config.h`. To modify, update the values there and recompile all sketches.

---

## Motor Calibration

- **Steps per mL:** 1000 steps/mL
- **Motor Addresses:** 0x0E (Motor 1), 0x0F (Motor 2) via I2C
- **Load Position:** 15000 steps
- **Zero Position (Offset):** 1600 steps
- **Max Extrusion Speed:** Configurable in firmware (default: 5 mm/s)

Motor calibration constants are stored in `config/config.h` for easy modification across all firmware variants.

---

## Data Logging

### Heat Chamber Testing
Run `heatchamber.ino` to log all 4 thermistors:
```
CSV Format: time_s, temp1_C, temp2_C, temp3_C, temp4_C
Saved to: thermistor_data_YYYYMMDD_HHMMSS.csv
Update Interval: 30 seconds
Visualize with: heatchamber.m (MATLAB)
```

### Insulation Testing
Run `insulationread.ino` to log system temperature sensors:
```
CSV Format: time_s, temp1_C, temp2_C
Saved to: thermistor_data_YYYYMMDD_HHMMSS.csv
Update Interval: 30 seconds
Visualize with: insulationtest.m (MATLAB)
```

---

## Refactoring Phases (Complete)

- ✅ **Phase 1:** Configuration & Constants Extraction → `config/config.h`
- ✅ **Phase 2:** Thermistor Library Extraction → `libraries/thermistor_sensor.h`
- ✅ **Phase 3:** Motor Controller Library Extraction → `libraries/motor_controller.h`
- ✅ **Phase 4:** Temperature Control Library Extraction → `libraries/pid_controller.h`
- ✅ **Phase 5:** UI System Library Extraction → `libraries/ui_system.h`
- ✅ **Phase 6:** State Machine & Extrusion Logic → `libraries/state_machine.h`, `libraries/extrusion_helpers.h`
- ✅ **Phase 7:** Consolidate Integrated Firmware Variants → `firmware/bioprint_integrated_canonical.ino` with `#define SKIP_TEMP_VALIDATION`
- ✅ **Phase 8:** Update MATLAB & Python Scripts (Documentation) → This file

---

