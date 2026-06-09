/*
 * BioPrint AM - State Machine & Data Structures
 * Phase 6: State Machine & Extrusion Logic Library
 * 
 * Defines:
 * - All system states
 * - Configuration and tracking structures
 * - Page/UI enumerations
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

// ==================== SYSTEM STATES ====================
enum SystemState {
  UNINITIALIZED,      // System just powered on
  LOAD,               // Waiting for syringe loading
  SETUP,              // Configuration setup (temp, volume, concentration, speed)
  PRIMED,             // Prime run complete, system ready for extrusion
  READY,              // Temperature and setup complete, waiting for extrusion command
  EXTRUDING,          // Currently performing extrusion cycle
  COMPLETE,           // Extrusion complete, waiting for next action
  SAFE_MODE           // Error state - motors halted, heaters off
};

// ==================== UI PAGE DEFINITIONS ====================
enum Page {
  MOTOR_ZERO_CHECK,
  RETRACTING_TO_ZERO,
  CALIBRATION_IN_PROGRESS,
  WELCOME,
  HOME,
  TEMPERATURE_PAGE,
  VOLUME1,
  VOLUME2,
  CONCENTRATION,
  SPEED,
  PRINT_CONFIRM,
  LOADING_PAGE,
  HOMING_PAGE,
  LOADING_SYRINGES,
  WAITING_FOR_SYRINGES,
  TEMP_READY,
  EXTRUSION_SETUP,
  READY_TO_PRINT,
  PRINTING_PAGE,
  POST_EXTRUSION_OPTIONS,
  PRINT_DONE,
  SHUTDOWN_CONFIRM,
  SHUTTING_DOWN,
  SHUTDOWN_COMPLETE,
  ERROR_PAGE
};

// ==================== SYSTEM CONFIGURATION STRUCT ====================
/*
 * Tracks all operational parameters and cycle state
 * Populated by UI user selections and updated during extrusion
 */
struct SystemConfig {
  // Concentration ratios (normalized, sum to 1.0)
  float ratio1 = 1.0f;      // Motor 1 fraction of total output
  float ratio2 = 1.0f;      // Motor 2 fraction of total output
  
  // Syringe capacities (mL)
  float syringe_vol1 = 10.0f;
  float syringe_vol2 = 10.0f;
  
  // Per-cycle extrusion amount (mL)
  float extrude_vol1 = 1.0f;
  
  // Speed control (mm/s)
  float extrude_speed = 2.0f;
  
  // Volume tracking (cumulative per session, mL)
  float dispensed1 = 0.0f;    // Total dispensed from motor 1
  float dispensed2 = 0.0f;    // Total dispensed from motor 2
  
  // Remaining volume in syringes (mL)
  float remaining1 = 10.0f;
  float remaining2 = 10.0f;
  
  // Prime position memory (step count)
  // Saved after priming for reference
  long prime_pos1 = 0;        // Will be set to LOAD_POSITION after prime
  long prime_pos2 = 0;        // Will be set to LOAD_POSITION after prime
};

// ==================== EXTRUSION VALIDATION STRUCT ====================
/*
 * Result of validateExtrusion() - indicates feasibility of requested extrusion
 * and provides diagnostic messages for GUI error display
 */
struct ExtrusionValidation {
  bool is_valid;              // true if extrusion can proceed safely
  String error_message;       // Human-readable error description (multi-line)
  String suggestion;          // Suggested adjustment to parameters
};

// ==================== CYCLE TRACKING STRUCT ====================
/*
 * Optional: tracks progress within multi-cycle print jobs
 * (Not heavily used in current firmware, but included for future enhancement)
 */
struct CycleTracking {
  float cycle_start_dispensed1 = 0.0f;  // Cumulative dispensed at cycle start
  float cycle_start_dispensed2 = 0.0f;
  
  float cycle_target_vol1 = 0.0f;       // Target volume for current cycle
  float cycle_target_vol2 = 0.0f;
  
  unsigned long cycle_start_time = 0;   // Timestamp when cycle began
};

// ==================== UI OPTIONS ====================
// Pre-defined choices for user selections (used in menu pages)
// These are global constants to maintain consistency across UI rendering

const int TEMP_OPTIONS[] = {30, 32, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80};
const int NUM_TEMP_OPTIONS = 12;

const int CONC_OPTIONS[] = {0, 25, 50, 75, 100};
const int NUM_CONC_OPTIONS = 5;

const int VOL_OPTIONS[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
const int NUM_VOL_OPTIONS = 10;

const int SPEED_OPTIONS[] = {1, 2, 3, 4, 5};
const int NUM_SPEED_OPTIONS = 5;

#endif // STATE_MACHINE_H
