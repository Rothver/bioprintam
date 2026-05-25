/*
 * BioPrint AM - Extrusion Helper Functions
 * Phase 6: State Machine & Extrusion Logic Library
 * 
 * Contains:
 * - Extrusion validation logic (boost system constraints)
 * - executeExtrude() - two-phase extrusion with adaptive boost
 * - executeSetup() - concentration ratio calculation
 * - executePrime() - prime position memory
 * 
 * Dependencies:
 * - config/config.h (for motor constants)
 * - libraries/motor_controller.h (for moveMotorsTimedSync)
 * - libraries/state_machine.h (for structs and enums)
 */

#ifndef EXTRUSION_HELPERS_H
#define EXTRUSION_HELPERS_H

#include "state_machine.h"

// Forward declarations (functions implemented in main sketch or motor_controller.h)
extern bool moveMotorsTimedSync(long target1, long target2, float speed1_mms, float speed2_mms, float duration_sec);
extern long arduino_pos1;
extern long arduino_pos2;
extern SystemState current_state;

// Motor constants - pulled from config.h in actual use
// These are referenced here for documentation; actual use pulls from config.h
#define MM_PER_ML       6.05f
#define MM_PER_STEP     0.00605f
#define STEPS_PER_ML    1000.0f
#define LOAD_POSITION   15000L

// ==================== CONFIGURATION HELPERS ====================

/*
 * executeSetup()
 * 
 * Calculates motor ratios based on user-selected concentration percentage
 * Concentration represents the mix of Material1 : Material2
 * 
 * Example:
 *   concentration = 50% → equal parts M1 and M2 (ratio1=0.5, ratio2=0.5)
 *   concentration = 75% → 75% M1, 25% M2 (ratio1=0.75, ratio2=0.25)
 * 
 * Args:
 *   config: SystemConfig struct to populate
 *   syringe_vol1, syringe_vol2: Loaded syringe volumes (mL)
 *   concentration: User selection 0-100%
 * 
 * Returns: true if successful
 */
bool executeSetup(SystemConfig &config, float syringe_vol1, float syringe_vol2, int concentration) {
  // Normalize concentration to ratio
  float conc_frac = concentration / 100.0f;
  
  config.ratio1 = conc_frac;
  config.ratio2 = 1.0f - conc_frac;
  
  config.syringe_vol1 = syringe_vol1;
  config.syringe_vol2 = syringe_vol2;
  
  config.remaining1 = config.syringe_vol1;
  config.remaining2 = config.syringe_vol2;
  config.dispensed1 = 0.0f;
  config.dispensed2 = 0.0f;
  
  Serial.println("\n=== SETUP COMPLETE ===");
  Serial.print("Syringes: M1=");
  Serial.print(config.syringe_vol1, 1);
  Serial.print("mL, M2=");
  Serial.print(config.syringe_vol2, 1);
  Serial.println("mL");
  Serial.print("Concentration: ");
  Serial.print(concentration);
  Serial.print("% → Ratios: M1=");
  Serial.print(config.ratio1 * 100, 0);
  Serial.print("%, M2=");
  Serial.print(config.ratio2 * 100, 0);
  Serial.println("%");
  
  return true;
}

/*
 * executePrime()
 * 
 * Records the current motor position as the "prime position"
 * This is typically called after a small test extrusion to establish
 * a reference point for the primed system
 * 
 * Args:
 *   config: SystemConfig struct to update with prime positions
 * 
 * Returns: true if successful
 */
bool executePrime(SystemConfig &config) {
  config.prime_pos1 = arduino_pos1;
  config.prime_pos2 = arduino_pos2;
  
  current_state = PRIMED;
  
  Serial.println("\n=== PRIME COMPLETE ===");
  Serial.print("Prime positions saved: M1=");
  Serial.print(config.prime_pos1);
  Serial.print(", M2=");
  Serial.println(config.prime_pos2);
  
  return true;
}

// ==================== EXTRUSION VALIDATION ====================

/*
 * validateExtrusion()
 * 
 * Validates if the requested extrusion is mechanically feasible
 * Checks motor speed constraints and boost system limits
 * 
 * Boost System Constraints:
 *   - Motors below 1.0 mm/s require a "boost" phase to overcome static friction
 *   - Boost phase runs at 2.0 mm/s for 1.0 second
 *   - Minimum distance for boost: 2.2 mm
 *   - After boost, phase 2 speed must exceed 0.3 mm/s minimum
 *   - Without boost, speed must exceed 0.3 mm/s minimum
 * 
 * Args:
 *   config: Current system configuration with ratios and remaining volumes
 *   total_volume_ml: Total output volume requested (mL)
 *   print_time_sec: Time to complete extrusion (seconds)
 * 
 * Returns: ExtrusionValidation struct with is_valid, error_message, and suggestion
 */
ExtrusionValidation validateExtrusion(SystemConfig &config, float total_volume_ml, float print_time_sec) {
  ExtrusionValidation result;
  result.is_valid = true;
  result.error_message = "";
  result.suggestion = "";
  
  // Boost system constants
  const float SLOW_SPEED_THRESHOLD = 1.0f;  // mm/s - below this triggers boost
  const float BOOST_SPEED = 2.0f;           // mm/s - speed during boost phase
  const float BOOST_DURATION = 1.0f;        // seconds - duration of boost
  const float MIN_VIABLE_SPEED = 0.3f;      // mm/s - below this motor stalls
  const float MIN_DISTANCE_FOR_BOOST = BOOST_SPEED * BOOST_DURATION * 1.1f;  // 2.2mm with safety margin
  
  // Calculate volumes for each motor based on ratio
  float vol1_to_dispense = total_volume_ml * config.ratio1;
  float vol2_to_dispense = total_volume_ml * config.ratio2;
  
  // Calculate distances in mm
  float dist1_mm = vol1_to_dispense * MM_PER_ML;
  float dist2_mm = vol2_to_dispense * MM_PER_ML;
  
  // Calculate speeds without boost
  float speed1_mms = dist1_mm / print_time_sec;
  float speed2_mms = dist2_mm / print_time_sec;
  
  // Check each motor
  for (int motor = 1; motor <= 2; motor++) {
    float dist_mm = (motor == 1) ? dist1_mm : dist2_mm;
    float speed_mms = (motor == 1) ? speed1_mms : speed2_mms;
    float vol_ml = (motor == 1) ? vol1_to_dispense : vol2_to_dispense;
    float ratio = (motor == 1) ? config.ratio1 : config.ratio2;
    String motor_name = (motor == 1) ? "M1" : "M2";
    
    // Skip if motor doesn't need to move
    if (vol_ml < 0.01f) continue;
    
    // Check if motor will need boost
    bool needs_boost = (speed_mms < SLOW_SPEED_THRESHOLD && print_time_sec > BOOST_DURATION);
    
    if (needs_boost) {
      // CASE 1: Needs boost but distance too short for boost phase
      if (dist_mm < MIN_DISTANCE_FOR_BOOST) {
        result.is_valid = false;
        result.error_message = motor_name + " boost failure";
        result.error_message += "\nDistance too short for boost";
        result.error_message += "\nNeeds: " + String(MIN_DISTANCE_FOR_BOOST, 1) + "mm";
        result.error_message += "\nHas: " + String(dist_mm, 1) + "mm";
        
        // Calculate suggestions
        float min_vol_needed = MIN_DISTANCE_FOR_BOOST / MM_PER_ML;
        float min_total_vol = min_vol_needed / ratio;
        
        // Time to avoid needing boost (speed >= 1.0 mm/s)
        float max_time_no_boost = dist_mm / SLOW_SPEED_THRESHOLD;
        
        result.suggestion = "Increase volume to >" + String(min_total_vol, 1) + "mL";
        result.suggestion += "\nOR decrease time to <" + String(max_time_no_boost, 0) + " sec";
        return result;
      }
      
      // CASE 2: Boost works, but phase 2 speed too slow
      float remaining_time = print_time_sec - BOOST_DURATION;
      float phase1_dist = BOOST_SPEED * BOOST_DURATION;
      float phase2_dist = dist_mm - phase1_dist;
      float phase2_speed = phase2_dist / remaining_time;
      
      if (phase2_speed < MIN_VIABLE_SPEED) {
        result.is_valid = false;
        result.error_message = motor_name + " phase 2 too slow";
        result.error_message += "\nAfter boost: " + String(phase2_speed, 2) + " mm/s";
        result.error_message += "\nMinimum: " + String(MIN_VIABLE_SPEED, 1) + " mm/s";
        
        // Calculate time needed for phase 2 to be viable
        float min_phase2_time = phase2_dist / MIN_VIABLE_SPEED;
        float max_total_time = BOOST_DURATION + min_phase2_time;
        
        // Calculate volume needed for phase 2 to be viable at current time
        float min_phase2_dist = MIN_VIABLE_SPEED * remaining_time;
        float min_total_dist = phase1_dist + min_phase2_dist;
        float min_vol_needed = min_total_dist / MM_PER_ML;
        float min_total_vol = min_vol_needed / ratio;
        
        result.suggestion = "Decrease time to <" + String(max_total_time, 0) + " sec";
        result.suggestion += "\nOR increase volume to >" + String(min_total_vol, 1) + "mL";
        return result;
      }
      
    } else {
      // CASE 3: Doesn't need boost, but speed too slow to move
      if (speed_mms < MIN_VIABLE_SPEED) {
        result.is_valid = false;
        result.error_message = motor_name + " speed too slow";
        result.error_message += "\nSpeed: " + String(speed_mms, 2) + " mm/s";
        result.error_message += "\nMinimum: " + String(MIN_VIABLE_SPEED, 1) + " mm/s";
        
        // Calculate max time for minimum viable speed
        float max_time = dist_mm / MIN_VIABLE_SPEED;
        
        result.suggestion = "Decrease time to <" + String(max_time, 0) + " sec";
        return result;
      }
    }
  }
  
  // All checks passed
  return result;
}

// ==================== EXTRUSION EXECUTION ====================

/*
 * executeExtrude()
 * 
 * Main extrusion control function with adaptive two-phase boost for slow motors
 * 
 * Algorithm:
 *   1. Calculates per-motor volumes from ratio and total volume
 *   2. Validates extrusion feasibility (via validateExtrusion)
 *   3. Checks available syringe volumes
 *   4. Determines if boost is needed (either motor < 1.0 mm/s)
 *   5. If boost needed: executes BOTH motors with two phases
 *      - Phase 1: Fixed 1.0 second at higher speed to overcome friction
 *      - Phase 2: Remaining time at calculated speed
 *   6. If no boost: single-phase movement at calculated speeds
 *   7. Updates config.dispensed and config.remaining volumes
 * 
 * Args:
 *   config: System configuration (ratios, syringe volumes, tracking)
 *   total_volume_ml: Total volume to extrude (distributed by ratio)
 *   print_time_sec: Time to complete extrusion (2-15 seconds typical)
 * 
 * Returns: true if extrusion completed successfully, false if error
 * 
 * Sets current_state to EXTRUDING during execution, COMPLETE on success, SAFE_MODE on error
 */
bool executeExtrude(SystemConfig &config, float total_volume_ml, float print_time_sec) {
  current_state = EXTRUDING;
  
  Serial.println("\n=== EXTRUSION START ===");
  Serial.print("Total volume requested: ");
  Serial.print(total_volume_ml, 2);
  Serial.print(" mL over ");
  Serial.print(print_time_sec, 1);
  Serial.println(" seconds");
  
  // Calculate volumes based on ratio
  // CRITICAL: Requested volume = TOTAL OUTPUT, distributed by ratio
  float vol1_to_dispense = total_volume_ml * config.ratio1;
  float vol2_to_dispense = total_volume_ml * config.ratio2;
  
  Serial.print("Ratio: M1=");
  Serial.print(config.ratio1 * 100, 0);
  Serial.print("% (");
  Serial.print(vol1_to_dispense, 3);
  Serial.print("mL), M2=");
  Serial.print(config.ratio2 * 100, 0);
  Serial.print("% (");
  Serial.print(vol2_to_dispense, 3);
  Serial.println("mL)");
  
  // SAFETY CHECK: Ensure sufficient volume in each syringe
  if (vol1_to_dispense > config.remaining1) {
    Serial.print("ERROR: M1 requires ");
    Serial.print(vol1_to_dispense, 2);
    Serial.print("mL but only ");
    Serial.print(config.remaining1, 2);
    Serial.println("mL remaining!");
    current_state = COMPLETE;
    return false;
  }
  
  if (vol2_to_dispense > config.remaining2) {
    Serial.print("ERROR: M2 requires ");
    Serial.print(vol2_to_dispense, 2);
    Serial.print("mL but only ");
    Serial.print(config.remaining2, 2);
    Serial.println("mL remaining!");
    current_state = COMPLETE;
    return false;
  }
  
  // Calculate speeds to finish at same time
  // speed = distance / time
  float dist1_mm = vol1_to_dispense * MM_PER_ML;
  float dist2_mm = vol2_to_dispense * MM_PER_ML;
  
  float speed1_mms = dist1_mm / print_time_sec;
  float speed2_mms = dist2_mm / print_time_sec;
  
  Serial.print("Distance: M1=");
  Serial.print(dist1_mm, 2);
  Serial.print("mm, M2=");
  Serial.print(dist2_mm, 2);
  Serial.println("mm");
  
  Serial.print("Initial calculated speeds: M1=");
  Serial.print(speed1_mms, 3);
  Serial.print(" mm/s, M2=");
  Serial.print(speed2_mms, 3);
  Serial.println(" mm/s");
  
  // ACCELERATION RAMP for slow motors (< 1.0 mm/s)
  // To overcome static friction, boost slow motors at startup
  const float SLOW_SPEED_THRESHOLD = 1.0f;
  const float BOOST_SPEED = 2.0f;
  const float BOOST_DURATION = 1.0f;
  
  bool motor1_needs_boost = (speed1_mms < SLOW_SPEED_THRESHOLD && print_time_sec > BOOST_DURATION);
  bool motor2_needs_boost = (speed2_mms < SLOW_SPEED_THRESHOLD && print_time_sec > BOOST_DURATION);
  
  // If EITHER motor needs boost, BOTH motors do two-phase movement
  bool use_two_phase = (motor1_needs_boost || motor2_needs_boost) && (print_time_sec > BOOST_DURATION);
  
  float phase1_speed_m1, phase2_speed_m1;
  float phase1_speed_m2, phase2_speed_m2;
  long phase1_steps_m1, phase2_steps_m1;
  long phase1_steps_m2, phase2_steps_m2;
  
  if (use_two_phase) {
    Serial.println("=== TWO-PHASE MOVEMENT ===");
    
    // BOTH motors execute Phase 1 and Phase 2 together
    // Phase 1: BOOST_DURATION seconds
    // Phase 2: Remaining time
    
    float remaining_time = print_time_sec - BOOST_DURATION;
    
    // Motor 1 calculations
    if (motor1_needs_boost) {
      // M1 gets boost in Phase 1
      phase1_speed_m1 = BOOST_SPEED;
      float phase1_dist_m1 = BOOST_SPEED * BOOST_DURATION;
      float phase2_dist_m1 = dist1_mm - phase1_dist_m1;
      phase2_speed_m1 = phase2_dist_m1 / remaining_time;
      
      phase1_steps_m1 = (long)(phase1_dist_m1 / MM_PER_STEP);
      phase2_steps_m1 = (long)(phase2_dist_m1 / MM_PER_STEP);
      
      Serial.print("M1 BOOST: Phase1=");
      Serial.print(phase1_speed_m1, 2);
      Serial.print("mm/s (");
      Serial.print(phase1_dist_m1, 2);
      Serial.print("mm), Phase2=");
      Serial.print(phase2_speed_m1, 3);
      Serial.print("mm/s (");
      Serial.print(phase2_dist_m1, 2);
      Serial.println("mm)");
    } else {
      // M1 doesn't need boost - calculate speeds to match total distance/time
      // Distribute distance across two phases proportionally
      float phase1_dist_m1 = dist1_mm * (BOOST_DURATION / print_time_sec);
      float phase2_dist_m1 = dist1_mm - phase1_dist_m1;
      
      phase1_speed_m1 = phase1_dist_m1 / BOOST_DURATION;
      phase2_speed_m1 = phase2_dist_m1 / remaining_time;
      
      phase1_steps_m1 = (long)(phase1_dist_m1 / MM_PER_STEP);
      phase2_steps_m1 = (long)(phase2_dist_m1 / MM_PER_STEP);
      
      Serial.print("M1 NORMAL: Phase1=");
      Serial.print(phase1_speed_m1, 2);
      Serial.print("mm/s (");
      Serial.print(phase1_dist_m1, 2);
      Serial.print("mm), Phase2=");
      Serial.print(phase2_speed_m1, 3);
      Serial.print("mm/s (");
      Serial.print(phase2_dist_m1, 2);
      Serial.println("mm)");
    }
    
    // Motor 2 calculations
    if (motor2_needs_boost) {
      // M2 gets boost in Phase 1
      phase1_speed_m2 = BOOST_SPEED;
      float phase1_dist_m2 = BOOST_SPEED * BOOST_DURATION;
      float phase2_dist_m2 = dist2_mm - phase1_dist_m2;
      phase2_speed_m2 = phase2_dist_m2 / remaining_time;
      
      phase1_steps_m2 = (long)(phase1_dist_m2 / MM_PER_STEP);
      phase2_steps_m2 = (long)(phase2_dist_m2 / MM_PER_STEP);
      
      Serial.print("M2 BOOST: Phase1=");
      Serial.print(phase1_speed_m2, 2);
      Serial.print("mm/s (");
      Serial.print(phase1_dist_m2, 2);
      Serial.print("mm), Phase2=");
      Serial.print(phase2_speed_m2, 3);
      Serial.print("mm/s (");
      Serial.print(phase2_dist_m2, 2);
      Serial.println("mm)");
    } else {
      // M2 doesn't need boost - calculate speeds to match total distance/time
      float phase1_dist_m2 = dist2_mm * (BOOST_DURATION / print_time_sec);
      float phase2_dist_m2 = dist2_mm - phase1_dist_m2;
      
      phase1_speed_m2 = phase1_dist_m2 / BOOST_DURATION;
      phase2_speed_m2 = phase2_dist_m2 / remaining_time;
      
      phase1_steps_m2 = (long)(phase1_dist_m2 / MM_PER_STEP);
      phase2_steps_m2 = (long)(phase2_dist_m2 / MM_PER_STEP);
      
      Serial.print("M2 NORMAL: Phase1=");
      Serial.print(phase1_speed_m2, 2);
      Serial.print("mm/s (");
      Serial.print(phase1_dist_m2, 2);
      Serial.print("mm), Phase2=");
      Serial.print(phase2_speed_m2, 3);
      Serial.print("mm/s (");
      Serial.print(phase2_dist_m2, 2);
      Serial.println("mm)");
    }
  }
  
  // Calculate target positions (extrusion moves DOWN toward 1600)
  long steps1 = (long)(vol1_to_dispense * STEPS_PER_ML);
  long steps2 = (long)(vol2_to_dispense * STEPS_PER_ML);
  
  long target1 = arduino_pos1 - steps1;
  long target2 = arduino_pos2 - steps2;
  
  Serial.print("Current positions: M1=");
  Serial.print(arduino_pos1);
  Serial.print(", M2=");
  Serial.println(arduino_pos2);
  
  Serial.print("Steps to move: M1=");
  Serial.print(steps1);
  Serial.print(", M2=");
  Serial.println(steps2);
  
  Serial.print("Target positions: M1=");
  Serial.print(target1);
  Serial.print(", M2=");
  Serial.println(target2);
  
  // Safety check: don't go below 1600 (0mL position)
  if (target1 < 1600 || target2 < 1600) {
    Serial.println("ERROR: Would go below 0mL (position 1600)!");
    current_state = COMPLETE;
    return false;
  }
  
  // Execute movement with acceleration ramp if needed
  Serial.println("Starting synchronized movement...");
  
  if (use_two_phase) {
    // TWO-PHASE MOVEMENT - BOTH motors execute phases simultaneously
    Serial.println("=== EXECUTING SYNCHRONIZED TWO-PHASE MOVEMENT ===");
    
    // PHASE 1: Both motors move for exactly BOOST_DURATION
    long phase1_target_m1 = arduino_pos1 - phase1_steps_m1;
    long phase1_target_m2 = arduino_pos2 - phase1_steps_m2;
    
    Serial.print("Phase 1: ");
    Serial.print(BOOST_DURATION, 1);
    Serial.println(" seconds");
    Serial.print("  M1: ");
    Serial.print(arduino_pos1);
    Serial.print(" → ");
    Serial.print(phase1_target_m1);
    Serial.print(" @ ");
    Serial.print(phase1_speed_m1, 3);
    Serial.println(" mm/s");
    Serial.print("  M2: ");
    Serial.print(arduino_pos2);
    Serial.print(" → ");
    Serial.print(phase1_target_m2);
    Serial.print(" @ ");
    Serial.print(phase1_speed_m2, 3);
    Serial.println(" mm/s");
    
    // Execute Phase 1 with time sync
    if (!moveMotorsTimedSync(phase1_target_m1, phase1_target_m2, phase1_speed_m1, phase1_speed_m2, BOOST_DURATION)) {
      Serial.println("ERROR: Phase 1 failed!");
      current_state = SAFE_MODE;
      return false;
    }
    
    Serial.println("Phase 1 complete - both motors synchronized");
    
    // PHASE 2: Both motors move for exactly remaining time
    long phase2_target_m1 = target1;  // Final target
    long phase2_target_m2 = target2;  // Final target
    
    float remaining_time = print_time_sec - BOOST_DURATION;
    
    Serial.print("Phase 2: ");
    Serial.print(remaining_time, 1);
    Serial.println(" seconds");
    Serial.print("  M1: ");
    Serial.print(arduino_pos1);
    Serial.print(" → ");
    Serial.print(phase2_target_m1);
    Serial.print(" @ ");
    Serial.print(phase2_speed_m1, 3);
    Serial.println(" mm/s");
    Serial.print("  M2: ");
    Serial.print(arduino_pos2);
    Serial.print(" → ");
    Serial.print(phase2_target_m2);
    Serial.print(" @ ");
    Serial.print(phase2_speed_m2, 3);
    Serial.println(" mm/s");
    
    // Execute Phase 2 with time sync
    if (!moveMotorsTimedSync(phase2_target_m1, phase2_target_m2, phase2_speed_m1, phase2_speed_m2, remaining_time)) {
      Serial.println("ERROR: Phase 2 failed!");
      current_state = SAFE_MODE;
      return false;
    }
    
    Serial.println("Phase 2 complete - BOTH MOTORS FINISHED SIMULTANEOUSLY");
    
  } else {
    // SINGLE-PHASE MOVEMENT (no boost needed)
    Serial.println("=== EXECUTING SINGLE-PHASE MOVEMENT ===");
    
    Serial.print("Duration: ");
    Serial.print(print_time_sec, 1);
    Serial.println(" seconds");
    Serial.print("  M1 speed: ");
    Serial.print(speed1_mms, 3);
    Serial.println(" mm/s");
    Serial.print("  M2 speed: ");
    Serial.print(speed2_mms, 3);
    Serial.println(" mm/s");
    
    // Execute with time sync
    if (!moveMotorsTimedSync(target1, target2, speed1_mms, speed2_mms, print_time_sec)) {
      Serial.println("ERROR: Movement failed!");
      current_state = SAFE_MODE;
      return false;
    }
    
    Serial.println("Movement complete - both motors synchronized");
  }
  
  // Update tracking
  config.dispensed1 += vol1_to_dispense;
  config.dispensed2 += vol2_to_dispense;
  config.remaining1 -= vol1_to_dispense;
  config.remaining2 -= vol2_to_dispense;
  
  Serial.println("=== EXTRUSION COMPLETE ===");
  Serial.print("Total dispensed: M1=");
  Serial.print(config.dispensed1, 2);
  Serial.print("mL, M2=");
  Serial.print(config.dispensed2, 2);
  Serial.println("mL");
  
  Serial.print("Remaining: M1=");
  Serial.print(config.remaining1, 2);
  Serial.print("mL, M2=");
  Serial.print(config.remaining2, 2);
  Serial.println("mL\n");
  
  current_state = COMPLETE;
  return true;
}

#endif // EXTRUSION_HELPERS_H
