/*
 * BioPrint AM - Extrusion Helper Functions
 * 
 * Contains:
 * - Extrusion validation logic (boost system constraints)
 * - executeExtrude() - two-phase extrusion with adaptive boost
 * - executeSetup() - concentration ratio calculation
 * - executePrime() - prime position memory
 * 
 * Dependencies:
 * - config.h (for motor constants)
 * - motor_controller.h (for arduino_pos1/2 position tracking)
 * - state_machine.h (for structs and enums)
 */

#ifndef EXTRUSION_HELPERS_H
#define EXTRUSION_HELPERS_H

#include "config.h"
#include "state_machine.h"

// Forward declarations 
extern long arduino_pos1;
extern long arduino_pos2;
extern ExtrusionPlan extrusionPlan;

// Motor constants (MM_PER_ML, MM_PER_STEP, STEPS_PER_ML, LOAD_POSITION) come from config.h

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
  
  setState(PRIMED);
  
  Serial.println("\n=== PRIME COMPLETE ===");
  Serial.print("Prime positions saved: M1=");
  Serial.print(config.prime_pos1);
  Serial.print(", M2=");
  Serial.println(config.prime_pos2);
  
  return true;
}

// ==================== BOOST PLANNING HELPERS ====================

/*
 * True if a motor dispensing vol_ml at speed_mms needs the friction-boost phase.
 * Shared by validateExtrusion() and prepareExtrude().
 * A motor that dispenses (almost) nothing, e.g. at 0% or 100% concentration,
 * must never be boosted: it would push material it shouldn't, and its
 * "remaining distance" after the boost would be negative.
 */
inline bool motorNeedsBoost(float vol_ml, float speed_mms, float print_time_sec) {
  return (vol_ml >= MIN_DISPENSE_VOLUME_ML && speed_mms < SLOW_SPEED_THRESHOLD && print_time_sec > BOOST_DURATION);
}

// Per-motor result of splitting an extrusion into boost/phase-1 and phase-2
struct MotorPhasePlan {
  float phase1_speed;   // mm/s
  float phase2_speed;   // mm/s
  long phase1_steps;    // steps moved during phase 1
};

/*
 * Split one motor's total travel into phase 1 (BOOST_DURATION seconds) and
 * phase 2 (the remaining time), and log the split.
 *   needs_boost: phase 1 runs at BOOST_SPEED to break static friction
 *   otherwise:   travel is split proportionally to time, so both phases run
 *                at the same speed the single-phase plan would have used
 */
inline MotorPhasePlan planMotorPhases(const char* motor_name, float dist_mm, bool needs_boost, float print_time_sec) {
  float remaining_time = print_time_sec - BOOST_DURATION;
  float phase1_dist, phase2_dist;
  MotorPhasePlan plan;

  if (needs_boost) {
    plan.phase1_speed = BOOST_SPEED;
    phase1_dist = BOOST_SPEED * BOOST_DURATION;
    phase2_dist = dist_mm - phase1_dist;
    plan.phase2_speed = phase2_dist / remaining_time;
  } else {
    phase1_dist = dist_mm * (BOOST_DURATION / print_time_sec);
    phase2_dist = dist_mm - phase1_dist;
    plan.phase1_speed = phase1_dist / BOOST_DURATION;
    plan.phase2_speed = phase2_dist / remaining_time;
  }
  plan.phase1_steps = (long)(phase1_dist / MM_PER_STEP);

  Serial.print(motor_name);
  Serial.print(needs_boost ? " BOOST: Phase1=" : " NORMAL: Phase1=");
  Serial.print(plan.phase1_speed, 2);
  Serial.print("mm/s (");
  Serial.print(phase1_dist, 2);
  Serial.print("mm), Phase2=");
  Serial.print(plan.phase2_speed, 3);
  Serial.print("mm/s (");
  Serial.print(phase2_dist, 2);
  Serial.println("mm)");

  return plan;
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
    if (vol_ml < MIN_DISPENSE_VOLUME_ML) continue;
    
    // CAPACITY CASE A: not enough material left in this syringe
    float remaining_ml = (motor == 1) ? config.remaining1 : config.remaining2;
    if (vol_ml > remaining_ml) {
      result.is_valid = false;
      result.error_message = motor_name + " syringe low";
      result.error_message += "\nNeeds: " + String(vol_ml, 2) + "mL";
      result.error_message += "\nHas: " + String(remaining_ml, 2) + "mL";
      
      float max_total_vol = (ratio > 0.0f) ? (remaining_ml / ratio) : 0.0f;
      if (max_total_vol < 0.1f) {
        result.suggestion = "Reload syringes";
      } else {
        result.suggestion = "Decrease volume to <" + String(max_total_vol, 1) + "mL";
        result.suggestion += "\nOR reload syringes";
      }
      return result;
    }
    
    // CAPACITY CASE B: plunger would travel past the 0 mL position
    long current_pos = (motor == 1) ? arduino_pos1 : arduino_pos2;
    if (current_pos - (long)(vol_ml * STEPS_PER_ML) < ZERO_ML_POSITION) {
      result.is_valid = false;
      result.error_message = motor_name + " out of travel";
      result.error_message += "\nWould pass the 0 mL position";
      result.suggestion = "Reload syringes";
      result.suggestion += "\nOR decrease volume";
      return result;
    }
    
    // Check if motor will need boost
    bool needs_boost = motorNeedsBoost(vol_ml, speed_mms, print_time_sec);
    
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
 * Rejects the request (plan.ok = false, state unchanged) if validateExtrusion() fails;
 * otherwise sets current_state to EXTRUDING
 */
ExtrusionPlan prepareExtrude(SystemConfig &config, float total_volume_ml, float print_time_sec) {
  // The UI validates before printing; re-check here so an unvalidated or stale
  // request can never reach the motors. Nothing changes state on rejection.
  ExtrusionValidation check = validateExtrusion(config, total_volume_ml, print_time_sec);
  if (!check.is_valid) {
    Serial.println("ERROR: extrusion rejected by validation:");
    Serial.println(check.error_message);
    extrusionPlan.ok = false;
    return extrusionPlan;
  }
  
  setState(EXTRUDING);
  
  Serial.println("\n=== EXTRUSION START ===");
  Serial.print("Total volume requested: ");
  Serial.print(total_volume_ml, 2);
  Serial.print(" mL over ");
  Serial.print(print_time_sec, 1);
  Serial.println(" seconds");
  
  // Calculate volumes based on ratio
  // CRITICAL: Requested volume = TOTAL OUTPUT, distributed by ratio
  extrusionPlan.vol1_to_dispense = total_volume_ml * config.ratio1;
  extrusionPlan.vol2_to_dispense = total_volume_ml * config.ratio2;
  
  Serial.print("Ratio: M1=");
  Serial.print(config.ratio1 * 100, 0);
  Serial.print("% (");
  Serial.print(extrusionPlan.vol1_to_dispense, 3);
  Serial.print("mL), M2=");
  Serial.print(config.ratio2 * 100, 0);
  Serial.print("% (");
  Serial.print(extrusionPlan.vol2_to_dispense, 3);
  Serial.println("mL)");
  
  // Calculate speeds to finish at same time
  // speed = distance / time
  float dist1_mm = extrusionPlan.vol1_to_dispense * MM_PER_ML;
  float dist2_mm = extrusionPlan.vol2_to_dispense * MM_PER_ML;
  
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
  
  // ACCELERATION RAMP for slow motors (< SLOW_SPEED_THRESHOLD mm/s)
  // To overcome static friction, boost slow motors at startup
  bool motor1_needs_boost = motorNeedsBoost(extrusionPlan.vol1_to_dispense, speed1_mms, print_time_sec);
  bool motor2_needs_boost = motorNeedsBoost(extrusionPlan.vol2_to_dispense, speed2_mms, print_time_sec);
  
  // If EITHER motor needs boost, BOTH motors do two-phase movement
  extrusionPlan.use_two_phase = (motor1_needs_boost || motor2_needs_boost) && (print_time_sec > BOOST_DURATION);
  
  if (extrusionPlan.use_two_phase) {
    Serial.println("=== TWO-PHASE MOVEMENT ===");
    
    // BOTH motors execute Phase 1 (BOOST_DURATION seconds) and Phase 2 (remaining time) together
    MotorPhasePlan m1 = planMotorPhases("M1", dist1_mm, motor1_needs_boost, print_time_sec);
    MotorPhasePlan m2 = planMotorPhases("M2", dist2_mm, motor2_needs_boost, print_time_sec);
    
    extrusionPlan.phase1_target1 = arduino_pos1 - m1.phase1_steps;
    extrusionPlan.phase1_target2 = arduino_pos2 - m2.phase1_steps;

    extrusionPlan.phase1_speed_m1 = m1.phase1_speed;
    extrusionPlan.phase1_speed_m2 = m2.phase1_speed;
    extrusionPlan.phase2_speed_m1 = m1.phase2_speed;
    extrusionPlan.phase2_speed_m2 = m2.phase2_speed;
  } else {
    extrusionPlan.use_two_phase = false;
    extrusionPlan.phase2_speed_m1 = speed1_mms;
    extrusionPlan.phase2_speed_m2 = speed2_mms;
  }
  
  // Calculate target positions (extrusion moves DOWN toward ZERO_ML_POSITION)
  long steps1 = (long)(extrusionPlan.vol1_to_dispense * STEPS_PER_ML);
  long steps2 = (long)(extrusionPlan.vol2_to_dispense * STEPS_PER_ML);
  
  long target1 = arduino_pos1 - steps1;
  long target2 = arduino_pos2 - steps2;

  extrusionPlan.phase2_target1 = target1;
  extrusionPlan.phase2_target2 = target2;
  
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
  
  extrusionPlan.ok = true;
  return extrusionPlan;
}
#endif // EXTRUSION_HELPERS_H
