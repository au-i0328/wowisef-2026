# Gravity Compensation Implementation

## Overview

Added adaptive PIDG (PID + Gravity compensation) that automatically switches coefficients based on **motor direction AND bar pose**. This correctly accounts for the fact that either gripper configuration can move up or down - the mechanical forces depend on direction of travel, not which gripper is attached.

## Configuration Structure

### PIDG Profiles

```cpp
struct PIDGConfig {
  float kp;                    // Proportional gain
  float ki;                    // Integral gain
  float kd;                    // Derivative gain
  float gravity_compensation;  // PWM offset to counteract weight
};
```

### Six Operating Configurations

The system selects from 6 different PIDG profiles based on:
1. **Motor direction** (FORWARD=up, BACKWARD=down, STOP=holding)
2. **Current bar pose** (parallel vs angled)

**Key insight:** Gripper state doesn't determine direction. With upper detached OR lower detached, the robot can move either up or down depending on motor command.

---

## 1. MOVING UP Configurations (FORWARD)

### PIDG_UP_PARALLEL
**Active when:** Motors moving FORWARD, **bars parallel**

```cpp
const PIDGConfig PIDG_UP_PARALLEL = {
  .kp = 0.8f,
  .ki = 0.05f,
  .kd = 0.2f,
  .gravity_compensation = 100.0f
};
```

**Use case:** Climbing upward with parallel bars, regardless of which gripper is attached. Full robot weight on motors, no mechanical advantage.

**Characteristics:**
- Highest P gain for strong response
- Maximum gravity compensation (100 PWM)
- Aggressive control for safety during climb

---

### PIDG_UP_ANGLED
**Active when:** Motors moving FORWARD, **bars in "up_detach" pose**

```cpp
const PIDGConfig PIDG_UP_ANGLED = {
  .kp = 0.7f,
  .ki = 0.04f,
  .kd = 0.18f,
  .gravity_compensation = 80.0f
};
```

**Use case:** Climbing upward while bars are angled to provide mechanical advantage. Bars push outward against structure, reducing effective weight on motors.

**Characteristics:**
- Lower gains than parallel (less force needed)
- Reduced gravity compensation (80 PWM vs 100)
- Smoother control due to bar leverage

---

## 2. MOVING DOWN Configurations (BACKWARD)

### PIDG_DOWN_PARALLEL
**Active when:** Motors moving BACKWARD, **bars parallel**

```cpp
const PIDGConfig PIDG_DOWN_PARALLEL = {
  .kp = 0.6f,
  .ki = 0.04f,
  .kd = 0.15f,
  .gravity_compensation = 40.0f
};
```

**Use case:** Descending with parallel bars, regardless of which gripper is attached. Gravity assists downward motion.

**Characteristics:**
- Moderate gains (don't want to overshoot)
- Lower gravity compensation (gravity helps)
- Smoother response to prevent drops

---

### PIDG_DOWN_ANGLED
**Active when:** Motors moving BACKWARD, **bars in "down_detach" pose**

```cpp
const PIDGConfig PIDG_DOWN_ANGLED = {
  .kp = 0.5f,
  .ki = 0.03f,
  .kd = 0.12f,
  .gravity_compensation = 30.0f
};
```

**Use case:** Descending with bars angled for maximum control. Bars provide leverage point for controlled lowering.

**Characteristics:**
- Lowest active gains (most mechanical assist)
- Minimal gravity compensation needed
- Gentlest control for safe descent

---

## 3. FALLBACK Configuration

### PIDG_BOTH
**Active when:** 
- Both grippers attached (robot locked)
- Both grippers open (error state)
- Motors STOPPED (holding position)

```cpp
const PIDGConfig PIDG_BOTH = {
  .kp = 0.5f,
  .ki = 0.03f,
  .kd = 0.1f,
  .gravity_compensation = 0.0f
};
```

**Use case:** 
- Both attached: Robot locked in place (shouldn't move)
- Both open: Free-fall or error state
- Stopped: Holding position between moves

**Characteristics:**
- No gravity compensation (no active movement)
- Minimal gains
- Used during transitions and errors

---

## Automatic Profile Selection

The system tracks three pieces of state:

```cpp
enum Direction { STOP, FORWARD, BACKWARD };
Direction current_direction = STOP;  // Updated by motor commands

enum GripperState { GRIPPER_OPEN, GRIPPER_CLOSED };
GripperState upperGripperState = GRIPPER_CLOSED;
GripperState lowerGripperState = GRIPPER_CLOSED;

String current_bar_pose = "parallel";
```

Selection logic:

```cpp
const PIDGConfig& selectPIDGConfig() {
  // Both attached or both detached = no compensation
  if ((upperGripperState == GRIPPER_CLOSED && lowerGripperState == GRIPPER_CLOSED) ||
      (upperGripperState == GRIPPER_OPEN && lowerGripperState == GRIPPER_OPEN)) {
    return PIDG_BOTH;
  }
  
  // One gripper attached - check DIRECTION and bar pose
  bool movingUp = (current_direction == FORWARD);
  bool movingDown = (current_direction == BACKWARD);
  
  if (movingUp) {
    // CLIMBING UP
    if (current_bar_pose == "up_detach") {
      return PIDG_UP_ANGLED;      // Bars provide mechanical advantage
    } else {
      return PIDG_UP_PARALLEL;    // Standard climb
    }
  }
  else if (movingDown) {
    // DESCENDING DOWN
    if (current_bar_pose == "down_detach") {
      return PIDG_DOWN_ANGLED;    // Bars assist descent
    } else {
      return PIDG_DOWN_PARALLEL;  // Standard descent
    }
  }
  else {
    // STOPPED - holding position
    return PIDG_BOTH;
  }
}
```

**Critical difference from previous version:** 
- Old: Assumed upper detached = always climbing, lower detached = always descending
- New: Uses motor `current_direction` to determine if moving up or down
- Reality: Either gripper configuration can move in either direction

---

## State Tracking

### Motor Direction Updates
Automatically updated when motor commands execute:

```cpp
void setMotorSpeed(int speed, Direction dir) {
  current_speed = speed;
  current_direction = dir;  // STOP, FORWARD, or BACKWARD
  // ... motor driver commands ...
}
```

### Gripper State Updates
Automatically updated when commands execute:

```cpp
void up_detach() {
  // ... servo operations ...
  upperGripperState = GRIPPER_OPEN;
}

void down_attach() {
  // ... servo operations ...
  lowerGripperState = GRIPPER_CLOSED;
}
```

### Bar Pose Updates
Updated by `setBarPosition()` function:

```cpp
void setBarPosition(String pose) {
  current_bar_pose = pose;  // "parallel", "up_detach", "down_detach"
  // ... servo angle calculations ...
}
```

---

## Example Scenarios

### Scenario 1: Climbing with upper gripper detached
```
State: upperGripperState = OPEN, lowerGripperState = CLOSED
Command: set_alt(500mm) - motors go FORWARD
Bar pose: "up_detach"
Result: Uses PIDG_UP_ANGLED
```

### Scenario 2: Descending with upper gripper detached
```
State: upperGripperState = OPEN, lowerGripperState = CLOSED
Command: set_alt(200mm) - motors go BACKWARD
Bar pose: "parallel"
Result: Uses PIDG_DOWN_PARALLEL
```

### Scenario 3: Climbing with lower gripper detached
```
State: upperGripperState = CLOSED, lowerGripperState = OPEN
Command: set_alt(600mm) - motors go FORWARD
Bar pose: "parallel"
Result: Uses PIDG_UP_PARALLEL
```

### Scenario 4: Descending with lower gripper detached
```
State: upperGripperState = CLOSED, lowerGripperState = OPEN
Command: set_alt(100mm) - motors go BACKWARD
Bar pose: "down_detach"
Result: Uses PIDG_DOWN_ANGLED
```

---

## Status Output Enhancement

JSON status now includes active PIDG configuration:

```json
{
  "speed": 120,
  "dir": "FORWARD",
  "mode": "hold",
  "pose": "up_detach",
  "gripper": {
    "upper": "open",
    "lower": "closed"
  },
  "pidg": {
    "kp": 0.70,
    "ki": 0.040,
    "kd": 0.18,
    "g": 80.0
  },
  "target_alt": 500.0,
  "true_height": 498.3
}
```

This allows real-time verification that:
- Correct profile is selected based on direction + bar pose
- Coefficients match expected values
- System responds to direction changes

---

## Configuration Variables

All tunable parameters at top of file:

```cpp
const float ALTITUDE_DEADBAND_MM = 5.0f;   // ±5mm tolerance
const int BASE_CLIMB_SPEED = 150;          // Manual climb PWM
const int MIN_HOLD_SPEED = 80;             // Minimum motor threshold
```

---

## Tuning Guidelines

### Initial Calibration Steps

**1. Find Gravity Compensation Values (4 measurements needed):**

```
Test sequence A: UP_PARALLEL
  1. Detach one gripper (either one)
  2. zero_alt
  3. Manual mode, motors FORWARD
  4. Keep bars parallel
  5. Gradually increase PWM
  6. Find PWM where robot holds position without drift
  7. Record as PIDG_UP_PARALLEL.gravity_compensation

Test sequence B: UP_ANGLED
  1. Detach one gripper
  2. zero_alt
  3. Set bars to up_detach pose (angled)
  4. Manual mode, motors FORWARD
  5. Find holding PWM (should be lower than parallel)
  6. Record as PIDG_UP_ANGLED.gravity_compensation

Test sequence C: DOWN_PARALLEL
  1. Detach one gripper
  2. Climb to 500mm
  3. Keep bars parallel
  4. Manual mode, motors BACKWARD
  5. Find holding PWM
  6. Record as PIDG_DOWN_PARALLEL.gravity_compensation

Test sequence D: DOWN_ANGLED
  1. Detach one gripper
  2. Climb to 500mm
  3. Set bars to down_detach pose (angled)
  4. Manual mode, motors BACKWARD
  5. Find holding PWM (should be lowest)
  6. Record as PIDG_DOWN_ANGLED.gravity_compensation
```

**Expected values:**
- UP_PARALLEL: ~100 PWM (highest - fighting full weight)
- UP_ANGLED: ~80 PWM (bar assist reduces effective weight)
- DOWN_PARALLEL: ~40 PWM (gravity helps pull down)
- DOWN_ANGLED: ~30 PWM (lowest - bars + gravity both help)

**2. Tune P Gain for Each Configuration:**

Start with UP_PARALLEL (most critical for safety):
```
1. Set KI=0, KD=0, KP=0.5
2. Command hold_alt at 300mm
3. Increase KP until oscillation
4. Reduce by 30-40%
5. Repeat for other 3 active configurations
```

**3. Add I Gain:**
```
1. Start with KI = 0.02
2. Increase until steady-state error disappears
3. Watch for overshoot (reduce if needed)
```

**4. Add D Gain:**
```
1. Start with KD = 0.1
2. Increase until oscillations dampen
3. Too much D = sluggish response
```

---

## Expected Behavior Differences

### Why UP_ANGLED uses lower gains:

When bars are angled during upward climb:
- Bars push against structure
- Creates outward force component
- Reduces effective vertical weight on motors
- Less motor power needed
- Lower gains prevent overshoot

### Why DOWN_ANGLED needs minimal compensation:

When bars are angled during descent:
- Bars create leverage point
- Weight distribution favors controlled drop
- Gravity already pulls down
- Minimal motor force needed to control rate
- Risk: too much gain causes uncontrolled drop

---

## Testing Sequence

### 1. Test Each Configuration Independently

**Test UP_PARALLEL:**
```
up_detach → zero_alt → keep bars parallel → set_alt(500mm)
Expected: Smooth climb, no oscillation, reaches target ±5mm
Status shows: dir="FORWARD", pidg.g=100.0
```

**Test UP_ANGLED:**
```
up_detach → zero_alt → bars auto-angle → set_alt(500mm)
Expected: Easier climb than parallel, less motor strain
Status shows: dir="FORWARD", pidg.g=80.0
```

**Test DOWN_PARALLEL:**
```
(at 500mm, parallel bars) → set_alt(200mm)
Expected: Controlled descent, no drop, smooth deceleration
Status shows: dir="BACKWARD", pidg.g=40.0
```

**Test DOWN_ANGLED:**
```
(at 500mm) → down_detach (bars angle) → set_alt(200mm)
Expected: Slowest descent, highest control, minimal overshoot
Status shows: dir="BACKWARD", pidg.g=30.0
```

### 2. Test Bidirectional Movement with Same Gripper Config

```
1. up_detach (upper open, lower closed)
2. zero_alt
3. set_alt(400mm) - climb with PIDG_UP_*
4. Wait for arrival
5. set_alt(200mm) - descend with PIDG_DOWN_*
6. Verify status shows correct PIDG profile for each direction
```

### 3. Test Direction Reversal

```
1. Start climbing: set_alt(500mm)
2. Mid-climb, change target: set_alt(100mm)
3. Should switch from PIDG_UP_* to PIDG_DOWN_* when direction reverses
4. Monitor status pidg values during transition
```

### 4. Test with Different Gripper Configurations

Both upper detached and lower detached should work identically:
```
Test A: up_detach → climb to 500mm → descend to 200mm
Test B: down_detach → climb to 500mm → descend to 200mm
Both should use same PIDG profiles based on direction
```

---

## Safety Considerations

- Gravity compensation **only active** when exactly one gripper detached
- Profile selection based on motor direction, not gripper choice
- Bar pose changes automatically update PIDG profile
- Each configuration independently tuned for its mechanical state
- Anti-windup prevents integral overshoot across all profiles
- MIN_HOLD_SPEED prevents motor stall in all modes
- Status output shows active config for debugging
- Direction changes trigger immediate profile switch

---

## Mechanical Advantage Summary

| Configuration | Direction | Bar Pose | Mechanical Advantage | G Compensation |
|--------------|-----------|----------|---------------------|----------------|
| UP_PARALLEL | FORWARD | Parallel | None | Highest (100) |
| UP_ANGLED | FORWARD | up_detach | Bars reduce weight | High (80) |
| DOWN_PARALLEL | BACKWARD | Parallel | Gravity assists | Low (40) |
| DOWN_ANGLED | BACKWARD | down_detach | Bars + gravity | Lowest (30) |
| BOTH | STOP or both grippers | Any | N/A | None (0) |

**Note:** Either gripper configuration (upper open OR lower open) can use any of these profiles depending on direction.

---

## Common Issues & Solutions

**Issue:** Robot climbs fine but oscillates when descending

**Solution:** DOWN_* gains too high. Reduce KP by 20%, KD by 10%.

---

**Issue:** Robot drops quickly when changing from climb to descent

**Solution:** Profile switching too abrupt. Consider ramping gravity compensation when direction reverses, or check that DOWN_* compensation isn't too low.

---

**Issue:** Status shows wrong PIDG profile during movement

**Solution:** Check that `current_direction` updates correctly in motor control functions. Verify with status JSON output.

---

**Issue:** Same gripper config behaves differently going up vs down

**Solution:** This is correct! Direction determines which PIDG profile is used. Verify compensation values are appropriate for each direction.

---

**Issue:** Robot doesn't hold position when stopped between moves

**Solution:** PIDG_BOTH may need small gravity compensation (~20 PWM) for holding, or implement separate "hold" mode that maintains last active PIDG profile.

---

## Future Enhancements

1. **Smooth transitions:** Gradually interpolate between PIDG profiles when direction reverses
2. **Auto-calibration:** Automatically measure gravity compensation by detecting motor stall in each direction
3. **Adaptive tuning:** Adjust gains based on observed performance over time
4. **Battery compensation:** Reduce gravity compensation as battery voltage drops
5. **Hold-position mode:** Remember last active profile when STOPPED instead of defaulting to PIDG_BOTH
