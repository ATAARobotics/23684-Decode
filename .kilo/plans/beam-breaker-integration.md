# Beam Breaker Integration Plan

## Overview
Integrate the beam breaker sensor to count artifacts (balls) entering/exiting the robot, with robust edge detection, intake direction awareness, and driver override capabilities.

---

## 1. Fix BeamBreaker.java (`Subsystem/BeamBreaker.java`)

### Bug Fix
- Change `intakeBeamBreaker.setMode(DigitalChannel.Mode.OUTPUT)` → `DigitalChannel.Mode.INPUT`

### New State Variables
- `private int ballCount = 0` — current count of balls in robot
- `private boolean lastBeamState = false` — previous beam state for edge detection
- `private boolean lastIntakeIn = true` — tracks if intake was running IN during last edge
- `private long lastEdgeTime = 0` — timestamp for debouncing (ms)
- `private static final long DEBOUNCE_MS = 50` — minimum time between valid edges

### New Methods
- `public int getBallCount()` — returns current count
- `public void setBallCount(int count)` — sets count (for override)
- `public void resetBallCount()` — sets count to 0
- `public void update(boolean intakeRunningIn)` — call each loop; performs edge detection with direction awareness
  - Detects **broken → unbroken** transition (object entered): increment count if intake was running IN
  - Detects **unbroken → broken** transition (object left): decrement count if intake was running OUT
   - Enforces **physically impossible transitions**:
     - If beam state flips twice within DEBOUNCE_MS, ignore (debounce)
     - If beam breaks → unbroken → breaks within < 300ms, treat as noise (ball passing through, count only once)
   - Ball count **can go negative** when intake runs backwards (OUT) and expels more balls than recorded
   - Logs every edge event with timestamp and direction

- `public void telemetry(TelemetryManager.TelemetryWrapper telemetry)` — outputs beam state, ball count, last event, last direction to Panels

---

## 2. Modify MainTeleOp.java (`OpModes/TeleOp/MainTeleOp.java`)

### New Fields
- `protected BeamBreaker beamBreaker` — subsystem reference
- `protected int ballCount = 0` — synced from BeamBreaker
- `protected boolean lastIntakeIn = true` — intake direction for current loop
- `protected boolean prespinTriggered = false` — one-shot flag for prespin at 3 balls
- `protected boolean g2AButtonPressed = false` — for override button (already declared on line 58, repurpose)

### init() Changes
- Initialize `beamBreaker = new BeamBreaker(hardwareMap)`

### loop() Changes — Beam Breaker Update (before displayTelemetry)
1. **Determine intake direction**:
   - `boolean intakeRunningIn = (intake.intake.getPower() > 0.1)` — check if intake motor is running forward
   - Update `lastIntakeIn`

2. **Call beam breaker update**:
   - `beamBreaker.update(intakeRunningIn)`
   - Sync: `ballCount = beamBreaker.getBallCount()`

3. **Prespin trigger at 3 balls**:
   - `if (ballCount >= 3 && !prespinTriggered)` → `scheduler.schedule(newShooter.setTarget(NewShooter.AUDIENCE_TPR, NewShooter.AUDIENCE_TPR))`, set flag true
   - Reset flag when ballCount drops below 3

4. **Ball count reset on shooter release**:
   - When right trigger is released (shooter stops) AND `ballCount > 0` → `beamBreaker.resetBallCount()`, `prespinTriggered = false`
   - This handles "after shooter trigger released, ball count goes to 0"

5. **Override button (gamepad2 A)**:
   - `if (gamepad2.a && !g2AButtonPressed)` → `beamBreaker.resetBallCount()`, `ballCount = 0`, `prespinTriggered = false`
   - Set `g2AButtonPressed = true` on press, reset on release

### displayTelemetry() Changes
- Add Beam Breaker section:
  ```
  === BEAM BREAKER ===
  Beam Broken: true/false
  Ball Count: N
  Last Event: "BROKEN→CLEAR" / "CLEAR→BROKEN" / "None"
  Last Direction: "IN" / "OUT"
  Debounce Active: true/false
  Intake Running: IN/OUT/STOPPED
  ```

---

## 3. Documentation Requirements
- All new methods in BeamBreaker.java get JavaDocs
- Inline comments explaining edge detection logic, debounce rationale, and direction awareness
- MainTeleOp.java comments explaining ball count lifecycle and override behavior

---

## Implementation Order
1. Rewrite BeamBreaker.java (fix + new logic + telemetry)
2. Update MainTeleOp.java (integration + override + prespin trigger + telemetry)
3. Verify no compilation errors
