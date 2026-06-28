# Plan: Rumble Gamepad 2 When Beam Breaker Reaches 3 Artifacts

## Goal
Trigger a short rumble on gamepad 2 in `MainTeleOp` the moment the beam breaker ball count first reaches 3 artifacts. Use edge-triggered behavior so it only rumbles once per "fill cycle" (not continuously while count >= 3).

## File to Modify
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/TeleOp/MainTeleOp.java`

No changes are needed to `Subsystem/BeamBreaker.java` — the existing `getBallCount()` already returns the current artifact count and `MainTeleOp.loop()` already updates `ballCount` from it.

## Implementation

### 1. Add edge-tracking field
In the "Rumble state tracking" block (around line 76–79, next to `wasShooterAtTarget`, `wasPathBusy`, `warnedEndGame`), add:

```java
protected boolean wasBeamAtThree = false;
```

### 2. Add rumble logic in `handleRumbleFeedback()`
In `MainTeleOp.handleRumbleFeedback()` (around line 622–643), append an edge-triggered rumble matching the existing pattern used for `wasShooterAtTarget`:

```java
// Driver 2: Rumble when beam breaker first reaches 3 artifacts
if (ballCount >= 3 && !wasBeamAtThree) {
    gamepad2.rumble(250);
}
wasBeamAtThree = ballCount >= 3;
```

### 3. Reset on manual reset points
The `prespinTriggered` flag is already reset in the existing code paths where `beamBreaker.resetBallCount()` is called (right trigger release at line 460 and A button at line 469). To keep edge behavior correct across those resets, also reset `wasBeamAtThree = false` at those same two spots so the next time the count climbs to 3 the rumble fires again.

## Duration choice
Using `250 ms` ("short"). Existing rumbles in this method use 100/300/500 ms, so 250 sits comfortably in the "short" range. Trivial to change if a different value is preferred.

## Verification
- Build the project (Gradle / Android Studio) to confirm no syntax errors.
- Manual test on robot: run intake, hold gamepad2 left trigger; confirm gamepad2 rumbles once as the third artifact passes the beam breaker, and not again until ball count drops below 3 (e.g., after shooting or pressing A to reset).
