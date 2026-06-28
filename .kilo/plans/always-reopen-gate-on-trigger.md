# Plan: Always Re-Open Gate When Shooter Is Triggered

## Problem
In `MainTeleOp.java`, the gate currently relies on an `openGate` boolean flag that is only flipped to `true` once via `new InstantCommand(() -> openGate = true)` inside the `SequentialCommandGroup` scheduled on the **first** right-trigger press (`!rightTriggerPressed` gate, line 442). If the gate servo fails to reach position `0` on that first pass (stall, jammed linkage, missed command, etc.), the code never re-asserts the open command — the flag is already `true` but the physical servo is not actually open.

## Goal
While the shooter is being triggered (right trigger held), every loop iteration must:
1. Read the current physical position of the gate servo.
2. If it is **not** at the required open position (`0`), re-send the open command.

## Changes

### 1. `Subsystem/Gate.java`
Add a small accessor so `MainTeleOp` can read the live servo position:
- New method `public double getCurrentPosition()` that returns `gateServo.getPosition()`.
- (Optional, for clarity) Constant `public static final double OPEN_POSITION = 0;` and `CLOSE_POSITION = 1;` so the magic numbers stop being duplicated.

No changes to `openGate()` / `closeGate()` command signatures.

### 2. `OpModes/TeleOp/MainTeleOp.java`
Replace the flag-only block at lines 221–225 in `loop()`:

```java
if (openGate) {
    scheduler.schedule(gate.openGate());
} else {
    scheduler.schedule(gate.closeGate());
}
```

with a position-based check tied to the shooter trigger:

```java
boolean shooterTriggered = gamepad2.right_trigger > 0.5;
if (shooterTriggered) {
    if (gate.getCurrentPosition() != Gate.OPEN_POSITION) {
        scheduler.schedule(gate.openGate());
    }
} else {
    scheduler.schedule(gate.closeGate());
}
```

Notes:
- This runs **every loop** while the trigger is held (not just on the first trigger edge), satisfying "always check instead of on first trigger".
- `openGate` flag handling in the existing `SequentialCommandGroup` (line 447) can be left in place for telemetry/state tracking, or removed if the team prefers — it is no longer the source of truth for whether to send the open command.
- The close branch still uses the existing `closeGate()` schedule, so releasing the trigger still closes the gate.

### 3. (Optional, no functional impact)
Add `gate.getCurrentPosition()` to the `displayTelemetry()` block (around line 577) so the driver station shows the actual servo position for debugging.

## Files Touched
- `D:\Aarav\Coding\23684-Decode\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\Subsystem\Gate.java`
- `D:\Aarav\Coding\23684-Decode\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\OpModes\TeleOp\MainTeleOp.java`

## Out of Scope
- Auto OpModes that use `ShootArtifacts` (`Utils/ShootArtifacts.java`, `OpModes/Auto/Modular/ModularAuto.java`) — those already explicitly schedule `gate.openGate()` / `closeGate()` in their command groups and do not rely on the TeleOp flag-based logic.
- Changes to shooter RPM tolerance or transfer logic.

## Verification
- Build/deploy and test on the robot: hold the right trigger, observe that if the gate is bumped closed mid-shot it re-opens within one loop (~20 ms).
- Release the trigger and confirm the gate closes.
- Check telemetry line shows the gate position.