# Plan: Open Gate at 80% Shooter Target

## Current Behavior
In `MainTeleOp.java`, when Right Trigger is pressed (lines 301-318):
- `openGate = true` is set immediately via `InstantCommand`
- Gate opens before shooter reaches target RPM
- This is line 305: `new InstantCommand(()-> openGate = true)`

## Required Change
Gate should only open when the shooter RPM is at or above 80% of the target RPM.

## Implementation Steps

### Step 1: Add percentage method to Shooter.java
Add a new method `getPercentToTarget()` in `Shooter.java` that returns a decimal value (0.0 to 1.0+) representing the shooter's progress toward target RPM. This provides flexibility for future threshold adjustments.

### Step 2: Modify MainTeleOp.java gate opening logic
Change the right trigger handler to:
- Start shooter and set target
- Wait for gate to open until `shooter.getPercentToTarget() >= 0.8`
- Use a `WaitUntilCommand` to wait for the 80% condition before opening gate

## Files to Modify
1. `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystem/Shooter.java` - Add method to check 80% threshold
2. `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/TeleOp/MainTeleOp.java` - Modify sequence to wait for 80% before opening gate

## Technical Details

### Shooter.getPercentToTarget()
Returns a decimal value representing progress toward target RPM:
- Calculate `averageRPM / averageTarget` 
- Return value clamped to [0.0, 1.0+] (can exceed 1.0 if overshooting)
- Handle edge case where target is 0 to avoid division by zero

### Modified sequence in MainTeleOp
Replace the immediate `openGate = true` with a wait condition:
```java
scheduler.schedule(
    new SequentialCommandGroup(
        shooter.SetTarget(Shooter.AUDIENCE_RPM, Shooter.AUDIENCE_RPM),
        new WaitUntilCommand(() -> shooter.getPercentToTarget() >= 0.8),
        new InstantCommand(() -> openGate = true),
        shooter.WaitForTarget().withTimeout(2500),
        transfer.TransferOut(),
        spindexer.DirectPower(1)
    ));
```