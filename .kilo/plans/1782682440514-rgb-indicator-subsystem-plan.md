# RGB Indicator Subsystem Plan

## Goal
Extract the RGB indicator to its own subsystem and extend behavior:
- Reflect beam-breaker ball count: `purple idle → yellow → azure → flashing blue/violet`.
- Reflect shooter RPM when right trigger is held: green when within tolerance, red otherwise.
- When shooter is triggered but the robot is outside the valid shooting zones, flash the shooter-status color and white rapidly.

## Affected Files
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Subsystem/RGBIndicator.java` (new)
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/TeleOp/MainTeleOp.java` (wire up subsystem, drop inline logic)

No autonomous or test opmodes reference the `rgbIndicator` servo; no other files need edits.

## Constants (in `RGBIndicator`)
```
SERVO_IDLE_PURPLE = 0.722
SERVO_YELLOW      = 0.388
SERVO_AZURE       = 0.555
SERVO_GREEN       = 0.500
SERVO_RED         = 0.277
SERVO_BLUE        = 0.611
SERVO_VIOLET      = 0.722
SERVO_WHITE       = 1.0

SLOW_FLASH_PERIOD_MS  = 1000   // 0.5 Hz toggle, used for the 3-artifact flash
RAPID_FLASH_PERIOD_MS = 250    // 4 Hz toggle, used when outside shooting zones

SHOOTER_RPM_TOLERANCE = 150    // mirrors Transfer.SHOOTER_RPM_TOLERANCE
```

## API
```java
public class RGBIndicator extends SubsystemBase {
    public RGBIndicator(HardwareMap hardwareMap);
    public void setShooter(Shooter shooter);
    public void setBeamBreaker(BeamBreaker beamBreaker);
    public void setFollower(Follower follower);
    public void setShooterTriggered(boolean triggered);
    @Override public void periodic(); // selects color & writes to servo
}
```

## Decision Tree (evaluated each loop)
Priority is top-down — the first matching rule wins.

1. `shooterTriggered` is true:
   a. Compute `inZone = isInTriangle(pose, A) || isInTriangle(pose, B)`.
   b. If `!inZone`: write `RAPID_FLASH` between `(GREEN if atTargetRPM else RED)` and `WHITE`.
   c. If `inZone`: solid `GREEN` when `Shooter.isAtTargetRPM()` else solid `RED`.
2. Otherwise (shooter not triggered): use `beamBreaker.getBallCount()`.
   - `0` → solid `IDLE_PURPLE`
   - `1` → solid `YELLOW`
   - `2` → solid `AZURE`
   - `>= 3` → `SLOW_FLASH` alternating `BLUE ↔ VIOLET`

## Geometry (point-in-triangle)
Standard sign-based test using cross products:
```
sign(p1, p2, p3) = (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y)
isInTriangle(p, a, b, c) =
    hasSameSign(sign(p, a, b), sign(p, b, c), sign(p, c, a))
```
Triangles (Pedro field coordinates, inches, origin bottom-left):
- Zone A: `(0, 144)`, `(72, 72)`, `(144, 144)`
- Zone B: `(48, 0)`, `(72, 24)`, `(96, 0)`

## `MainTeleOp` Changes
1. Remove the `Servo rgbServo` field and its `hardwareMap.get(Servo.class, "rgbIndicator")` lookup in `init()`.
2. Remove the private `updateRGBIndicator()` method.
3. Add `protected RGBIndicator rgbIndicator;`.
4. In `init()` after the existing subsystems are constructed:
   ```java
   rgbIndicator = new RGBIndicator(hardwareMap);
   rgbIndicator.setShooter(shooter);
   rgbIndicator.setBeamBreaker(beamBreaker);
   rgbIndicator.setFollower(follower);
   ```
5. In `loop()`, replace the `updateRGBIndicator()` call with:
   ```java
   boolean shooterTriggered = gamepad2.right_trigger > 0.5;
   rgbIndicator.setShooterTriggered(shooterTriggered);
   rgbIndicator.periodic(); // safe even though SubsystemBase already runs it via scheduler
   ```
   (Alternatively, rely on the scheduler — the explicit call keeps the wiring obvious.)

## Out of Scope
- No changes to `Shooter`, `BeamBreaker`, `Transfer`, or any other subsystem.
- No new opmode registrations; `RGBIndicator` is only used by `MainTeleOp` subclasses.
- Panels telemetry for the RGB state is optional and not added here.

## Validation
Build the project (Gradle / Android Studio sync) — must compile clean.

Then either on the field or with a Pedro sim:
1. Idle (no balls, trigger released) → solid purple.
2. Intake one ball → solid yellow.
3. Intake a second → solid azure.
4. Intake a third → slow blue/violet flash (~1 Hz).
5. Release intake, hold right trigger inside Zone A or Zone B → solid green once shooter reaches target, red before.
6. Move outside both zones while holding right trigger → rapid white/(green|red) flash (~4 Hz).
7. Release trigger → return to ball-count mode regardless of position.

## Risks
- The Pedro `Follower.getPose()` coordinates use field inches with origin at the bottom-left of the standard FTC field, which matches the triangle coordinates supplied. Confirm during validation by parking at a known point (e.g., `Pose(72, 72)`) and verifying the indicator behaviour.
- REV Blinkin LEDs map PWM positions to discrete internal frames; positions chosen (0.722, 0.388, 0.555, 0.500, 0.611, 0.28, 1.0) all fall within the documented Blinkin color range and should read as the intended colors.

## Open Questions
None blocking — defaults were chosen per your earlier answers.