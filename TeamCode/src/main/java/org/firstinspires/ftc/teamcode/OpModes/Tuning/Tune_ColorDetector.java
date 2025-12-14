package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.LifecycleManagementUtilities.HardwareInitializer;
import org.firstinspires.ftc.teamcode.LifecycleManagementUtilities.HardwareShutdown;
import org.firstinspires.ftc.teamcode.Subsystems.ColorDetector;
import org.firstinspires.ftc.teamcode.Utilities.ActionScheduler;

/**
 * Tuning OpMode for the ColorDetector Subsystem
 * <p>
 * Purpose: Calibrate hue and saturation thresholds for accurate ball color detection
 * <p>
 * FTC Dashboard Constants:
 * - GREEN_HUE_MIN: [0, 360] (default: 100)
 * - GREEN_HUE_MAX: [0, 360] (default: 140)
 * - PURPLE_HUE_MIN: [0, 360] (default: 250)
 * - PURPLE_HUE_MAX: [0, 360] (default: 290)
 * - MIN_SATURATION: [0.0, 1.0] (default: 0.4)
 * <p>
 * Controls:
 * - DPAD UP/DOWN: Adjust GREEN_HUE_MIN (±1)
 * - DPAD RIGHT/LEFT: Adjust GREEN_HUE_MAX (±1)
 * - LB/RB: Adjust PURPLE_HUE_MIN (±1)
 * - LT/RT: Adjust PURPLE_HUE_MAX (±1)
 * - A/Y: Adjust MIN_SATURATION (±0.05)
 * - X: Reset to defaults
 * <p>
 * Expected Behavior:
 * - Real-time RGB and HSV value display
 * - Color detection status feedback
 * - Threshold range validation
 * - Visual confirmation of color changes
 * <p>
 * Notes:
 * - Place test balls in front of sensors for calibration
 * - Values update in real-time through FTC Dashboard
 * - No timeout - operator can tune as long as needed
 * - Focus on accurate color differentiation
 */
@Disabled
@TeleOp(name = "Tune_ColorDetector", group = "Tuning")
public class Tune_ColorDetector extends OpMode {

	// Store default values for reset functionality
	private static final double DEFAULT_GREEN_HUE_MIN = 100;
	private static final double DEFAULT_GREEN_HUE_MAX = 140;
	private static final double DEFAULT_PURPLE_HUE_MIN = 250;
	private static final double DEFAULT_PURPLE_HUE_MAX = 290;
	private static final double DEFAULT_MIN_SATURATION = 0.4;
	private static final double HUE_STEP = 1.0;
	private static final double SATURATION_STEP = 0.05;
	private ColorDetector colorDetector;
	private ActionScheduler scheduler;

	@Override
	public void init() {
		// Initialize hardware
		HardwareInitializer.initialize(hardwareMap);
		colorDetector = ColorDetector.getInstance();
		scheduler = ActionScheduler.getInstance();

		telemetry.addData("Status", "Initialized - Waiting for START");
		telemetry.addData("Purpose", "Calibrate color detection thresholds");
		telemetry.addData("Note", "Place test balls in front of sensors");
		telemetry.update();
	}

	@Override
	public void start() {
		telemetry.addData("Status", "Running");
		telemetry.addData("Instructions", "Use gamepad to adjust thresholds");
		telemetry.update();
	}

	@Override
	public void loop() {
		// Update color sensor readings
		scheduler.schedule(colorDetector.update());

		// === GREEN HUE RANGE TUNING ===

		// DPAD UP - Increase GREEN_HUE_MIN
		if (gamepad1.dpad_up) {
			ColorDetector.GREEN_HUE_MIN = Math.min(ColorDetector.GREEN_HUE_MIN + HUE_STEP, 360);
			telemetry.addData("GREEN_HUE_MIN", "Adjusted to " + ColorDetector.GREEN_HUE_MIN);
		}

		// DPAD DOWN - Decrease GREEN_HUE_MIN
		if (gamepad1.dpad_down) {
			ColorDetector.GREEN_HUE_MIN = Math.max(ColorDetector.GREEN_HUE_MIN - HUE_STEP, 0);
			telemetry.addData("GREEN_HUE_MIN", "Adjusted to " + ColorDetector.GREEN_HUE_MIN);
		}

		// DPAD RIGHT - Increase GREEN_HUE_MAX
		if (gamepad1.dpad_right) {
			ColorDetector.GREEN_HUE_MAX = Math.min(ColorDetector.GREEN_HUE_MAX + HUE_STEP, 360);
			telemetry.addData("GREEN_HUE_MAX", "Adjusted to " + ColorDetector.GREEN_HUE_MAX);
		}

		// DPAD LEFT - Decrease GREEN_HUE_MAX
		if (gamepad1.dpad_left) {
			ColorDetector.GREEN_HUE_MAX = Math.max(ColorDetector.GREEN_HUE_MAX - HUE_STEP, 0);
			telemetry.addData("GREEN_HUE_MAX", "Adjusted to " + ColorDetector.GREEN_HUE_MAX);
		}

		// === PURPLE HUE RANGE TUNING ===

		// Left Bumper - Decrease PURPLE_HUE_MIN
		if (gamepad1.left_bumper) {
			ColorDetector.PURPLE_HUE_MIN = Math.max(ColorDetector.PURPLE_HUE_MIN - HUE_STEP, 0);
			telemetry.addData("PURPLE_HUE_MIN", "Adjusted to " + ColorDetector.PURPLE_HUE_MIN);
		}

		// Right Bumper - Increase PURPLE_HUE_MIN
		if (gamepad1.right_bumper) {
			ColorDetector.PURPLE_HUE_MIN = Math.min(ColorDetector.PURPLE_HUE_MIN + HUE_STEP, 360);
			telemetry.addData("PURPLE_HUE_MIN", "Adjusted to " + ColorDetector.PURPLE_HUE_MIN);
		}

		// Left Trigger - Decrease PURPLE_HUE_MAX
		if (gamepad1.left_trigger > 0.5) {
			ColorDetector.PURPLE_HUE_MAX = Math.max(ColorDetector.PURPLE_HUE_MAX - HUE_STEP, 0);
			telemetry.addData("PURPLE_HUE_MAX", "Adjusted to " + ColorDetector.PURPLE_HUE_MAX);
		}

		// Right Trigger - Increase PURPLE_HUE_MAX
		if (gamepad1.right_trigger > 0.5) {
			ColorDetector.PURPLE_HUE_MAX = Math.min(ColorDetector.PURPLE_HUE_MAX + HUE_STEP, 360);
			telemetry.addData("PURPLE_HUE_MAX", "Adjusted to " + ColorDetector.PURPLE_HUE_MAX);
		}

		// === SATURATION TUNING ===

		// Y button - Increase MIN_SATURATION
		if (gamepad1.y) {
			ColorDetector.MIN_SATURATION = Math.min(ColorDetector.MIN_SATURATION + SATURATION_STEP, 1.0);
			telemetry.addData("MIN_SATURATION", "Adjusted to " + String.format("%.2f", ColorDetector.MIN_SATURATION));
		}

		// A button - Decrease MIN_SATURATION
		if (gamepad1.a) {
			ColorDetector.MIN_SATURATION = Math.max(ColorDetector.MIN_SATURATION - SATURATION_STEP, 0.0);
			telemetry.addData("MIN_SATURATION", "Adjusted to " + String.format("%.2f", ColorDetector.MIN_SATURATION));
		}

		// === RESET TO DEFAULTS ===

		// X button - Reset all thresholds
		if (gamepad1.x) {
			ColorDetector.GREEN_HUE_MIN = DEFAULT_GREEN_HUE_MIN;
			ColorDetector.GREEN_HUE_MAX = DEFAULT_GREEN_HUE_MAX;
			ColorDetector.PURPLE_HUE_MIN = DEFAULT_PURPLE_HUE_MIN;
			ColorDetector.PURPLE_HUE_MAX = DEFAULT_PURPLE_HUE_MAX;
			ColorDetector.MIN_SATURATION = DEFAULT_MIN_SATURATION;
			telemetry.addData("Action", "Reset to defaults");
		}

		// === UPDATE SCHEDULER ===
		scheduler.update();

		// === DISPLAY TELEMETRY ===

		// Left sensor readings
		telemetry.addData("", "=== LEFT SENSOR ===");
		telemetry.addData("Red", colorDetector.leftRed);
		telemetry.addData("Green", colorDetector.leftGreen);
		telemetry.addData("Blue", colorDetector.leftBlue);
		telemetry.addData("Hue (0-360)", String.format("%.1f", colorDetector.leftHSV[0]));
		telemetry.addData("Saturation (0-1)", String.format("%.3f", colorDetector.leftHSV[1]));
		telemetry.addData("Value (0-1)", String.format("%.3f", colorDetector.leftHSV[2]));
		telemetry.addData("Is Green", colorDetector.leftIsGreen);
		telemetry.addData("Is Purple", colorDetector.leftIsPurple);

		// Right sensor readings
		telemetry.addData("", "=== RIGHT SENSOR ===");
		telemetry.addData("Red", colorDetector.rightRed);
		telemetry.addData("Green", colorDetector.rightGreen);
		telemetry.addData("Blue", colorDetector.rightBlue);
		telemetry.addData("Hue (0-360)", String.format("%.1f", colorDetector.rightHSV[0]));
		telemetry.addData("Saturation (0-1)", String.format("%.3f", colorDetector.rightHSV[1]));
		telemetry.addData("Value (0-1)", String.format("%.3f", colorDetector.rightHSV[2]));
		telemetry.addData("Is Green", colorDetector.rightIsGreen);
		telemetry.addData("Is Purple", colorDetector.rightIsPurple);

		// Current thresholds
		telemetry.addData("", "=== GREEN THRESHOLDS ===");
		telemetry.addData("GREEN_HUE_MIN", String.format("%d (DPAD ↑↓)", (int) ColorDetector.GREEN_HUE_MIN));
		telemetry.addData("GREEN_HUE_MAX", String.format("%d (DPAD ←→)", (int) ColorDetector.GREEN_HUE_MAX));

		telemetry.addData("", "=== PURPLE THRESHOLDS ===");
		telemetry.addData("PURPLE_HUE_MIN", String.format("%d (LB/RB)", (int) ColorDetector.PURPLE_HUE_MIN));
		telemetry.addData("PURPLE_HUE_MAX", String.format("%d (LT/RT)", (int) ColorDetector.PURPLE_HUE_MAX));

		telemetry.addData("", "=== SATURATION ===");
		telemetry.addData("MIN_SATURATION", String.format("%.2f (A/Y)", ColorDetector.MIN_SATURATION));

		// Controls reminder
		telemetry.addData("", "=== CONTROLS ===");
		telemetry.addData("X: Reset to defaults", "");

		// Range validation
		String greenRangeStatus = (ColorDetector.GREEN_HUE_MIN < ColorDetector.GREEN_HUE_MAX) ? "✓ VALID" : "✗ INVALID";
		String purpleRangeStatus = (ColorDetector.PURPLE_HUE_MIN < ColorDetector.PURPLE_HUE_MAX) ? "✓ VALID" : "✗ INVALID";
		telemetry.addData("Range Check", "Green: " + greenRangeStatus + ", Purple: " + purpleRangeStatus);

		telemetry.update();
	}

	@Override
	public void stop() {
		// Shutdown
		HardwareShutdown.shutdown();
	}
}
