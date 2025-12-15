package org.firstinspires.ftc.teamcode.OpModes.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LifecycleManagementUtilities.HardwareInitializer;
import org.firstinspires.ftc.teamcode.LifecycleManagementUtilities.HardwareShutdown;
import org.firstinspires.ftc.teamcode.LifecycleManagementUtilities.SubsystemUpdater;
import org.firstinspires.ftc.teamcode.Subsystems.RobotState;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Utilities.ActionScheduler;
import org.firstinspires.ftc.teamcode.Utilities.SpindexerPosition;

/**
 * Tuning OpMode for the Spindexer Subsystem
 * <p>
 * Purpose: Optimize PID coefficients for smooth position control using real-time adjustment
 * <p>
 * FTC Dashboard Constants:
 * - P: [0.0, 0.1] (default: 0.005)
 * - I: [0.0, 0.01] (default: 0.0)
 * - D: [0.0, 0.001] (default: 0.0)
 * - F: [0.0, 0.1] (default: 0.0)
 * - zeroOffset: [-100, 100] (default: 0)
 * <p>
 * Controls:
 * - DPAD UP/DOWN: Adjust P (±0.0005)
 * - LB/RB: Adjust I (±0.0001)
 * - LT/RT: Adjust D (±0.00001)
 * - B: Move to position 0.25 (90°)
 * - X: Move to position 0.5 (180°)
 * - Y: Move to position 0.75 (270°)
 * - A: Move to next shoot position using utility
 * - Right Stick: Fine-tune with bigger increments
 * <p>
 * Expected Behavior:
 * - Real-time PID coefficient adjustment
 * - Position control validation
 * - Smooth motion with proper damping
 * - Accurate zero point calibration
 * <p>
 * Notes:
 * - MUST zero spindexer before position commands
 * - Values update in real-time through FTC Dashboard
 * - Use right stick for larger adjustment increments
 * - Monitor error values for tuning feedback
 */
@TeleOp(name = "Tune_Spindexer", group = "Tuning")
public class Tune_Spindexer extends OpMode {
	private static final double FINE_MULTIPLIER = 5.0; // Right stick increases step size
	private Spindexer spindexer;
	private ActionScheduler scheduler;
	// Button state tracking for edge detection
	private boolean prevA = false;
	private boolean prevB = false;
	private boolean prevX = false;
	private boolean prevY = false;
	private double targetPosition = 0;
	private Telemetry dashboardTelemetry;

	@Override
	public void init() {
		// Initialize hardware
		HardwareInitializer.initialize(hardwareMap);
		spindexer = Spindexer.getInstance();
		scheduler = ActionScheduler.getInstance();

		telemetry.addData("Status", "Initialized - Waiting for START");
		telemetry.addData("Purpose", "Tune spindexer PID coefficients");
		telemetry.addData("Note", "Use FTC Dashboard to adjust values in real-time");
		telemetry.update();

		dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
	}

	@Override
	public void start() {
		telemetry.addData("Status", "Running");
		telemetry.addData("Instructions", "B/X/Y=Move");
		telemetry.update();
	}

	@Override
	public void loop() {
		// Calculate step multipliers
		double stepMultiplier = gamepad1.right_stick_button ? FINE_MULTIPLIER : 1.0;

		// === POSITION COMMANDS ===
		// Only schedule on button press (not held)
		if (gamepad1.b && !prevB) {
			scheduler.schedule(spindexer.toPosition(0.25)); // 90 degrees
			targetPosition = 90;
			telemetry.addData("Action", "Moving to 90°");
		}
		prevB = gamepad1.b;

		if (gamepad1.x && !prevX) {
			scheduler.schedule(spindexer.toPosition(0.5)); // 180 degrees
			targetPosition = 180;
			telemetry.addData("Action", "Moving to 180°");
		}
		prevX = gamepad1.x;

		if (gamepad1.y && !prevY) {
			scheduler.schedule(spindexer.toPosition(0.75)); // 270 degrees
			targetPosition = 270;
			telemetry.addData("Action", "Moving to 270°");
		}
		prevY = gamepad1.y;

		// Button A: Use utility class to calculate next position
		if (gamepad1.a && !prevA) {
			// Get current position in degrees (already a double)
			double currentDegrees = spindexer.getCalibratedPosition();

			// Calculate next shoot position using utility (now handles doubles)
			double nextShootPosition = SpindexerPosition.getNextShootPosition((int) currentDegrees);

			// Convert to revolutions for the spindexer (0-1 range)
			double targetRevolutions = nextShootPosition / 360.0;
			scheduler.schedule(spindexer.toPosition(targetRevolutions));
			targetPosition = nextShootPosition;
			telemetry.addData("Action", "Moving to next shoot position: " + String.format("%.1f", nextShootPosition) + "°");
		}
		prevA = gamepad1.a;

		// === UPDATE SCHEDULER AND PID CONTROLLER ===
		SubsystemUpdater.update();
		scheduler.update();
		spindexer.controller.setPID(Spindexer.P, Spindexer.I, Spindexer.D, Spindexer.F);
		spindexer.update();

		// === TELEMETRY DISPLAY ===

		// Position information
		double currentTicks = spindexer.getCalibratedPosition();
		double currentRevolutions = currentTicks / Spindexer.TICKS_PER_REV;
		double currentDegrees = currentRevolutions * 360.0;
		int currentSlot = (int) ((currentDegrees / 120) % 3);

		telemetry.addData("", "=== POSITION CONTROL ===");
		telemetry.addData("Current Position", currentDegrees);
		telemetry.addData("Current Slot", currentSlot + " (0-2)");
		telemetry.addData("Target Position", targetPosition);

		dashboardTelemetry.addData("Current Position", currentDegrees);
		dashboardTelemetry.addData("Target Position", targetPosition);
		dashboardTelemetry.addData("power",spindexer.power);
		dashboardTelemetry.update();

		// PID coefficients display
		telemetry.addData("", "=== PID COEFFICIENTS ===");
		telemetry.addData("P", String.format("%.4f (DPAD ↑↓)", Spindexer.P));
		telemetry.addData("I", String.format("%.4f (LB/RB)", Spindexer.I));
		telemetry.addData("D", String.format("%.4f (LT/RT)", Spindexer.D));
		telemetry.addData("F", String.format("%.4f", Spindexer.F));

		// RobotState display
		RobotState state = RobotState.getInstance();
		telemetry.addData("", "=== ROBOT STATE ===");
		telemetry.addData("Absolute Offset", String.format("%.2f°", state.absoluteOffset));
		telemetry.addData("Average Quality", String.format("%.2f sec", state.averageQuality));
		telemetry.addData("Has Valid Data", state.hasValidData);

		// Control information
		telemetry.addData("", "=== CONTROLS ===");
		telemetry.addData("A: NEXT SHOOT    B: 90°    X: 180°    Y: 270°", "");
		telemetry.addData("Fine tune: " + (gamepad1.right_stick_button ? "ENABLED" : "disabled"), "Right stick button");
		if (stepMultiplier > 1.0) {
			telemetry.addData("Step Multiplier", String.format("%.1fx", stepMultiplier));
		}

		// Performance feedback
		String stabilityStatus;
		if (Math.abs(currentTicks % (Spindexer.TICKS_PER_REV / 3)) < 50) {
			stabilityStatus = "✓ STABLE";
		} else {
			stabilityStatus = "Moving...";
		}
		telemetry.addData("Status", stabilityStatus);

		telemetry.update();
	}

	@Override
	public void stop() {
		// Shutdown
		HardwareShutdown.shutdown();
	}
}
