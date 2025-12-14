package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Utilities.PIDFController;
import org.firstinspires.ftc.teamcode.Utilities.FeedForwardController;

@Config
public class Shooter {
	// --- PID Controller Constants ---
	public static double UPPER_P = 0.0024, UPPER_I = 0, UPPER_D = 0;
	public static double LOWER_P = 0.002, LOWER_I = 0, LOWER_D = 0;

	// --- Feedforward Constants ---
	public static double UPPER_KS = 0.28, UPPER_KV = 0.000095;
	public static double LOWER_KS = 0.29, LOWER_KV = 0.00017;

	// --- Motor Power Constants ---
	public static double STOP_POWER = 0.0;

	// --- RPM & Control Constants ---
	public static double RPM_TOLERANCE = 100.0;
	public static double TICKS_PER_REVOLUTION = 28.0;

	// --- Pre-calculated constants ---
	private static final double RPM_CONVERSION = 60.0 / TICKS_PER_REVOLUTION;
	private static final double HALF_DIVISOR = 0.5;
	public static double AUDIENCE_RPM = 2570.0;

	// --- Motor Offsets ---
	public static double UPPER_OFFSET = 0.0;
	public static double LOWER_OFFSET = 0.0;

	private static Shooter instance = null;

	private DcMotorEx upperShooter;
	private DcMotorEx lowerShooter;

	// Public State
	public double averageRPM = 0.0;
	public double upperRPM = 0.0;
	public double lowerRPM = 0.0;

	// Acceleration tracking (kept if you need it for tuning, otherwise unused)
	public double upperAcceleration = 0.0;
	public double lowerAcceleration = 0.0;
	public double averageAcceleration = 0.0;

	private long lastNanoTime = 0;
	private double lastUpperVelocity = 0.0;
	private double lastLowerVelocity = 0.0;

	private PIDFController upperController;
	private PIDFController lowerController;
	private FeedForwardController upperFF;
	private FeedForwardController lowerFF;

	private Shooter() {}

	public static void initialize(HardwareMap hardwareMap) {
		// Correct: Always create new instance to avoid stale hardware references
		instance = new Shooter();
		instance.upperShooter = hardwareMap.get(DcMotorEx.class, "upperShooter");
		instance.lowerShooter = hardwareMap.get(DcMotorEx.class, "lowerShooter");

		instance.upperShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		instance.lowerShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

		instance.upperController = new PIDFController(UPPER_P, UPPER_I, UPPER_D, 0);
		instance.lowerController = new PIDFController(LOWER_P, LOWER_I, LOWER_D, 0);

		instance.upperFF = new FeedForwardController(UPPER_KS, UPPER_KV, 0);
		instance.lowerFF = new FeedForwardController(LOWER_KS, LOWER_KV, 0);
	}

	public static Shooter getInstance() {
		if (instance == null) {
			throw new IllegalStateException("Shooter not initialized. Call initialize(hardwareMap) first.");
		}
		return instance;
	}

	public static void shutdown() {
		if(instance != null) {
			instance.upperShooter.setPower(STOP_POWER);
			instance.lowerShooter.setPower(STOP_POWER);
		}
	}

	public void updateRPM(long nanoTime) {
		double upperVelocity = upperShooter.getVelocity();
		double lowerVelocity = lowerShooter.getVelocity();

		upperRPM = upperVelocity * RPM_CONVERSION;
		lowerRPM = lowerVelocity * RPM_CONVERSION;
		averageRPM = (upperRPM + lowerRPM) * HALF_DIVISOR;

		if (lastNanoTime != 0) {
			double timeElapsedSeconds = (nanoTime - lastNanoTime) / 1_000_000_000.0;
			if (timeElapsedSeconds > 0) {
				upperAcceleration = (upperVelocity - lastUpperVelocity) / timeElapsedSeconds;
				lowerAcceleration = (lowerVelocity - lastLowerVelocity) / timeElapsedSeconds;
				averageAcceleration = (upperAcceleration + lowerAcceleration) * HALF_DIVISOR;
			}
		}
		lastNanoTime = nanoTime;
		lastUpperVelocity = upperVelocity;
		lastLowerVelocity = lowerVelocity;
	}

	/**
	 * Checks if BOTH motors are at their specific targets.
	 * Returns FALSE if targets are < 100 (Safety: You aren't "Ready to Shoot" if stopped).
	 */
	public boolean isAtTargetRPM(double upperTarget, double lowerTarget) {
		// Check Upper: Must be a valid target (>100) AND within tolerance
		boolean upperReady = (upperTarget >= 100) && (Math.abs(upperRPM - upperTarget) <= RPM_TOLERANCE);

		// Check Lower: Must be a valid target (>100) AND within tolerance
		boolean lowerReady = (lowerTarget >= 100) && (Math.abs(lowerRPM - lowerTarget) <= RPM_TOLERANCE);

		return upperReady && lowerReady;
	}

	/**
	 * Convenience method for single target (both motors same speed)
	 */
	public boolean isAtTargetRPM(double targetRPM) {
		return isAtTargetRPM(targetRPM, targetRPM);
	}

	/**
	 * CENTRAL PHYSICS METHOD
	 * Calculates powers, applies them, and handles common telemetry.
	 */
	private void updateMotors(double upperTarget, double lowerTarget, TelemetryPacket packet) {
		double upperPower, lowerPower;

		// UPPER MOTOR
		if (upperTarget < 100) {
			upperPower = STOP_POWER;
		} else {
			// Only calculate PID/FF when we actually want to move
			double pid = upperController.getOutput(upperRPM, upperTarget);
			double ff = upperFF.calculate(upperTarget, 0);
			upperPower = ff + pid + UPPER_OFFSET;
		}

		// LOWER MOTOR
		if (lowerTarget < 100) {
			lowerPower = STOP_POWER;
		} else {
			double pid = lowerController.getOutput(lowerRPM, lowerTarget);
			double ff = lowerFF.calculate(lowerTarget, 0);
			lowerPower = ff + pid + LOWER_OFFSET;
		}

		upperShooter.setPower(upperPower);
		lowerShooter.setPower(lowerPower);

		// Standardized Telemetry
		packet.put("Shooter Upper Target", upperTarget);
		packet.put("Shooter Lower Target", lowerTarget);
		packet.put("Shooter Upper RPM", upperRPM);
		packet.put("Shooter Lower RPM", lowerRPM);
		packet.put("Upper Power", upperPower);
		packet.put("Lower Power", lowerPower);
	}

	public Action run(double targetRPM) {
		return run(targetRPM, targetRPM);
	}

	public Action run(double upperTargetRPM, double lowerTargetRPM) {
		return packet -> {
			updateRPM(System.nanoTime());
			updateMotors(upperTargetRPM, lowerTargetRPM, packet);
			return false;
		};
	}

	public Action runAndWait(double upperTargetRPM, double lowerTargetRPM) {
		return new Action() {
			private boolean hasReachedTarget = false;
			private long startTime = -1;
			private final long TIMEOUT_NS = 5_000_000_000L;

			@Override
			public boolean run(@NonNull TelemetryPacket packet) {
				long now = System.nanoTime();
				if (startTime == -1) startTime = now;

				// 1. TIMEOUT CHECK
				if (now - startTime > TIMEOUT_NS) {
					// Safety: Stop if we time out
					upperShooter.setPower(0);
					lowerShooter.setPower(0);
					return false;
				}

				updateRPM(now);

				// 2. CONTROL LOGIC (Reused from helper)
				updateMotors(upperTargetRPM, lowerTargetRPM, packet);

				// 3. CHECK IF AT TARGET
				if (!hasReachedTarget && isAtTargetRPM(upperTargetRPM, lowerTargetRPM)) {
					hasReachedTarget = true;
				}

				packet.put("Shooter Ready", hasReachedTarget);

				// 4. SHOT DETECTION
				if (!hasReachedTarget) {
					return true; // Keep spinning up
				}

				// Drop detection
				double targetAvg = (upperTargetRPM + lowerTargetRPM) * 0.5;
				double currentAvg = (upperRPM + lowerRPM) * 0.5;
				boolean shotFired = currentAvg < (targetAvg * 0.92);

				return !shotFired;
			}
		};
	}

	public Action stop() {
		return new InstantAction(() -> {
			upperShooter.setPower(STOP_POWER);
			lowerShooter.setPower(STOP_POWER);
		});
	}
}
