package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.BallColor;
import org.firstinspires.ftc.teamcode.Utilities.PIDFController;

@Config
public class Spindexer {
	// 360 ticks per 360 degrees for the through-bore encoder
	public static double TICKS_PER_REV = 360.0;

	public static double zeroOffset = 73.6;

	// PID coefficients for position control. Tuned 2025-11-22.
	public static double P = 0.01, I = 0.0, D = 0.0, F = 0.0;
	// EMA tuning parameters
	public static double ALPHA_SMOOTH = 0.05;
	// Hybrid encoder calibration constants
	public static double TOLERANCE_DEGREES = 3.0;
	public static double MANUAL_TRIM_STEP_DEGREES = 2.0;
	private static Spindexer instance = null;
	// Store detected ball colors for each slot (0, 1, 2)
	private final BallColor[] ballColors = new BallColor[3];
	public PIDFController controller;
	public double targetPosition = 0;
	public double power = 0;
	public double currentPositionDegrees = 0;
	public double quadratureOffest = 0;
	private CRServo spindexerLeft;
	private CRServo spindexerRight;
	private AnalogInput spindexerEncoder;
	private DcMotorEx quadratureEncoder;
	// EMA filter state variables
	private double filteredValue = 0.0;
	private boolean isInitialized = false;
	// Hybrid encoder calibration state
	private double calibrationSum = 0.0;
	private int calibrationCount = 0;
	private long calibrationStartTime = 0;
	private boolean calibrationActive = false;

	private Spindexer() {
		// Initialize all slots as EMPTY
		for (int i = 0; i < 3; i++) {
			ballColors[i] = BallColor.EMPTY;
		}
	}

	public static void initialize(HardwareMap hardwareMap) {
		instance = new Spindexer();
		instance.spindexerLeft = hardwareMap.get(CRServo.class, "spindexerLeft");
		instance.spindexerRight = hardwareMap.get(CRServo.class, "spindexerRight");
		instance.spindexerEncoder = hardwareMap.get(AnalogInput.class, "spindexerEncoder");
		instance.quadratureEncoder = hardwareMap.get(DcMotorEx.class, "frontRight");
		instance.controller = new PIDFController(P, I, D, F);
		instance.controller.setOutputLimits(-1, 1);
	}

	public static Spindexer getInstance() {
		if (instance == null) {
			throw new IllegalStateException("Spindexer not initialized. Call initialize(hardwareMap, elapsedTime) first.");
		}
		return instance;
	}

	public static void shutdown() {
		// No cleanup needed currently
	}

	/**
	 * Resets the calibration averaging process.
	 */
	public void resetCalibrationAverage() {
		instance.quadratureOffest = instance.quadratureEncoder.getCurrentPosition();
		calibrationSum = 0.0;
		calibrationCount = 0;
		calibrationStartTime = System.nanoTime();
		calibrationActive = true;
		RobotState state = RobotState.getInstance();
		state.absoluteOffset = 0;
		state.averageQuality = 0;
		state.hasValidData = false;
	}

	/**
	 * Updates the incremental average with the current absolute encoder reading.
	 */
	public void updateCalibrationAverage() {
		if (!calibrationActive) return;
		double absPos = getPosition(); // degrees
		calibrationSum += absPos;
		calibrationCount++;
	}

	/**
	 * Finalizes teleop calibration with validation against existing data.
	 */
	public void finalizeTeleOpCalibration() {
		if (!calibrationActive || calibrationCount == 0) return;
		double avgAbs = calibrationSum / calibrationCount;
		double currentQuad = getQuadraturePosition();
		double teleOpOffset = avgAbs - currentQuad;
		RobotState state = RobotState.getInstance();
		if (state.hasValidData) {
			double diff = Math.abs(teleOpOffset - state.absoluteOffset);
			if (diff <= TOLERANCE_DEGREES) {
				// Weighted average (50-50)
				state.absoluteOffset = (teleOpOffset + state.absoluteOffset) / 2.0;
			} else {
				// Fallback: overwrite
				state.absoluteOffset = teleOpOffset;
			}
		} else {
			state.absoluteOffset = teleOpOffset;
		}
		state.averageQuality = (System.nanoTime() - calibrationStartTime) / 1e9;
		state.hasValidData = true;
		calibrationActive = false;
	}

	/**
	 * Returns the calibrated position using quadrature + offset.
	 */
	public double getCalibratedPosition() {
		return -(getQuadraturePosition() + RobotState.getInstance().absoluteOffset);
	}

	/**
	 * Gets the quadrature encoder position in degrees.
	 */
	public double getQuadraturePosition() {
		return (((quadratureEncoder.getCurrentPosition() - quadratureOffest) / 8192.0) * 360.0) + zeroOffset;
	}

	/**
	 * Gets the filtered position using Dynamic Exponential Moving Average (EMA).
	 * This provides a smoothed position value that adapts to motion.
	 */
	public double getPosition() {
		double rawInput = ((spindexerEncoder.getVoltage() / 3.3) * 360.0);
		if (!isInitialized) {
			filteredValue = rawInput;
			isInitialized = true;
			return filteredValue;
		}
		filteredValue = (rawInput * ALPHA_SMOOTH) + (filteredValue * (1 - ALPHA_SMOOTH));
		return filteredValue;
	}

	public void update() {
		currentPositionDegrees = getCalibratedPosition();

		// PID values should never be updated during operation (per requirements)
		// controller.setPID(P, I, D, F); // REMOVED as per requirements

		power = controller.getOutput(currentPositionDegrees, targetPosition);
		instance.spindexerRight.setDirection(DcMotorSimple.Direction.REVERSE);
		spindexerLeft.setPower(power);
		spindexerRight.setPower(power);

		//currentPosition = currentPositionDegrees;
	}

	public Action setTarget(double angle) {
		return new InstantAction(() -> {
			targetPosition = angle;
		});
	}

	public Action toPosition(double angle) {
		return packet -> {
			targetPosition = angle;

			// This action is considered "done" when the error is small.
			// This allows it to be a "blocking" call in a sequence.
			// Returns true while moving (error >= threshold), false when within tolerance
			double error = targetPosition - getCalibratedPosition();
			packet.put("Spindexer Error", error);
			return Math.abs(error) >= 17; // Returns true while moving, false when within tolerance
		};
	}

	public BallColor getBallColor(int slotIndex) {
		if (slotIndex >= 0 && slotIndex < 3) {
			return ballColors[slotIndex];
		}
		return BallColor.UNKNOWN;
	}

	public void setBallColor(int slotIndex, BallColor color) {
		if (slotIndex >= 0 && slotIndex < 3) {
			ballColors[slotIndex] = color;
		}
	}

	public void setTargetPosition(double revolutions) {
		targetPosition = revolutions; //* TICKS_PER_REV;
	}

	/**
	 * Force set the spindexer power to a value between -1 and 1.
	 * Clamps the power to the valid range.
	 */
	public InstantAction setDirectPower(double power) {
		return new InstantAction(() -> {
			// Clamp power between -1 and 1
			double clampedPower = Math.max(-1.0, Math.min(1.0, power));
			spindexerLeft.setPower(clampedPower);
			spindexerRight.setPower(clampedPower);
		});
	}
}
