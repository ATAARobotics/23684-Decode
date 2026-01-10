package org.firstinspires.ftc.teamcode.Subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Utills.FeedForwardController;
import org.firstinspires.ftc.teamcode.Utills.PIDFController;

@Config
@Configurable
public class Shooter extends SubsystemBase {
	private static final double HALF_DIVISOR = 0.5;
	// --- PID Controller Constants ---
	public static double UPPER_P = 0.0024, UPPER_I = 0, UPPER_D = 0;
	public static double LOWER_P = 0.001, LOWER_I = 0, LOWER_D = 0;
	// --- Feedforward Constants ---
	public static double UPPER_KS = 0.15, UPPER_KV = 0.000074;
	public static double LOWER_KS = 0.3, LOWER_KV = 0.00017;
	// --- Motor Power Constants ---
	public static double STOP_POWER = 0.0;
	// --- RPM & Control Constants ---
	public static double RPM_TOLERANCE = 50; // 100 before
	public static double TICKS_PER_REVOLUTION = 28.0;
	// --- Pre-calculated constants ---
	private static final double RPM_CONVERSION = 60.0 / TICKS_PER_REVOLUTION;
	public static double AUDIENCE_RPM = 2310;
	public static double GOAL_RPM = 2310;

	// --- Motor Offsets ---
	public static double UPPER_OFFSET = 0.0;
	public static double LOWER_OFFSET = 0.0;

	private final DcMotorEx upperShooter;
	private final DcMotorEx lowerShooter;

	// Public State
	public double averageRPM = 0.0;
	public double upperRPM = 0.0;
	public double lowerRPM = 0.0;

	// Current Targets
	public double upperTarget = 0.0;
	public double lowerTarget = 0.0;

	private final PIDFController upperController;
	private final PIDFController lowerController;
	private final FeedForwardController upperFF;
	private final FeedForwardController lowerFF;

	public Shooter(HardwareMap hardwareMap) {
		upperShooter = hardwareMap.get(DcMotorEx.class, "upperShooter");
		lowerShooter = hardwareMap.get(DcMotorEx.class, "lowerShooter");

		upperShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		lowerShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		upperShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
		lowerShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

		upperController = new PIDFController(UPPER_P, UPPER_I, UPPER_D, 0);
		lowerController = new PIDFController(LOWER_P, LOWER_I, LOWER_D, 0);
		upperFF = new FeedForwardController(UPPER_KS, UPPER_KV, 0);
		lowerFF = new FeedForwardController(LOWER_KS, LOWER_KV, 0);
	}

	@Override
	public void periodic() {
		updateRPM();
		updateMotors();
	}

	private void updateRPM() {
		double upperVelocity = upperShooter.getVelocity();
		double lowerVelocity = lowerShooter.getVelocity();

		upperRPM = upperVelocity * RPM_CONVERSION;
		lowerRPM = lowerVelocity * RPM_CONVERSION;
		averageRPM = (upperRPM + lowerRPM) * HALF_DIVISOR;
	}

	public boolean isAtTargetRPM() {
		// Check Upper: Must be a valid target (>100) AND within tolerance
		boolean upperReady = (upperTarget >= 100) && (Math.abs(upperRPM - upperTarget) <= RPM_TOLERANCE);

		// Check Lower: Must be a valid target (>100) AND within tolerance
		boolean lowerReady = (lowerTarget >= 100) && (Math.abs(lowerRPM - lowerTarget) <= RPM_TOLERANCE);

		return upperReady && lowerReady;
	}

	private void updateMotors() {
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
	}

	public void setTarget(double upperTarget, double lowerTarget) {
		this.upperTarget = upperTarget;
		this.lowerTarget = lowerTarget;
	}

	public Command SetTarget(double upperTarget, double lowerTarget) {
		return new InstantCommand(() -> setTarget(upperTarget, lowerTarget), this);
	}

	public Command WaitForTarget() {
		return new WaitUntilCommand(this::isAtTargetRPM);
	}
}
