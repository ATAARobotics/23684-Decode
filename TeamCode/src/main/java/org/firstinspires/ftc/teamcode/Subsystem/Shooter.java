package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Utils.FeedForwardController;
import org.firstinspires.ftc.teamcode.Utils.PIDFController;

@Configurable
public class Shooter extends SubsystemBase {
	private static final double HALF_DIVISOR = 0.5;
	public static boolean TUNING_MODE = false;
	// --- PID Controller Constants ---
	public static double UPPER_P = 0.01, UPPER_I = 0, UPPER_D = 0;
	public static double LOWER_P = 0.017, LOWER_I = 0.00, LOWER_D = 0;
	// --- Feedforward Constants ---
	public static double UPPER_KS = 0.32, UPPER_KV = 0.00007;
	public static double LOWER_KS = 0.333, LOWER_KV = 0.00009;
	// --- Only for Tuning ---
	public static double PREV_UPPER_P = 0, PREV_UPPER_I = 0, PREV_UPPER_D = 0;
	public static double PREV_LOWER_P = 0, PREV_LOWER_I = 0, PREV_LOWER_D = 0;
	public static double PREV_UPPER_KS = 0, PREV_UPPER_KV = 0;
	public static double PREV_LOWER_KS = 0, PREV_LOWER_KV = 0;
	// --- Motor Power Constants ---
	public static double STOP_POWER = 0.0;
	// --- RPM & Control Constants ---
	public static double RPM_TOLERANCE = 35;
	public static double DROP_RPM_TOLERANCE = 150;
	public static double TICKS_PER_REVOLUTION = 28.0;

	// --- Pre-calculated constants ---
	private static final double RPM_CONVERSION = 60.0 / TICKS_PER_REVOLUTION;
	public static double AUDIENCE_RPM = 2070;
	public static double GOAL_RPM_UPPER = 1900;
	public static double GOAL_RPM_LOWER = 2250;

	private final DcMotorEx upperShooter;
	private final DcMotorEx lowerShooter;

	// Public State
	public double averageRPM = 0.0;
	public double upperRPM = 0.0;
	public double lowerRPM = 0.0;

	// Current Targets
	public double upperTarget = 0.0;
	public double lowerTarget = 0.0;

	private PIDFController upperController;
	private PIDFController lowerController;
	private FeedForwardController upperFF;
	private FeedForwardController lowerFF;

	public Shooter(HardwareMap hardwareMap) {
		upperShooter = hardwareMap.get(DcMotorEx.class, "upperShooter");
		lowerShooter = hardwareMap.get(DcMotorEx.class, "lowerShooter");

		upperShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		lowerShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

	public void updatePIDCoefficients() {
		if (TUNING_MODE) {
			if (UPPER_P != PREV_UPPER_P || UPPER_I != PREV_UPPER_I || UPPER_D != PREV_UPPER_D) {
				upperController = new PIDFController(UPPER_P, UPPER_I, UPPER_D);

				PREV_UPPER_P = UPPER_P;
				PREV_UPPER_I = UPPER_I;
				PREV_UPPER_D = UPPER_D;
			}

			if (LOWER_P != PREV_LOWER_P || LOWER_I != PREV_LOWER_I || LOWER_D != PREV_LOWER_D) {
				lowerController = new PIDFController(LOWER_P, LOWER_I, LOWER_D);

				PREV_LOWER_P = LOWER_P;
				PREV_LOWER_I = LOWER_I;
				PREV_LOWER_D = LOWER_D;
			}

			if (UPPER_KS != PREV_UPPER_KS || UPPER_KV != PREV_UPPER_KV) {
				upperFF = new FeedForwardController(UPPER_KS, UPPER_KV, 0);

				PREV_UPPER_KS = UPPER_KS;
				PREV_UPPER_KV = UPPER_KV;
			}

			if (LOWER_KS != PREV_LOWER_KS || LOWER_KV != PREV_LOWER_KV) {
				lowerFF = new FeedForwardController(LOWER_KS, LOWER_KV, 0);

				PREV_LOWER_KS = LOWER_KS;
				PREV_LOWER_KV = LOWER_KV;
			}
		}
	}

	public void setTuningMode(boolean TUNING_MODE) {
		Shooter.TUNING_MODE = TUNING_MODE;
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

	public boolean isRPMDropped() {
		// Check Upper: Must be a valid target (>100) AND within tolerance
		boolean upperReady = (upperTarget >= 100) && (Math.abs(upperRPM - upperTarget) <= DROP_RPM_TOLERANCE);

		// Check Lower: Must be a valid target (>100) AND within tolerance
		boolean lowerReady = (lowerTarget >= 100) && (Math.abs(lowerRPM - lowerTarget) <= DROP_RPM_TOLERANCE);

		return !(upperReady && lowerReady);
	}

	private void updateMotors() {
		double upperPower, lowerPower;

		// UPPER MOTOR
		if (Math.abs(upperTarget) < 50) {
			upperPower = STOP_POWER;
		} else {
			// Only calculate PID/FF when we actually want to move
			double pid = upperController.getOutput(upperRPM, upperTarget);
			double ff = upperFF.calculate(upperTarget, 0);
			upperPower = ff + pid;
		}

		// LOWER MOTOR
		if (lowerTarget < 100) {
			lowerPower = STOP_POWER;
		} else {
			double pid = lowerController.getOutput(lowerRPM, lowerTarget);
			double ff = lowerFF.calculate(lowerTarget, 0);
			lowerPower = ff + pid;
		}

		upperShooter.setPower(upperPower);
		lowerShooter.setPower(lowerPower);
	}

	public void setTarget(double upperTarget, double lowerTarget) {
		this.upperTarget = upperTarget;
		this.lowerTarget = lowerTarget;
	}

	public double TotalCurrentDrawn(){
		return upperShooter.getCurrent(CurrentUnit.AMPS) + lowerShooter.getCurrent(CurrentUnit.AMPS);
	}

	public Command SetTarget(double upperTarget, double lowerTarget) {
		return new InstantCommand(() -> setTarget(upperTarget, lowerTarget), this);
	}

	public Command WaitForTarget() {
		return new WaitUntilCommand(this::isAtTargetRPM);
	}

	public Command WaitForDrop() {
		return new WaitUntilCommand(this::isRPMDropped);
	}
}
