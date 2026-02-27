package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.Utils.RobotPosition;
import org.firstinspires.ftc.teamcode.Utils.SpindexerPosition;

@Configurable
public class Spindexer extends SubsystemBase {
	public static double P = 0.009;
	public static double I = 0;
	public static double D = 0.006;
	public static double F = 0;
	public static double SLOT_TOLERANCE = 15;

	// 537.7 PPR motor with 2:1 gear ratio (reduction)
	// TICKS_PER_REV = 537.7 * 2 = 1075.4
	// TICKS_PER_DEGREE = 1075.4 / 360 = 2.987222222222222
	// DEGREES_PER_TICK = 360 / 1075.4 = 0.3347591593825553
	public static final double TICKS_PER_DEGREE = 2.987222222222222;
	public static final double DEGREES_PER_TICK = 0.3347591593825553;

	public final DcMotor spindexerMotor;
PIDFController spindexerPIDF;
	double targetTicks = 0;
	double targetDegrees = 0;
	private int offset = 0;
	private double prevTarget;
	private double PREV_P = 0;
	private double PREV_I = 0;
	private double PREV_D = 0;
	private double PREV_F = 0;
	private double power = 0;
	private boolean isAtTarget = true;
	private boolean overrided = false;

	public Spindexer(HardwareMap hardwareMap) {
		spindexerMotor = hardwareMap.get(DcMotor.class, "spindexerMotor");
		spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		spindexerPIDF = new PIDFController(P, I, D, F);

		if (RobotPosition.isSpindexerSet) {
			offset = spindexerMotor.getCurrentPosition() - RobotPosition.spindexerTicks;
			// RobotPosition.isSpindexerSet = false;
		}

		prevTarget = 0;
	}

	@Override
	public void periodic() {
		updatePIDFCoefficients();
	}

	public void updatePIDFCoefficients() {
		if (P != PREV_P || I != PREV_I || D != PREV_D || F != PREV_F) {
			spindexerPIDF = new PIDFController(P, I, D, F);
			spindexerPIDF.setOutputLimits(1);
			spindexerPIDF.setSetpoint(targetTicks);
			spindexerPIDF.setSetpointRange(30);
			PREV_P = P;
			PREV_I = I;
			PREV_D = D;
			PREV_F = F;
		}
	}

	public void zeroSpindexer() {
		offset = spindexerMotor.getCurrentPosition();
		prevTarget = 0;
	}

	public double getPosition() {
		return spindexerMotor.getCurrentPosition() - offset;
	}

	public double getDegrees() {
		return getPosition() * DEGREES_PER_TICK;
	}

	public int getCurrentSlot() {
		double degrees = getDegrees();
		double normalizedDegrees = ((degrees % 360) + 360) % 360;

		// Slot 1 is at 0 degrees, Slot 2 at 120, Slot 3 at 240
		if (Math.abs(normalizedDegrees - 0) <= SLOT_TOLERANCE || Math.abs(normalizedDegrees - 360) <= SLOT_TOLERANCE) return 1;
		if (Math.abs(normalizedDegrees - 120) <= SLOT_TOLERANCE) return 2;
		if (Math.abs(normalizedDegrees - 240) <= SLOT_TOLERANCE) return 3;

		return -1; // Not at a valid slot
	}

	public Command DirectPower(double power) {
		return new InstantCommand(
				() -> {
					spindexerMotor.setPower(power);
					overrided = true;
					isAtTarget = false;
				}, this
		);
	}

	public Command NextTarget() {
		return new CommandBase() {
			@Override
			public void initialize() {
				double currentDegrees = getDegrees();
				// Fast-forward prevTarget if we are already past the next derived target
				while (SpindexerPosition.getNextIntakePosition((int) prevTarget) <= currentDegrees + 10) {
					prevTarget = SpindexerPosition.getNextIntakePosition((int) prevTarget);
				}

				double nextTarget = SpindexerPosition.getNextIntakePosition((int) prevTarget);
				prevTarget = nextTarget;

				targetDegrees = nextTarget - 20; // We subtract 20 as our algorithm overshoots (by design)
				targetTicks = targetDegrees * TICKS_PER_DEGREE;

				isAtTarget = false;
				overrided = false;

				spindexerPIDF.setSetpoint(targetTicks);
				//spindexerPIDF.setSetpointRange(60);
			}

			@Override
			public void execute() {
				power = spindexerPIDF.getOutput(getPosition(), targetTicks);
				spindexerMotor.setPower(power);
			}

			@Override
			public void end(boolean interrupted) {
				spindexerMotor.setPower(0);
				isAtTarget = true;
			}

			@Override
			public boolean isFinished() {
				return Math.abs(power) < 0.05 || overrided;
			}
		};
	}

	public void Telemetry(Telemetry telemetry) {
		double currentPosTicks = getPosition();

		telemetry.addData("Spindexer Position (degrees)", currentPosTicks * DEGREES_PER_TICK);
		telemetry.addData("Spindexer Target (degrees)", targetDegrees);
		telemetry.addData("Spindexer Power", power);
		telemetry.addData("Tick Error", targetTicks - currentPosTicks);
		telemetry.addData("Current Slot", getCurrentSlot());
	}

	public void Telemetry(TelemetryManager telemetry) {
		double currentPosTicks = getPosition();

		telemetry.addData("Spindexer Position (degrees)", currentPosTicks * DEGREES_PER_TICK);
		telemetry.addData("Spindexer Target (degrees)", targetDegrees);
		telemetry.addData("Spindexer Power", power);
		telemetry.addData("Tick Error", targetTicks - currentPosTicks);
		telemetry.addData("Current Slot", getCurrentSlot());
	}

	public boolean isAtTarget() {
		return isAtTarget;
	}

	public void savePosition() {
		RobotPosition.spindexerTicks = (int) getPosition();
		RobotPosition.isSpindexerSet = true;
	}
}
