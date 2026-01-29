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
import org.firstinspires.ftc.teamcode.Utils.SpindexerPosition;

@Configurable
public class Spindexer extends SubsystemBase {
	public final DcMotor spindexerMotor;
	PIDFController spindexerPIDF;
	private int offset = 0;
	private double prevTarget;
	public static double P = 0.1;
	public static double I = 0;
	public static double D = 0;
	public static double F = 0;
	private double PREV_P = 0;
	private double PREV_I = 0;
	private double PREV_D = 0;
	private double PREV_F = 0;
	private double power = 0;
	private boolean isAtTarget = true;
	double targetTicks = 0;
	double targetDegrees = 0;

	public Spindexer(HardwareMap hardwareMap) {
		spindexerMotor = hardwareMap.get(DcMotor.class, "spindexerMotor");
		spindexerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		spindexerPIDF = new PIDFController(P, I, D, F);
		spindexerPIDF.setOutputLimits(1);

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

	public Command DirectPower(double power) {
		return new InstantCommand(
				() -> spindexerMotor.setPower(power), this
		);
	}

	public Command NextTarget() {
		return new CommandBase() {
			@Override
			public void initialize() {
				double nextTarget = SpindexerPosition.getNextShootPosition((int) prevTarget);
				prevTarget = nextTarget;

				targetDegrees = nextTarget - 20; // We subtract 20 as our algorithm overshoots (by design)
				targetTicks = targetDegrees * 1.4936;

				isAtTarget = false;

				spindexerPIDF.setSetpoint(targetTicks);
				spindexerPIDF.setSetpointRange(60);
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
				return Math.abs(getPosition() - targetTicks) < 40;
			}
		};
	}

	public void Telemetry(Telemetry telemetry) {
		double currentPosTicks = getPosition();

		telemetry.addData("Spindexer Position (degrees)", currentPosTicks * 0.6695);
		telemetry.addData("Spindexer Target (degrees)", targetDegrees);
		telemetry.addData("Spindexer Power", power);
		telemetry.addData("Tick Error", targetTicks - currentPosTicks);
	}

	public void Telemetry(TelemetryManager telemetry) {
		double currentPosTicks = getPosition();

		telemetry.addData("Spindexer Position (degrees)", currentPosTicks * 0.6695);
		telemetry.addData("Spindexer Target (degrees)", targetDegrees);
		telemetry.addData("Spindexer Power", power);
		telemetry.addData("Tick Error", targetTicks - currentPosTicks);
	}

	public boolean isAtTarget() {
		return isAtTarget;
	}
}
