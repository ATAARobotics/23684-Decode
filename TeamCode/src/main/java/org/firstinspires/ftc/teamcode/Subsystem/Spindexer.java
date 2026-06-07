package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.Utils.RobotPosition;
import org.firstinspires.ftc.teamcode.Utils.SpindexerPosition;
import org.firstinspires.ftc.teamcode.Utils.SqurPIDFController;

@Configurable
public class Spindexer extends SubsystemBase {
	public static double P1 = 0.02;

	public static  double P2 = 0.04;
	public static double I = 0;
	public static double D = 0.006;
	public static double F = 0;

	public static double SLOT_TOLERANCE = 15;

	public double Nextslot = 1;

	// 537.7 PPR motor with 2:1 gear ratio (reduction)
	// TICKS_PER_REV = 537.7 * 2 = 1075.4
	// TICKS_PER_DEGREE = 1075.4 / 360 = 2.987222222222222
	// DEGREES_PER_TICK = 360 / 1075.4 = 0.3347591593825553

	// 8192 CPR through bore encoder
	// TICKS_PER_REV = 8192
	// TICKS_PER_DEGREE = 8192 / 360 = 22.75555555555556
	// DEGREES_PER_TICK = 360 / 8192 = 0.0439453125
	public static final double TICKS_PER_DEGREE = 22.75555555555556;
	public static final double DEGREES_PER_TICK = 0.0439453125;

	public final DcMotorEx spindexerMotor;

	AnalogInput spindexerAnalog;
PIDFController secondSpindexerPIDF;


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
	double target;
	boolean stop = false;

//	public double Pvalue(double distance){
//
//	}

	public Spindexer(HardwareMap hardwareMap) {
		spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
		spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		spindexerAnalog = hardwareMap.get(AnalogInput.class, "spindexerAnalog");

		secondSpindexerPIDF = new PIDFController(P1, I, D, F);



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
		if (P1 != PREV_P || I != PREV_I || D != PREV_D || F != PREV_F) {
			secondSpindexerPIDF = new PIDFController(P1, I, D, F);
			secondSpindexerPIDF.setOutputLimits(1);
			secondSpindexerPIDF.setSetpoint(targetTicks);
			secondSpindexerPIDF.setSetpointRange(30);
			PREV_P = P1;
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

	public double getDistanceFromSlot() {
		double degrees = getDegrees();
		double normalizedDegrees = ((degrees % 360) + 360) % 360;

		// Find distance to nearest slot (0, 120, 240 degrees)
		double distTo0 = Math.min(Math.abs(normalizedDegrees - 0), Math.abs(normalizedDegrees - 360));
		double distTo120 = Math.abs(normalizedDegrees - 120);
		double distTo240 = Math.abs(normalizedDegrees - 240);

		return Math.min(distTo0, Math.min(distTo120, distTo240));
	}

	public double getDistanceFromSlot(double slot) {
		double degrees = getDegrees();
		double normalizedDegrees = ((degrees % 360) + 360) % 360;

		// Find distance to nearest slot (0, 120, 240 degrees)
		double distTo0 = Math.min(Math.abs(normalizedDegrees - 0), Math.abs(normalizedDegrees - 360));
		double distTo120 = Math.abs(normalizedDegrees - 120);
		double distTo240 = Math.abs(normalizedDegrees - 240);

		if (slot == 0){
			return distTo0;
		}
		if (slot == 1){
			return distTo120;
		}
		if (slot == 2){
			return distTo240;
		}

		return Math.min(distTo0, Math.min(distTo120, distTo240));
	}


	public boolean isAlignedWithSlot() {
		return getCurrentSlot() != -1;
	}

	public Command DirectPower(double power) {
		return new InstantCommand(
				() -> {
					spindexerMotor.setPower(-power);
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

				secondSpindexerPIDF.setSetpoint(targetTicks);
				secondSpindexerPIDF.setSetpointRange(60);
			}

			@Override
			public void execute() {
				power = secondSpindexerPIDF.getOutput(getPosition(), targetTicks);
				spindexerMotor.setPower(power);
			}

			@Override
			public void end(boolean interrupted) {
				spindexerMotor.setPower(0);
				isAtTarget = true;
			}

			@Override
			public boolean isFinished() {
				return Math.abs(targetTicks - getPosition()) < 150 || overrided;
			}
		};
	}

	public Command NewNextTarget() {
		return new CommandBase() {




			@Override
			public void initialize() {
//				double currentDegrees = getDegrees();
//
//				double nextTarget = SpindexerPosition.getNextIntakePosition((int) currentDegrees);
//				prevTarget = nextTarget;
//
//				targetDegrees = nextTarget - 20; // We subtract 20 as our algorithm overshoots (by design)
//				targetTicks = targetDegrees * TICKS_PER_DEGREE;

				target = getDistanceFromSlot(Nextslot) + getDegrees();


				isAtTarget = false;
				overrided = false;

				//spindexerPIDF.setSetpoint(-getDistanceFromSlot(Nextslot));
				secondSpindexerPIDF.setSetpointRange(SLOT_TOLERANCE);
			}

			@Override
			public void execute() {
				if(Math.abs(target - getDegrees()) > 60){
					if(getDegrees() > target){
						power = secondSpindexerPIDF.getOutput(getDegrees(), target);
					}else{
						power = 0.8;
					}
				}else if(Math.abs(target - getDegrees()) < SLOT_TOLERANCE/2){
					power = 0;
				}
				else {
					power = secondSpindexerPIDF.getOutput(getDegrees(), target);
				}
				spindexerMotor.setPower(power);
				stop = true;

			}

			@Override
			public void end(boolean interrupted) {
				spindexerMotor.setPower(0);
				isAtTarget = true;
				Nextslot++;
				if(Nextslot > 2) Nextslot = 0;
				stop = false;
			}

			@Override
			public boolean isFinished() {
				return (Math.abs(spindexerMotor.getVelocity()) == 0 & Math.abs(target - getDegrees()) < SLOT_TOLERANCE) || overrided;
			}
		};
	}

//	public void Telemetry(Telemetry telemetry) {
//		double currentPosTicks = getPosition();
//
//		telemetry.addData("Spindexer Position (degrees)", currentPosTicks * DEGREES_PER_TICK);
//		telemetry.addData("Spindexer Target (degrees)", targetDegrees);
//		telemetry.addData("Spindexer Power", power);
//		telemetry.addData("Tick Error", targetTicks - currentPosTicks);
//		telemetry.addData("Current Slot", getCurrentSlot());
//	}

	public void Telemetry(TelemetryManager telemetry) {
		double currentPosTicks = getPosition();

		telemetry.addData("Spindexer Position (degrees)", currentPosTicks * DEGREES_PER_TICK);
		telemetry.addData("Spindexer Target (degrees)", target);
		telemetry.addData(" Slot", Nextslot);
		telemetry.addData("stop",stop);
		telemetry.addData("velo",spindexerMotor.getVelocity());
		telemetry.addData("Distance from Slot", getDistanceFromSlot(Nextslot));
		telemetry.addData("Aligned with Slot", isAlignedWithSlot() ? "Yeperoosie" : "Nuh uh");
		telemetry.addData("Spindexer Power", power);
		telemetry.addData("Tick Error", targetTicks - currentPosTicks);
		telemetry.addData("Current Slot", getCurrentSlot());
	}

	public void Telemetry(TelemetryManager.TelemetryWrapper telemetry) {
		double currentPosTicks = getPosition();

		telemetry.addData("Spindexer Position (degrees)", currentPosTicks * DEGREES_PER_TICK);
		telemetry.addData("Spindexer Target (degrees)", SpindexerPosition.getNextIntakePosition((int)getDegrees()));
		telemetry.addData(" Slot", Nextslot);
		telemetry.addData("velo",spindexerMotor.getVelocity());
		telemetry.addData("Distance from Slot", getDistanceFromSlot(Nextslot));
		telemetry.addData("Aligned with Slot", isAlignedWithSlot() ? "Yeperoosie" : "Nuh uh");
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
