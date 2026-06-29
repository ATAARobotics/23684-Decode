package org.firstinspires.ftc.teamcode.Subsystem;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

@Configurable
public class Transfer extends SubsystemBase {
	public static double SHOOTER_RPM_TOLERANCE = 150;

	public final CRServo transferLeft;
	public final CRServo transferRight;
	public final CRServo intakeDoorRight;
	public final CRServo intakeDoorLeft;
	public boolean runAutomaticTransfer = false;
	public boolean reachedUpperTarget;
	public boolean reachedLowerTarget;
	public boolean reachedAverageTarget;
	private Shooter shooter;

	public Transfer(HardwareMap hardwareMap) {
		transferLeft = hardwareMap.get(CRServo.class, "transferLeft");
		transferRight = hardwareMap.get(CRServo.class, "transferRight");
		transferRight.setDirection(DcMotorSimple.Direction.REVERSE);
		intakeDoorRight = hardwareMap.get(CRServo.class, "intakeDoorRight");
		intakeDoorLeft = hardwareMap.get(CRServo.class, "intakeDoorLeft");
		intakeDoorRight.setDirection(DcMotorSimple.Direction.REVERSE);
	}

	@Override
	public void periodic() {
		updateTargetFlags();
		if (runAutomaticTransfer) {
			updateAutomaticTransfer(false);
		}
	}

	public void updateTargetFlags() {
		if (shooter != null) {
			reachedUpperTarget = (shooter.upperTarget >= 100) && (Math.abs(shooter.upperRPM - shooter.upperTarget) <= SHOOTER_RPM_TOLERANCE);
			reachedLowerTarget = (shooter.lowerTarget >= 100) && (Math.abs(shooter.lowerRPM - shooter.lowerTarget) <= SHOOTER_RPM_TOLERANCE);
			reachedAverageTarget = reachedUpperTarget && reachedLowerTarget;
		}
	}

	public void setShooter(Shooter shooter) {
		this.shooter = shooter;
	}

	public Command IntakeDoorIn() {
		return new InstantCommand(() -> {
			intakeDoorLeft.setPower(0.5);
			intakeDoorRight.setPower(1);
		}, this
		);
	}

	public Command IntakeDoorOut() {
		return new InstantCommand(() -> {
			intakeDoorLeft.setPower(-0.5);
			intakeDoorRight.setPower(-1);
		}, this
		);
	}

	public Command IntakeDoorStop() {
		return new InstantCommand(() -> {
			intakeDoorLeft.setPower(0);
			intakeDoorRight.setPower(0);
		}, this
		);
	}

	public Command TransferIn() {
		return new InstantCommand(() -> {
			transferLeft.setPower(-1);
			transferRight.setPower(-1);
		}, this
		);
	}

	public Command TransferOut() {
		return new InstantCommand(() -> {
			transferLeft.setPower(1);
			transferRight.setPower(1);
		}, this
		);
	}

	public Command TransferStop() {
		return new InstantCommand(() -> {
			transferLeft.setPower(0);
			transferRight.setPower(0);
		}, this
		);
	}

	public void updateAutomaticTransfer(boolean passive) {
		if (shooter != null) {
			if (reachedUpperTarget && reachedLowerTarget) {
				transferLeft.setPower(1);
				transferRight.setPower(1);
			} else if (!passive) {
				transferLeft.setPower(-1);
				transferRight.setPower(-1);
			}
		}
	}
}
