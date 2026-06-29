package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
	public static final double FORWARD_SPEED = -1;
	public static final double REVERSE_SPEED = 1;

	private final DcMotorEx motor;

	public Conveyor(HardwareMap hardwareMap) {
		motor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
		motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	public Command In() {
		return new InstantCommand(() -> motor.setPower(FORWARD_SPEED), this);
	}

	public Command Out() {
		return new InstantCommand(() -> motor.setPower(REVERSE_SPEED), this);
	}

	public Command Stop() {
		return new InstantCommand(() -> motor.setPower(0), this);
	}

	public Command DirectPower(double power) {
		return new InstantCommand(() -> motor.setPower(power), this);
	}
}
