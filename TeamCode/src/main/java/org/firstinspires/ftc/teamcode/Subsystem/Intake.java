package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Intake extends SubsystemBase {
	DcMotor intake;

	double INSPEED = 1;
	double OUTSPEED = -1 ;
	double SLOWSPEED = 0.5;
	double SLOWSPEEDOUT = -0.1;

	public Intake(HardwareMap hardwareMap) {
		intake = hardwareMap.get(DcMotor.class, "intake");
		intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

	}

	public Command In() {
		return new InstantCommand(
				() -> intake.setPower(INSPEED), this
		);
	}

	public Command Slow() {
		return new InstantCommand(
				() -> intake.setPower(SLOWSPEED), this
		);
	}

	public Command SlowOut() {
		return new InstantCommand(
				() -> intake.setPower(SLOWSPEEDOUT), this
		);
	}



	public Command Out() {
		return new InstantCommand(
				() -> intake.setPower(OUTSPEED), this
		);
	}

	public Command Stop() {
		return new InstantCommand(
				() -> intake.setPower(0), this
		);
	}
}
