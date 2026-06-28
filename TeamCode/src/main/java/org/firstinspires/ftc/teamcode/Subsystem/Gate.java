package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Gate extends SubsystemBase {

	public static final double OPEN_POSITION = 0;
	public static final double CLOSE_POSITION = 1;

	Servo gateServo;

	public Gate(HardwareMap hw) {
		gateServo = hw.get(Servo.class, "gateServo");
	}

	public Command openGate() {
		return new InstantCommand(() -> gateServo.setPosition(OPEN_POSITION), this);
	}

	public Command closeGate() {
		return new InstantCommand(() -> gateServo.setPosition(CLOSE_POSITION), this);
	}

	public double getCurrentPosition() {
		return gateServo.getPosition();
	}

}
