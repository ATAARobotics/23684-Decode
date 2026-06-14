package org.firstinspires.ftc.teamcode.Subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;

public class Gate extends SubsystemBase {

	Servo gateServo;

	public Gate(HardwareMap hw) {
		gateServo = hw.get(Servo.class, "gateServo");
	}

	public Command openGate() {
		return new InstantCommand(() -> gateServo.setPosition(0), this);
	}

	public Command closeGate() {
		return new InstantCommand(() -> gateServo.setPosition(1), this);
	}

}
