package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Spindexer Digital", group = "Test")
public class SpindexerDigital extends OpMode {

	DigitalChannel spindexer;

	@Override
	public void init() {

		spindexer = hardwareMap.get(DigitalChannel.class, "spindexerDigital");
	}

	@Override
	public void loop() {
		telemetry.addData("Digital State", spindexer.getState());
		telemetry.addData("Digital State", spindexer.getMode());

		if (gamepad1.a) {
			spindexer.setMode(DigitalChannel.Mode.OUTPUT);
		}
		if (gamepad1.b) {
			spindexer.setMode(DigitalChannel.Mode.INPUT);
		}

	}
}
