package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.BeamBreaker;
import org.firstinspires.ftc.teamcode.Utils.RobotConfig;

@TeleOp(name = "Beam Breaker Test", group = "Test")
public class BeamBreakerTest extends OpMode {
	BeamBreaker beamBreaker;

	@Override
	public void init() {
		if (!RobotConfig.COMPETITION) {
			telemetry.addData("Status", "Waiting for start");
			telemetry.update();
		}
		beamBreaker = new BeamBreaker(hardwareMap);
	}

	@Override
	public void loop() {
		boolean isBeamBroken = beamBreaker.isBeamBroken();
		String beamStatus = isBeamBroken ? "Closed" : "Open";

		if (!RobotConfig.COMPETITION) {
			telemetry.addData("Beam Status", beamStatus);
			telemetry.addData("Beam Broken", isBeamBroken);
			telemetry.update();
		}

	}
}
