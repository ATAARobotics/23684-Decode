package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystem.BeamBreaker;

@TeleOp(name = "Beam Breaker Test", group = "Test")
public class BeamBreakerTest extends LinearOpMode {

	@Override
	public void runOpMode() {
		BeamBreaker beamBreaker = new BeamBreaker(hardwareMap);

		telemetry.addData("Status", "Waiting for start");
		telemetry.update();

		waitForStart();

		while (opModeIsActive()) {
			boolean isBeamBroken = beamBreaker.isBeamBroken();
			String beamStatus = isBeamBroken ? "Closed" : "Open";

			telemetry.addData("Beam Status", beamStatus);
			telemetry.addData("Beam Broken", isBeamBroken);
			telemetry.update();
		}
	}
}
