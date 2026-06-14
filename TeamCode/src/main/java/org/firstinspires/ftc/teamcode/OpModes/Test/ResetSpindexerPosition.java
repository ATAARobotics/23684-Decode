package org.firstinspires.ftc.teamcode.OpModes.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.RobotPosition;

@TeleOp(name = "Reset Spindexer Position", group = "Test")
public class ResetSpindexerPosition extends LinearOpMode {
	@Override
	public void runOpMode() {
		RobotPosition.spindexerTicks = 0;
		RobotPosition.isSpindexerSet = false;
		telemetry.addData("Status", "Spindexer Position Reset!");
		telemetry.update();
		sleep(1000);
	}
}
